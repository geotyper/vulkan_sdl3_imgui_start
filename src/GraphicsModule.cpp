#include "GraphicsModule.h"
#include "RayTracingModule.h" // Full definition of RayTracingModule now included
#include "framework/vulkanhelpers.h"
#include <volk.h>
#include <SDL3/SDL_vulkan.h>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <algorithm>

#define VK_CHECK(x, msg)                                            \
do {                                                            \
        VkResult err = (x);                                         \
        if (err != VK_SUCCESS) {                                    \
            throw std::runtime_error(std::string(msg) + " failed: " + std::to_string(err)); \
    }                                                           \
} while (0)

    // Forward declaration for the debug callback
    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
        VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
        VkDebugUtilsMessageTypeFlagsEXT messageType,
        const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
        void* pUserData);


// --- Constructor and Destructor Definitions ---
// These MUST be defined in the .cpp file where RayTracingModule is a complete type.
GraphicsModule::GraphicsModule() = default;
GraphicsModule::~GraphicsModule() = default;


// --- Public API Implementations ---

void GraphicsModule::Initialize(SDL_Window* window, const std::string& appName) {
    m_window = window;
    initVulkan(appName);
    initRayTracingModule();
}

void GraphicsModule::Shutdown() {
    // Если устройство уже было уничтожено, выходим
    if (!m_device) return;

    // 1. Убеждаемся, что GPU закончил все операции.
    vkDeviceWaitIdle(m_device);

    // 2. Уничтожаем все ресурсы, созданные нашими модулями.
    //    unique_ptr сделает это автоматически при выходе из области видимости,
    //    но явный вызов Cleanup более нагляден.
    if (m_rtxModule) {
        m_rtxModule->Cleanup();
        m_rtxModule.reset(); // Уничтожаем объект
    }

    // 3. Уничтожаем все объекты, связанные со Swapchain.
    cleanupSwapchain();

    // 4. Уничтожаем объекты синхронизации и командный пул.
    //    Они были созданы из VkDevice.
    for (size_t i = 0; i < m_inFlightFences.size(); i++) {
        vkDestroySemaphore(m_device, m_renderFinishedSemaphores[i], nullptr);
        vkDestroySemaphore(m_device, m_imageAvailableSemaphores[i], nullptr);
        vkDestroyFence(m_device, m_inFlightFences[i], nullptr);
    }

    if (m_commandPool) {
        vkDestroyCommandPool(m_device, m_commandPool, nullptr);
    }

    // 5. После того как все дочерние объекты уничтожены, уничтожаем само логическое устройство.
    vkDestroyDevice(m_device, nullptr);
    m_device = VK_NULL_HANDLE; // Обнуляем хендл

    // 6. Теперь уничтожаем объекты уровня Instance, которые зависели от него.
    if (m_debugMessenger) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func) {
            func(m_instance, m_debugMessenger, nullptr);
        }
    }

    if (m_surface) {
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
    }

    // 7. В самую последнюю очередь уничтожаем сам инстанс Vulkan.
    if (m_instance) {
        vkDestroyInstance(m_instance, nullptr);
        m_instance = VK_NULL_HANDLE;
    }
}

void GraphicsModule::RenderFrame(const Camera& cam) {
    vkWaitForFences(m_device, 1, &m_inFlightFences[m_currentFrame], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(m_device, m_swapchain, UINT64_MAX, m_imageAvailableSemaphores[m_currentFrame], VK_NULL_HANDLE, &imageIndex);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_framebufferResized) {
        m_framebufferResized = false;
        recreateSwapchain();
        return;
    }
    VK_CHECK(result, "Failed to acquire swap chain image");

    m_rtxModule->UpdateCamera(cam);

    vkResetFences(m_device, 1, &m_inFlightFences[m_currentFrame]);
    vkResetCommandBuffer(m_commandBuffers[m_currentFrame], 0);
    recordCommandBuffer(imageIndex, cam);

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = { m_imageAvailableSemaphores[m_currentFrame] };
    VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &m_commandBuffers[m_currentFrame];

    VkSemaphore signalSemaphores[] = { m_renderFinishedSemaphores[m_currentFrame] };
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    VK_CHECK(vkQueueSubmit(m_graphicsQueue, 1, &submitInfo, m_inFlightFences[m_currentFrame]), "Failed to submit draw command buffer");

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &m_swapchain;
    presentInfo.pImageIndices = &imageIndex;

    result = vkQueuePresentKHR(m_graphicsQueue, &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        m_framebufferResized = false;
        recreateSwapchain();
    } else if (result != VK_SUCCESS) {
        VK_CHECK(result, "Failed to present swap chain image");
    }

    m_currentFrame = (m_currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}


void GraphicsModule::SignalResize() {
    m_framebufferResized = true;
}


// --- Private Method Implementations ---

void GraphicsModule::initVulkan(const std::string& appName) {
    VK_CHECK(volkInitialize(), "Failed to initialize Volk");

    createInstance(appName);
    volkLoadInstance(m_instance);

    setupDebugMessenger();
    createSurface(m_window);
    pickPhysicalDevice();
    findQueueFamilies();
    createLogicalDevice();

    volkLoadDevice(m_device);

    createCommandPool();
    createSwapchain();
    createSyncObjects();
}

void GraphicsModule::initRayTracingModule() {

    vulkanhelpers::VulkanContext context;
    context.device = m_device;
    context.physicalDevice = m_physicalDevice;
    context.commandPool = m_commandPool;
    context.transferQueue = m_graphicsQueue;
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &context.physicalDeviceMemoryProperties);

    m_rtxModule = std::make_unique<rtx::RayTracingModule>();
    rtx::RayTracingModule::CreateInfo ci{};
    ci.device = m_device;
    ci.physicalDevice = m_physicalDevice;
    ci.commandPool = m_commandPool;
    ci.graphicsQueue = m_graphicsQueue;
    ci.shaderDir = "shaders/";
    m_rtxModule->Initialize(context, ci);

    //m_rtxModule->LoadScene("assets/CornellBox-Original.obj");

    // 3. Create vectors to hold your geometry data
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    // 4. Call your generator to fill the vectors
    // You can use any of your functions here: createUVSphere, createIcosphere, etc.
    //GeomCreate::createIcosphere(2, vertices, indices);
    GeomCreate::createUVSphere(32,32, vertices, indices);

    // 5. Load the generated data into the ray tracing module
    m_rtxModule->LoadFromVerticesAndIndices(vertices, indices);
}


void GraphicsModule::recreateSwapchain() {
    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(m_window, &width, &height);
    while (width == 0 || height == 0) {
        SDL_GetWindowSizeInPixels(m_window, &width, &height);
        SDL_WaitEvent(nullptr);
    }

    vkDeviceWaitIdle(m_device);
    cleanupSwapchain();
    createSwapchain();
    if (m_rtxModule) {
        m_rtxModule->OnResize(m_swapchainExtent);
    }
}

void GraphicsModule::cleanupSwapchain() {
    for (auto imageView : m_swapchainImageViews) {
        vkDestroyImageView(m_device, imageView, nullptr);
    }
    m_swapchainImageViews.clear();
    if (m_swapchain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(m_device, m_swapchain, nullptr);
        m_swapchain = VK_NULL_HANDLE;
    }
}


void GraphicsModule::recordCommandBuffer(uint32_t imageIndex, const Camera& cam) {
    VkCommandBuffer cmd = m_commandBuffers[m_currentFrame];
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    VK_CHECK(vkBeginCommandBuffer(cmd, &beginInfo), "Failed to begin recording command buffer");

    m_rtxModule->RecordCommands(cmd, m_swapchainImageViews[imageIndex], m_swapchainImages[imageIndex], m_swapchainExtent);

    VK_CHECK(vkEndCommandBuffer(cmd), "Failed to record command buffer");
}

#include <array>
#include <vector>
#include <SDL3/SDL_vulkan.h>
#include <volk.h>
#include "GraphicsModule.h"
#include "framework/vulkanhelpers.h"

/* ------------------------------------------------------------
 * GraphicsModule::createInstance
 * ---------------------------------------------------------- */
// GraphicsModule.cpp
void GraphicsModule::createInstance(const std::string& appName)
{
    /* ------------------------------------------------- *
     * 1.  Базовая «визитка» приложения                  *
     * ------------------------------------------------- */
    VkApplicationInfo appInfo{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    appInfo.pApplicationName   = appName.c_str();
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName        = "No Engine";
    appInfo.engineVersion      = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion         = VK_API_VERSION_1_3;


    /* ------------------------------------------------- *
     * 2.  Расширения, которые требуют SDL + Debug       *
     * ------------------------------------------------- */
    uint32_t sdlExtCount = 0;
    const char* const* sdlExt = SDL_Vulkan_GetInstanceExtensions(&sdlExtCount);
    if (!sdlExt)
        throw std::runtime_error("SDL_Vulkan_GetInstanceExtensions failed");

    std::vector<const char*> instExt{ sdlExt, sdlExt + sdlExtCount };

#ifndef NDEBUG
    instExt.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    instExt.push_back(VK_EXT_VALIDATION_FEATURES_EXTENSION_NAME);   // цепочка pNext
#endif


    /* ------------------------------------------------- *
     * 3.  (Debug)  Messenger + validation-features chain *
     * ------------------------------------------------- */
#ifndef NDEBUG
    /* 3.1 Debug messenger */
    VkDebugUtilsMessengerCreateInfoEXT dbgInfo{
                                               VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT };
    dbgInfo.messageSeverity =
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
        VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    dbgInfo.messageType =
        VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT      |
        VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT   |
        VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    dbgInfo.pfnUserCallback = debugCallback;

    /* 3.2 Validation-features (можно на лету конфигурировать) */
    std::vector<VkValidationFeatureEnableEXT> enableList = {
        VK_VALIDATION_FEATURE_ENABLE_BEST_PRACTICES_EXT
    };
#ifdef ENABLE_GPU_ASSISTED             // добавьте -DENABLE_GPU_ASSISTED в cmake/Makefile
    enableList.push_back(VK_VALIDATION_FEATURE_ENABLE_GPU_ASSISTED_EXT);
#endif

    VkValidationFeaturesEXT valFeatures{
                                        VK_STRUCTURE_TYPE_VALIDATION_FEATURES_EXT };
    valFeatures.enabledValidationFeatureCount =
        static_cast<uint32_t>(enableList.size());
    valFeatures.pEnabledValidationFeatures = enableList.data();

    /* сцепляем pNext-цепочку: dbgInfo → valFeatures → nullptr */
    dbgInfo.pNext = &valFeatures;
#endif  // !NDEBUG


    /* ------------------------------------------------- *
     * 4.  Сам VkInstance                                *
     * ------------------------------------------------- */
    const char* validationLayer = "VK_LAYER_KHRONOS_validation";

    VkInstanceCreateInfo ci{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
#ifndef NDEBUG
    ci.pNext = &dbgInfo;                        // отладочная цепочка
    ci.enabledLayerCount       = 1;
    ci.ppEnabledLayerNames     = &validationLayer;
#else
    ci.pNext = nullptr;
    ci.enabledLayerCount       = 0;
    ci.ppEnabledLayerNames     = nullptr;
#endif
    ci.pApplicationInfo        = &appInfo;
    ci.enabledExtensionCount   = static_cast<uint32_t>(instExt.size());
    ci.ppEnabledExtensionNames = instExt.data();

    VK_CHECK(vkCreateInstance(&ci, nullptr, &m_instance),
             "Failed to create Vulkan instance");

    /* ------------------------------------------------- *
     * 5.  Загрузка volk – теперь можно вызывать vk*     *
     * ------------------------------------------------- */
    volkLoadInstance(m_instance);
}



void GraphicsModule::setupDebugMessenger() {
    VkDebugUtilsMessengerCreateInfoEXT createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    createInfo.pfnUserCallback = debugCallback;

    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_instance, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr) {
        VK_CHECK(func(m_instance, &createInfo, nullptr, &m_debugMessenger), "Failed to set up debug messenger");
    }
}

void GraphicsModule::createSurface(SDL_Window* window) {
    if (!SDL_Vulkan_CreateSurface(window, m_instance, nullptr, &m_surface)) {
        throw std::runtime_error("Failed to create window surface: " + std::string(SDL_GetError()));
    }
}


void GraphicsModule::pickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error("Failed to find GPUs with Vulkan support!");
    }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

    for (const auto& device : devices) {
        m_physicalDevice = device;
        return;
    }
    throw std::runtime_error("Failed to find a suitable GPU!");
}


void GraphicsModule::findQueueFamilies() {
    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &queueFamilyCount, queueFamilies.data());

    for (uint32_t i = 0; i < queueFamilies.size(); ++i) {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
            m_graphicsFamilyIndex = i;
            return;
        }
    }
    throw std::runtime_error("Failed to find a queue family supporting graphics.");
}


void GraphicsModule::createLogicalDevice() {
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = m_graphicsFamilyIndex;
    queueCreateInfo.queueCount = 1;
    float queuePriority = 1.0f;
    queueCreateInfo.pQueuePriorities = &queuePriority;

    VkPhysicalDeviceBufferDeviceAddressFeatures bdaFeatures{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES, nullptr, VK_TRUE};
    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtpFeatures{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR, &bdaFeatures, VK_TRUE};
    VkPhysicalDeviceAccelerationStructureFeaturesKHR asFeatures{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR, &rtpFeatures, VK_TRUE};

    VkPhysicalDeviceFeatures2 deviceFeatures2{VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2, &asFeatures};

    std::vector<const char*> deviceExtensions = {
          VK_KHR_SWAPCHAIN_EXTENSION_NAME,
          VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
          VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
          VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
       // VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
    };

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pNext = &deviceFeatures2;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    VK_CHECK(vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device), "Failed to create logical device!");
    vkGetDeviceQueue(m_device, m_graphicsFamilyIndex, 0, &m_graphicsQueue);
}


void GraphicsModule::createCommandPool() {
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_graphicsFamilyIndex;
    VK_CHECK(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool), "Failed to create command pool");
}


void GraphicsModule::createSwapchain() {
    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_physicalDevice, m_surface, &capabilities);

    uint32_t formatCount;
    vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    if(formatCount != 0) {
        vkGetPhysicalDeviceSurfaceFormatsKHR(m_physicalDevice, m_surface, &formatCount, formats.data());
    }

    VkSurfaceFormatKHR surfaceFormat = formats[0];
    for (const auto& availableFormat : formats) {
        if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            surfaceFormat = availableFormat;
            break;
        }
    }
    m_swapchainFormat = surfaceFormat.format;

    if (capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        m_swapchainExtent = capabilities.currentExtent;
    } else {
        int width, height;
        SDL_GetWindowSizeInPixels(m_window, &width, &height);
        m_swapchainExtent.width = std::clamp(static_cast<uint32_t>(width), capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        m_swapchainExtent.height = std::clamp(static_cast<uint32_t>(height), capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    }

    uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount) {
        imageCount = capabilities.maxImageCount;
    }

    VkSwapchainCreateInfoKHR createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    createInfo.surface = m_surface;
    createInfo.minImageCount = imageCount;
    createInfo.imageFormat = m_swapchainFormat;
    createInfo.imageColorSpace = surfaceFormat.colorSpace;
    createInfo.imageExtent = m_swapchainExtent;
    createInfo.imageArrayLayers = 1;
    createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    createInfo.preTransform = capabilities.currentTransform;
    createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    createInfo.clipped = VK_TRUE;

    VK_CHECK(vkCreateSwapchainKHR(m_device, &createInfo, nullptr, &m_swapchain), "Failed to create swap chain");

    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, nullptr);
    m_swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(m_device, m_swapchain, &imageCount, m_swapchainImages.data());

    m_swapchainImageViews.resize(m_swapchainImages.size());
    for (size_t i = 0; i < m_swapchainImages.size(); i++) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        VK_CHECK(vkCreateImageView(m_device, &viewInfo, nullptr, &m_swapchainImageViews[i]), "Failed to create image views");
    }
}


void GraphicsModule::createSyncObjects() {
    m_imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
    m_inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
    m_commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = m_commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = (uint32_t)m_commandBuffers.size();
    VK_CHECK(vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data()), "Failed to allocate command buffers");

    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        VK_CHECK(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_imageAvailableSemaphores[i]), "Sync object creation failed");
        VK_CHECK(vkCreateSemaphore(m_device, &semaphoreInfo, nullptr, &m_renderFinishedSemaphores[i]), "Sync object creation failed");
        VK_CHECK(vkCreateFence(m_device, &fenceInfo, nullptr, &m_inFlightFences[i]), "Sync object creation failed");
    }
}

static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
    VkDebugUtilsMessageTypeFlagsEXT messageType,
    const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
    void* pUserData) {
    std::cerr << "[Vulkan Validation] " << pCallbackData->pMessage << std::endl;
    return VK_FALSE;
}

