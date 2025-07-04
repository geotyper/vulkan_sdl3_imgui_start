#include "GraphicsModule.h"
#include "volk.h"
#include <imgui.h>
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_vulkan.h"

#include "RayTracingModule.h" // Full definition of RayTracingModule now included
#include <SDL3/SDL_vulkan.h>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <algorithm>
#include "GeomCreate.h"
#include <fstream>


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

void GraphicsModule::Initialize(const std::string appName) {

    initSDL();
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


void GraphicsModule::initSDL() {

    if (!SDL_Init(SDL_INIT_VIDEO))
        throw std::runtime_error(std::string("Failed to initialize SDL3: ") + SDL_GetError());

    m_window = SDL_CreateWindow("Vulkan raytracer exp", 800, 600, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    std::cout << "SDL Window created at: " << m_window << std::endl;
    assert(m_window && "SDL_CreateWindow returned NULL");
    if (!m_window)
        throw std::runtime_error("Failed to create SDL3 window");
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
        //createImGuiFramebuffers()
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
    // Guard: prevent double-initialization
    if (m_device != VK_NULL_HANDLE || m_swapchain != VK_NULL_HANDLE || m_instance != VK_NULL_HANDLE) {
        throw std::runtime_error("[GraphicsModule] initVulkan called twice or already initialized!");
    }

    // Optional: assign instance ID for debug
    static int s_instanceCounter = 0;
    instanceId = ++s_instanceCounter;
    std::cout << "[GraphicsModule] initVulkan() called for instance ID: " << instanceId << " @ " << this << std::endl;

    VK_CHECK(volkInitialize(), "Failed to initialize Volk");

    createInstance(appName);
    volkLoadInstance(m_instance);

    setupDebugMessenger();
    createSurface();
    std::cout << "[Vulkan] Surface created: " << m_surface << std::endl;

    pickPhysicalDevice();
    findQueueFamilies();
    createLogicalDevice();
    volkLoadDevice(m_device);

    createCommandPool();
    createSwapchain();
    //createImageViews();
    //createRenderPass();

    //createFramebuffers();

    //createGraphicsPipeline();
    initImgui();  // sets up descriptor pool, context, SDL bridge, etc.

    std::cout << "Window @GraphicsModule: " << m_window << std::endl;

    createSyncObjects();
}


/* ---------------------- ImGui helpers ----------------------- */
void GraphicsModule::createImGuiDescriptorPool()
{
    VkDescriptorPoolSize poolSizes[] = {
                                        { VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 },
                                        { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         1000 },
                                        { VK_DESCRIPTOR_TYPE_SAMPLER,                1000 },
                                        };
    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.flags         = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets       = 1000 * (uint32_t)std::size(poolSizes);
    poolInfo.poolSizeCount = (uint32_t)std::size(poolSizes);
    poolInfo.pPoolSizes    = poolSizes;
    VK_CHECK(vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &m_imguiPool),
             "ImGui descriptor pool");
}

void GraphicsModule::createImGuiRenderPass()
{
    VkAttachmentDescription color{};
    color.format         = m_swapchainFormat;
    color.samples        = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp         = VK_ATTACHMENT_LOAD_OP_LOAD;        // сохраняем RT-картинку
    color.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    color.initialLayout  = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference ref{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments    = &ref;

    VkRenderPassCreateInfo rp{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    rp.attachmentCount = 1;
    rp.pAttachments    = &color;
    rp.subpassCount    = 1;
    rp.pSubpasses      = &subpass;
    VK_CHECK(vkCreateRenderPass(m_device, &rp, nullptr, &m_imguiRenderPass),
             "ImGui render pass");

    /* framebuffers */
    m_imguiFramebuffers.resize(m_swapchainImageViews.size());
    for (size_t i = 0; i < m_swapchainImageViews.size(); ++i)
    {
        VkImageView att[]{ m_swapchainImageViews[i] };
        VkFramebufferCreateInfo fb{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        fb.renderPass      = m_imguiRenderPass;
        fb.attachmentCount = 1;
        fb.pAttachments    = att;
        fb.width  = m_swapchainExtent.width;
        fb.height = m_swapchainExtent.height;
        fb.layers = 1;
        VK_CHECK(vkCreateFramebuffer(m_device, &fb, nullptr, &m_imguiFramebuffers[i]),
                 "ImGui framebuffer");
    }
}

/* ---------------------- initImGui --------------------------- */
void GraphicsModule::initImgui()
{
    createImGuiDescriptorPool();     // <--
    createImGuiRenderPass();         // <--

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplVulkan_LoadFunctions(
        VK_API_VERSION_1_3,
        /* loader_func = */ [](const char* name, void* user_data) -> PFN_vkVoidFunction
        {
            VkInstance inst = reinterpret_cast<VkInstance>(user_data);
            return vkGetInstanceProcAddr(inst, name);
        },
        /* user_data = */ m_instance);

    ImGui_ImplVulkan_InitInfo info{};
    info.Instance       = m_instance;
    info.PhysicalDevice = m_physicalDevice;
    info.Device         = m_device;
    info.QueueFamily    = m_graphicsQueueFamilyIndex;
    info.Queue          = m_graphicsQueue;
    info.DescriptorPool = m_imguiPool;
    info.MinImageCount  = std::max(2u, (uint32_t)m_swapchainImages.size());
    info.ImageCount     = (uint32_t)m_swapchainImages.size();
    info.MSAASamples    = VK_SAMPLE_COUNT_1_BIT;   // ОБЯЗАТЕЛЬНО
    info.Subpass        = 0;
    info.RenderPass     = m_imguiRenderPass;

    info.CheckVkResultFn = +[](VkResult err)
    {
        if (err) std::cerr << "[ImGui/Vk] error = " << err << std::endl;
    };


    ImGui_ImplSDL3_InitForVulkan(m_window);
    ImGui_ImplVulkan_Init(&info);

   // /* шрифты */
   // VkCommandBuffer cmd = beginOneTimeCommands();
   // ImGui_ImplVulkan_CreateFontsTexture(cmd);
   // endOneTimeCommands(cmd);
   // ImGui_ImplVulkan_DestroyFontUploadObjects();
}


void GraphicsModule::createImGuiFramebuffers()
{
    VkAttachmentDescription color{};
    color.format         = m_swapchainFormat;
    color.samples        = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp         = VK_ATTACHMENT_LOAD_OP_LOAD;              // Keep raytraced content
    color.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout  = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    color.finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout     = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint    = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments    = &colorRef;

    // Optional: subpass dependency (recommended for ImGui)
    VkSubpassDependency dependency{};
    dependency.srcSubpass    = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass    = 0;
    dependency.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo rpInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    rpInfo.attachmentCount = 1;
    rpInfo.pAttachments    = &color;
    rpInfo.subpassCount    = 1;
    rpInfo.pSubpasses      = &subpass;
    rpInfo.dependencyCount = 1;
    rpInfo.pDependencies   = &dependency;

    VK_CHECK(vkCreateRenderPass(m_device, &rpInfo, nullptr, &m_imguiRenderPass),
             "Failed to create ImGui render pass");
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
    //GeomCreate::createUVSphere(32,32, vertices, indices);

    GeomCreate::createIcosphere(4, vertices, indices); // 4 подразделения для гладкости

    std::vector<glm::mat4> transforms;
    const float spacing = 2.5f; // Distance between spheres
    const float scale = 1.1f;   // Reduce size
    for (int z = -2; z <= 2; ++z) {
        for (int y = -2; y <= 2; ++y) {
            for (int x = -2; x <= 2; ++x) {

                if(x==0 and y==0)
                    continue;
                glm::vec3 position = glm::vec3(x * spacing, y * spacing, z * spacing);
                glm::mat4 model = glm::translate(glm::mat4(1.0f), position);
                model = glm::scale(model, ((z+1)/2.0f)*glm::vec3(scale));
                transforms.push_back(model);
            }
        }
    }

    // 5. Load the generated data into the ray tracing module
    m_rtxModule->LoadFromSingleMesh(vertices, indices, transforms);
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

    // 1. Record ray tracing rendering to swapchain image
    m_rtxModule->RecordCommands(
        cmd,
        m_swapchainImageViews[imageIndex],
        m_swapchainImages[imageIndex],
        m_swapchainExtent
        );

    // 2. Record ImGui UI rendering pass
    //imguiModule.renderMenu(cmd);

    // 3. End the command buffer
    VK_CHECK(vkEndCommandBuffer(cmd), "Failed to record command buffer");
}

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

void GraphicsModule::createSurface() {
    if (!SDL_Vulkan_CreateSurface(m_window, m_instance, nullptr, &m_surface)) {
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
            m_graphicsQueueFamilyIndex = i;
            return;
        }
    }
    throw std::runtime_error("Failed to find a queue family supporting graphics.");
}


void GraphicsModule::createLogicalDevice() {
    VkDeviceQueueCreateInfo queueCreateInfo{};
    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueCreateInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
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
        VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
    };

    VkDeviceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    createInfo.pNext = &deviceFeatures2;
    createInfo.queueCreateInfoCount = 1;
    createInfo.pQueueCreateInfos = &queueCreateInfo;
    createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    createInfo.ppEnabledExtensionNames = deviceExtensions.data();

    VK_CHECK(vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device), "Failed to create logical device!");
    vkGetDeviceQueue(m_device, m_graphicsQueueFamilyIndex, 0, &m_graphicsQueue);
}


void GraphicsModule::createCommandPool() {
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    poolInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
    VK_CHECK(vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool), "Failed to create command pool");

    //VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    //poolInfo.queueFamilyIndex = m_graphicsQueueFamilyIndex;
    //poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    //if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_commandPool) != VK_SUCCESS)
    //    throw std::runtime_error("Failed to create command pool");

    //m_commandBuffers.resize(m_swapchainImageViews.size());

    //VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    //allocInfo.commandPool = m_commandPool;
    //allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    //allocInfo.commandBufferCount = static_cast<uint32_t>(m_commandBuffers.size());

    //if (vkAllocateCommandBuffers(m_device, &allocInfo, m_commandBuffers.data()) != VK_SUCCESS)
    //    throw std::runtime_error("Failed to allocate command buffers");
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

void GraphicsModule::createImageViews() {
    m_swapchainImageViews.resize(m_swapchainImages.size());
    for (size_t i = 0; i < m_swapchainImages.size(); ++i) {
        VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = m_swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = m_swapchainFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(m_device, &viewInfo, nullptr, &m_swapchainImageViews[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create swapchain image views");
    }
}

void GraphicsModule::createRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = m_swapchainFormat ;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo renderPassInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO };
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 1;
    renderPassInfo.pDependencies = &dependency;

    if (vkCreateRenderPass(m_device, &renderPassInfo, nullptr, &m_renderPass) != VK_SUCCESS)
        throw std::runtime_error("Failed to create render pass");
}

void GraphicsModule::createFramebuffers() {
    m_swapchainFramebuffers.resize(m_swapchainImageViews.size());

    for (size_t i = 0; i < m_swapchainImageViews.size(); ++i) {
        VkImageView attachments[] = { m_swapchainImageViews[i] };

        VkFramebufferCreateInfo framebufferInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        framebufferInfo.renderPass = m_renderPass;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = attachments;
        framebufferInfo.width = m_swapchainExtent.width;
        framebufferInfo.height = m_swapchainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(m_device, &framebufferInfo, nullptr, &m_swapchainFramebuffers[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create framebuffer");
    }
}


void GraphicsModule::createGraphicsPipeline() {
    auto readFile = [](const std::string& path) -> std::vector<char> {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open())
            throw std::runtime_error("Failed to open file: " + path);

        size_t size = (size_t)file.tellg();
        if (size == static_cast<size_t>(-1))
            throw std::runtime_error("Failed to get file size: " + path);

        std::vector<char> buffer(size);
        file.seekg(0);
        file.read(buffer.data(), size);
        return buffer;
    };

    std::string shaderPath = SHADER_PATH_GLSL;
    auto vertShaderCode = readFile(shaderPath + "sphere.vert.spv");
    auto fragShaderCode = readFile(shaderPath + "sphere.frag.spv");

    VkShaderModuleCreateInfo createInfo{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    createInfo.codeSize = vertShaderCode.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(vertShaderCode.data());
    VkShaderModule vertModule;
    vkCreateShaderModule(m_device, &createInfo, nullptr, &vertModule);

    createInfo.codeSize = fragShaderCode.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(fragShaderCode.data());
    VkShaderModule fragModule;
    vkCreateShaderModule(m_device, &createInfo, nullptr, &fragModule);

    VkPipelineShaderStageCreateInfo vertStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    vertStage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vertStage.module = vertModule;
    vertStage.pName = "main";

    VkPipelineShaderStageCreateInfo fragStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    fragStage.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fragStage.module = fragModule;
    fragStage.pName = "main";

    VkPipelineShaderStageCreateInfo shaderStages[] = { vertStage, fragStage };

    auto binding = GeomCreate::getBindingDescription2();
    auto attributes = GeomCreate::getAttributeDescriptions2();

    VkPipelineVertexInputStateCreateInfo vertexInput{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
    vertexInput.pVertexAttributeDescriptions = attributes.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO };
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    inputAssembly.primitiveRestartEnable = VK_FALSE;

    VkViewport viewport{ 0.0f, 0.0f, (float)m_swapchainExtent.width, (float)m_swapchainExtent.height, 0.0f, 1.0f };
    VkRect2D scissor{ {0, 0}, m_swapchainExtent };
    VkPipelineViewportStateCreateInfo viewportState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO };
    viewportState.viewportCount = 1;
    viewportState.pViewports = &viewport;
    viewportState.scissorCount = 1;
    viewportState.pScissors = &scissor;

    VkPipelineRasterizationStateCreateInfo rasterizer{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO };
    rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
    rasterizer.lineWidth = 1.0f;
    rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;

    VkPipelineMultisampleStateCreateInfo multisampling{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO };
    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                          VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;

    VkPipelineColorBlendStateCreateInfo colorBlending{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO };
    colorBlending.attachmentCount = 1;
    colorBlending.pAttachments = &colorBlendAttachment;

    VkPushConstantRange pushConstant{};
    pushConstant.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushConstant.offset = 0;
    pushConstant.size = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstant;

    vkCreatePipelineLayout(m_device, &pipelineLayoutInfo, nullptr, &m_pipelineLayout);

    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.layout = m_pipelineLayout;
    pipelineInfo.renderPass = m_renderPass;
    pipelineInfo.subpass = 0;

    vkCreateGraphicsPipelines(m_device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_graphicsPipeline);

    vkDestroyShaderModule(m_device, fragModule, nullptr);
    vkDestroyShaderModule(m_device, vertModule, nullptr);
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

