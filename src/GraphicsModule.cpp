#include "GraphicsModule.h"
#include <SDL3/SDL_vulkan.h>
#include "backends/imgui_impl_sdl3.h"
#include "VulkanHelperMethods.h"
#include "AccelerationStructures.h"
#include "GeomCreate.h"
#include <fstream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstring>
#include <string>
#include <set>





void GraphicsModule::init() {
    initSDL();
    createVulkanInstance();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapchain();
    createImageViews();
    createRenderPass();
    createCommandPoolAndBuffers();
    createFramebuffers();
    createGraphicsPipeline();

    // --- Ray Tracing Init ---
    accelManager = std::make_unique<AccelerationStructureManager>(
        device, physicalDevice, commandPool, graphicsQueue);
    createRayTracingDescriptorsAndLayout();
    createRayTracingPipeline();
    createOutputImageRayTrace();
}


void GraphicsModule::initSDL() {
    if (!SDL_Init(SDL_INIT_VIDEO))
        throw std::runtime_error(std::string("Failed to initialize SDL3: ") + SDL_GetError());
    window = SDL_CreateWindow("Drone Visualizer", 800, 600, SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    if (!window)
        throw std::runtime_error("Failed to create SDL3 window");
}

void GraphicsModule::createVulkanInstance() {
    VkApplicationInfo appInfo{ VK_STRUCTURE_TYPE_APPLICATION_INFO };
    appInfo.pApplicationName = "DroneVisualizer";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "No Engine";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_3;
    uint32_t sdlExtCount = 0;
    const char* const* sdlExts = SDL_Vulkan_GetInstanceExtensions(&sdlExtCount);
    if (!sdlExts)
        throw std::runtime_error("Failed to get SDL Vulkan extensions");
    std::vector<const char*> extensions(sdlExts, sdlExts + sdlExtCount);
    VkInstanceCreateInfo instInfo{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
    instInfo.pApplicationInfo = &appInfo;
    instInfo.enabledExtensionCount = sdlExtCount;
    instInfo.ppEnabledExtensionNames = extensions.data();
    if (vkCreateInstance(&instInfo, nullptr, &instance) != VK_SUCCESS)
        throw std::runtime_error("Failed to create Vulkan instance");
}

void GraphicsModule::createSurface() {
    if (!SDL_Vulkan_CreateSurface(window, instance, nullptr, &surface)) {
        throw std::runtime_error(std::string("Failed to create Vulkan surface from SDL window: ") + SDL_GetError());
    }
}

void GraphicsModule::pickPhysicalDevice() {
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);
    if (deviceCount == 0) {
        throw std::runtime_error("Failed to find GPUs with Vulkan support!");
    }
    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());
    const std::vector<const char*> requiredDeviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
        VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
        VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
        VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
        VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME
    };
    for (const auto& device : devices) {
        if (!checkDeviceExtensionSupport(device, requiredDeviceExtensions)) {
            continue;
        }
        VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeatures{};
        rtPipelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
        VkPhysicalDeviceAccelerationStructureFeaturesKHR accelStructFeatures{};
        accelStructFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
        rtPipelineFeatures.pNext = &accelStructFeatures;
        VkPhysicalDeviceBufferDeviceAddressFeatures bufferDeviceAddressFeatures{};
        bufferDeviceAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
        accelStructFeatures.pNext = &bufferDeviceAddressFeatures;
        VkPhysicalDeviceFeatures2 deviceFeatures2{};
        deviceFeatures2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
        deviceFeatures2.pNext = &rtPipelineFeatures;
        vkGetPhysicalDeviceFeatures2(device, &deviceFeatures2);
        if (!rtPipelineFeatures.rayTracingPipeline || !accelStructFeatures.accelerationStructure || !bufferDeviceAddressFeatures.bufferDeviceAddress) {
            continue;
        }
        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());
        int foundQueueFamily = -1;
        for (int i = 0; i < queueFamilies.size(); i++) {
            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);
            if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT && presentSupport) {
                foundQueueFamily = i;
                break;
            }
        }
        if (foundQueueFamily != -1) {
            physicalDevice = device;
            graphicsQueueFamilyIndex = foundQueueFamily;
            break;
        }
    }
    if (physicalDevice == VK_NULL_HANDLE) {
        throw std::runtime_error("Failed to find a suitable GPU that supports Ray Tracing!");
    }
}

void GraphicsModule::createLogicalDevice() {
    float queuePriority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{ VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO };
    queueInfo.queueFamilyIndex = graphicsQueueFamilyIndex;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &queuePriority;
    const std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME,
        VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
        VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
        VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
        VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
        VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME
    };
    VkPhysicalDeviceDescriptorIndexingFeatures descriptorIndexingFeatures{};
    descriptorIndexingFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES;
    descriptorIndexingFeatures.runtimeDescriptorArray = VK_TRUE;
    VkPhysicalDeviceBufferDeviceAddressFeatures bufferDeviceAddressFeatures{};
    bufferDeviceAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
    bufferDeviceAddressFeatures.pNext = &descriptorIndexingFeatures;
    bufferDeviceAddressFeatures.bufferDeviceAddress = VK_TRUE;
    VkPhysicalDeviceAccelerationStructureFeaturesKHR accelStructFeatures{};
    accelStructFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
    accelStructFeatures.pNext = &bufferDeviceAddressFeatures;
    accelStructFeatures.accelerationStructure = VK_TRUE;
    VkPhysicalDeviceRayTracingPipelineFeaturesKHR rtPipelineFeatures{};
    rtPipelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
    rtPipelineFeatures.pNext = &accelStructFeatures;
    rtPipelineFeatures.rayTracingPipeline = VK_TRUE;
    VkDeviceCreateInfo devInfo{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
    devInfo.queueCreateInfoCount = 1;
    devInfo.pQueueCreateInfos = &queueInfo;
    devInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    devInfo.ppEnabledExtensionNames = deviceExtensions.data();
    devInfo.pNext = &rtPipelineFeatures;
    if (vkCreateDevice(physicalDevice, &devInfo, nullptr, &device) != VK_SUCCESS)
        throw std::runtime_error("Failed to create logical device");
    vkGetDeviceQueue(device, graphicsQueueFamilyIndex, 0, &graphicsQueue);
    loadRayTracingFunctions(device);
    rayTracingProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
    VkPhysicalDeviceProperties2 deviceProps2{ VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2 };
    deviceProps2.pNext = &rayTracingProperties;
    vkGetPhysicalDeviceProperties2(physicalDevice, &deviceProps2);
}


void GraphicsModule::createSwapchain() {
    VkSurfaceCapabilitiesKHR capabilities;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &capabilities);

    uint32_t formatCount;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());

    VkSurfaceFormatKHR chosenFormat = formats[0];
    for (const auto& availableFormat : formats) {
        if (availableFormat.format == VK_FORMAT_B8G8R8A8_UNORM &&
            availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosenFormat = availableFormat;
            break;
        }
    }

    uint32_t presentModeCount;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, nullptr);
    std::vector<VkPresentModeKHR> presentModes(presentModeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, presentModes.data());

    VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR; // guaranteed available

    swapchainExtent = capabilities.currentExtent;
    swapchainImageFormat = chosenFormat.format;

    VkSwapchainCreateInfoKHR swapchainInfo{ VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR };
    swapchainInfo.surface = surface;
    swapchainInfo.minImageCount = capabilities.minImageCount + 1;
    swapchainInfo.imageFormat = swapchainImageFormat;
    swapchainInfo.imageColorSpace = chosenFormat.colorSpace;
    swapchainInfo.imageExtent = swapchainExtent;
    swapchainInfo.imageArrayLayers = 1;
    swapchainInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    swapchainInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    swapchainInfo.preTransform = capabilities.currentTransform;
    swapchainInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchainInfo.presentMode = presentMode;
    swapchainInfo.clipped = VK_TRUE;
    swapchainInfo.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(device, &swapchainInfo, nullptr, &swapchain) != VK_SUCCESS)
        throw std::runtime_error("Failed to create swapchain");

    uint32_t imageCount = 0;
    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
    swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapchainImages.data());
}

void GraphicsModule::createImageViews() {
    swapchainImageViews.resize(swapchainImages.size());
    for (size_t i = 0; i < swapchainImages.size(); ++i) {
        VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
        viewInfo.image = swapchainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = swapchainImageFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(device, &viewInfo, nullptr, &swapchainImageViews[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create swapchain image views");
    }
}

void GraphicsModule::createRenderPass() {
    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = swapchainImageFormat;
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

    if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS)
        throw std::runtime_error("Failed to create render pass");
}

void GraphicsModule::createCommandPoolAndBuffers() {
    VkCommandPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
    poolInfo.queueFamilyIndex = graphicsQueueFamilyIndex;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS)
        throw std::runtime_error("Failed to create command pool");

    commandBuffers.resize(swapchainImageViews.size());

    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool = commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());

    if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS)
        throw std::runtime_error("Failed to allocate command buffers");
}

void GraphicsModule::createFramebuffers() {
    swapchainFramebuffers.resize(swapchainImageViews.size());

    for (size_t i = 0; i < swapchainImageViews.size(); ++i) {
        VkImageView attachments[] = { swapchainImageViews[i] };

        VkFramebufferCreateInfo framebufferInfo{ VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO };
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = attachments;
        framebufferInfo.width = swapchainExtent.width;
        framebufferInfo.height = swapchainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapchainFramebuffers[i]) != VK_SUCCESS)
            throw std::runtime_error("Failed to create framebuffer");
    }
}

void GraphicsModule::beginFrame() {
    // SDL_PumpEvents(); // SDL_PollEvent in pollEvents() is generally preferred for explicit event handling
}

void GraphicsModule::draw(std::function<void(VkCommandBuffer)> renderCallback) {
    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX, VK_NULL_HANDLE, VK_NULL_HANDLE, &imageIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        m_framebufferResized = true;
        return;
    } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swapchain image!");
    }

    VkCommandBuffer cmd = commandBuffers[imageIndex];
    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &beginInfo);

    VkClearValue clearColor = { {0.0f, 0.0f, 1.0f, 1.0f} };

    VkRenderPassBeginInfo renderPassInfo{ VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO };
    renderPassInfo.renderPass = renderPass;
    renderPassInfo.framebuffer = swapchainFramebuffers[imageIndex];
    renderPassInfo.renderArea.offset = { 0, 0 };
    renderPassInfo.renderArea.extent = swapchainExtent;
    renderPassInfo.clearValueCount = 1;
    renderPassInfo.pClearValues = &clearColor;

    vkCmdBeginRenderPass(cmd, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    if (renderCallback) {
        renderCallback(cmd);
    }

    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;

    vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);

    VkPresentInfoKHR presentInfo{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain;
    presentInfo.pImageIndices = &imageIndex;

    result = vkQueuePresentKHR(graphicsQueue, &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        m_framebufferResized = true;
        return;
    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to present swapchain image!");
    }
}


void GraphicsModule::cleanup() {
    // Ensure all Vulkan operations are finished before cleanup
    if (device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device);
    }

    for (auto framebuffer : swapchainFramebuffers)
        if (framebuffer != VK_NULL_HANDLE) vkDestroyFramebuffer(device, framebuffer, nullptr);
    if (commandPool != VK_NULL_HANDLE)
        vkDestroyCommandPool(device, commandPool, nullptr);
    for (auto view : swapchainImageViews)
        if (view != VK_NULL_HANDLE) vkDestroyImageView(device, view, nullptr);
    if (swapchain != VK_NULL_HANDLE)
        vkDestroySwapchainKHR(device, swapchain, nullptr);
    if (renderPass != VK_NULL_HANDLE)
        vkDestroyRenderPass(device, renderPass, nullptr);
    if (device != VK_NULL_HANDLE)
        vkDestroyDevice(device, nullptr);
    if (surface != VK_NULL_HANDLE)
        vkDestroySurfaceKHR(instance, surface, nullptr);
    if (instance != VK_NULL_HANDLE)
        vkDestroyInstance(instance, nullptr);
    if (window)
        SDL_DestroyWindow(window);

    SDL_Quit();
}

void GraphicsModule::pollEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL3_ProcessEvent(&event); // Forward to ImGui

        switch (event.type) {
        case SDL_EVENT_QUIT:
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            if (event.window.windowID == SDL_GetWindowID(window))
                m_windowShouldClose = true;
            break;

        case SDL_EVENT_WINDOW_RESIZED:
            if (event.window.windowID == SDL_GetWindowID(window))
                m_framebufferResized = true;
            break;

        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (event.button.button == SDL_BUTTON_LEFT)
                mousePressed = true;
            break;

        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (event.button.button == SDL_BUTTON_LEFT)
                mousePressed = false;
            break;

        case SDL_EVENT_MOUSE_MOTION:
            if (mousePressed && !ImGui::GetIO().WantCaptureMouse) {
                int dx = event.motion.x - lastMouseX;
                int dy = event.motion.y - lastMouseY;
                camera.rotate(dx * 0.005f, dy * 0.005f); // Adjust sensitivity
            }
            lastMouseX = event.motion.x;
            lastMouseY = event.motion.y;
            break;

        case SDL_EVENT_MOUSE_WHEEL:
            if (!ImGui::GetIO().WantCaptureMouse) {
                camera.zoom(-event.wheel.y * 0.1f);
            }
            break;

        case SDL_EVENT_KEY_DOWN:
            if (!ImGui::GetIO().WantCaptureKeyboard) {
                switch (event.key.key) {
                case SDLK_Q: camera.roll(0.05f); break;
                case SDLK_E: camera.roll(-0.05f); break;
                }
            }
            break;
        }
    }
}


void GraphicsModule::recreateSwapchain() {
    vkDeviceWaitIdle(device);

    for (auto framebuffer : swapchainFramebuffers)
        vkDestroyFramebuffer(device, framebuffer, nullptr);
    for (auto view : swapchainImageViews)
        vkDestroyImageView(device, view, nullptr);
    if (swapchain)
        vkDestroySwapchainKHR(device, swapchain, nullptr);

    createSwapchain();
    createImageViews();
    createFramebuffers();
}


void GraphicsModule::handleResizeIfNeeded() {
    if (!m_framebufferResized) return;

    // Wait until window has non-zero size
    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(window, &width, &height);
    while (width == 0 || height == 0) {
        SDL_GetWindowSizeInPixels(window, &width, &height);
        SDL_WaitEvent(nullptr);
    }

    recreateSwapchain();
    m_framebufferResized = false;

    camera.setViewport(static_cast<float>(swapchainExtent.width),
                       static_cast<float>(swapchainExtent.height));

}


bool GraphicsModule::shouldClose() const {
    return m_windowShouldClose;
}

bool GraphicsModule::wasFramebufferResized() const {
    return m_framebufferResized;
}

void GraphicsModule::acknowledgeResize() {
    m_framebufferResized = false;
}


VkShaderModule GraphicsModule::createShaderModule(VkDevice device, const uint32_t* code, size_t codeSize) {
    VkShaderModuleCreateInfo createInfo{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    createInfo.codeSize = codeSize;
    createInfo.pCode = code;
    VkShaderModule shaderModule;
    if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create shader module.");
    }
    return shaderModule;
}

void GraphicsModule::createGraphicsPipeline() {
    auto readFile = [](const std::string& path) -> std::vector<char> {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) throw std::runtime_error("Failed to open file: " + path);
        size_t size = (size_t)file.tellg();
        if (size == static_cast<size_t>(-1)) throw std::runtime_error("Failed to get file size: " + path);
        std::vector<char> buffer(size);
        file.seekg(0);
        file.read(buffer.data(), size);
        return buffer;
    };
    std::string shaderPath = SHADER_PATH;
    auto vertShaderCode = readFile(shaderPath + "sphere.vert.spv");
    auto fragShaderCode = readFile(shaderPath + "sphere.frag.spv");
    VkShaderModule vertModule = createShaderModule(device, reinterpret_cast<const uint32_t*>(vertShaderCode.data()), vertShaderCode.size());
    VkShaderModule fragModule = createShaderModule(device, reinterpret_cast<const uint32_t*>(fragShaderCode.data()), fragShaderCode.size());

    VkPipelineShaderStageCreateInfo vertStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vertModule, "main", nullptr };
    VkPipelineShaderStageCreateInfo fragStage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fragModule, "main", nullptr };
    VkPipelineShaderStageCreateInfo shaderStages[] = { vertStage, fragStage };

    auto binding = GeomCreate::getBindingDescription();
    auto attributes = GeomCreate::getAttributeDescriptions();
    VkPipelineVertexInputStateCreateInfo vertexInput{ VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO };
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes.size());
    vertexInput.pVertexAttributeDescriptions = attributes.data();
    VkPipelineInputAssemblyStateCreateInfo inputAssembly{ VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO, nullptr, 0, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, VK_FALSE };
    VkViewport viewport{ 0.0f, 0.0f, (float)swapchainExtent.width, (float)swapchainExtent.height, 0.0f, 1.0f };
    VkRect2D scissor{ {0, 0}, swapchainExtent };
    VkPipelineViewportStateCreateInfo viewportState{ VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO, nullptr, 0, 1, &viewport, 1, &scissor };
    VkPipelineRasterizationStateCreateInfo rasterizer{ VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO, nullptr, 0, VK_FALSE, VK_FALSE, VK_POLYGON_MODE_FILL, VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE, VK_FALSE, 0.0f, 0.0f, 0.0f, 1.0f };
    VkPipelineMultisampleStateCreateInfo multisampling{ VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO, nullptr, 0, VK_SAMPLE_COUNT_1_BIT, VK_FALSE, 1.0f, nullptr, VK_FALSE, VK_FALSE };
    VkPipelineColorBlendAttachmentState colorBlendAttachment{};
    colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    colorBlendAttachment.blendEnable = VK_FALSE;
    VkPipelineColorBlendStateCreateInfo colorBlending{ VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO, nullptr, 0, VK_FALSE, VK_LOGIC_OP_COPY, 1, &colorBlendAttachment, {0.0f, 0.0f, 0.0f, 0.0f} };
    VkPushConstantRange pushConstant{ VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants) };
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO, nullptr, 0, 0, nullptr, 1, &pushConstant };
    if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &m_graphicsPipelineLayout) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create graphics pipeline layout!");
    }
    VkGraphicsPipelineCreateInfo pipelineInfo{ VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO };
    pipelineInfo.stageCount = 2;
    pipelineInfo.pStages = shaderStages;
    pipelineInfo.pVertexInputState = &vertexInput;
    pipelineInfo.pInputAssemblyState = &inputAssembly;
    pipelineInfo.pViewportState = &viewportState;
    pipelineInfo.pRasterizationState = &rasterizer;
    pipelineInfo.pMultisampleState = &multisampling;
    pipelineInfo.pColorBlendState = &colorBlending;
    pipelineInfo.layout = m_graphicsPipelineLayout;
    pipelineInfo.renderPass = renderPass;
    pipelineInfo.subpass = 0;
    if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create graphics pipeline!");
    }
    vkDestroyShaderModule(device, fragModule, nullptr);
    vkDestroyShaderModule(device, vertModule, nullptr);
}

void GraphicsModule::drawSphere(VkCommandBuffer cmd) {
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);
    VkDeviceSize offsets[] = { 0 };
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, offsets);
    vkCmdBindIndexBuffer(cmd, indexBuffer, 0, VK_INDEX_TYPE_UINT32);
    glm::mat4 view = camera.getViewMatrix();
    glm::mat4 proj = camera.getProjectionMatrix();
    PushConstants pc;
    pc.model = glm::mat4(1.0f);
    pc.mvp = proj * view * pc.model;
    // CLEANUP: Using the correct layout handle
    vkCmdPushConstants(cmd, m_graphicsPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(PushConstants), &pc);
    vkCmdDrawIndexed(cmd, indexCount, 1, 0, 0, 0);
}

void GraphicsModule::destroySphereBuffers() {
    if (vertexBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, vertexBuffer, nullptr);
        vkFreeMemory(device, vertexMemory, nullptr);
        vertexBuffer = VK_NULL_HANDLE;
        vertexMemory = VK_NULL_HANDLE;
    }

    if (indexBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, indexBuffer, nullptr);
        vkFreeMemory(device, indexMemory, nullptr);
        indexBuffer = VK_NULL_HANDLE;
        indexMemory = VK_NULL_HANDLE;
    }
}

void GraphicsModule::createRayTracingPipelineLayout() {
    // Один layout binding для acceleration structure
    VkDescriptorSetLayoutBinding asLayoutBinding{};
    asLayoutBinding.binding = 0;
    asLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    asLayoutBinding.descriptorCount = 1;
    asLayoutBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

    // Один для output image
    VkDescriptorSetLayoutBinding imageLayoutBinding{};
    imageLayoutBinding.binding = 1;
    imageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    imageLayoutBinding.descriptorCount = 1;
    imageLayoutBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

    std::array<VkDescriptorSetLayoutBinding, 2> bindings = {
        asLayoutBinding, imageLayoutBinding
    };

    VkDescriptorSetLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &rtDescriptorSetLayout) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ray tracing descriptor set layout");

    // === Create Pipeline Layout ===
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &rtDescriptorSetLayout;

    if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ray tracing pipeline layout");
}


VkShaderModule GraphicsModule::createShaderModuleFromFile(const std::string& path) {
    std::ifstream file(path, std::ios::ate | std::ios::binary);
    if (!file.is_open())
        throw std::runtime_error("Failed to open shader file: " + path);

    size_t fileSize = static_cast<size_t>(file.tellg());
    std::vector<char> buffer(fileSize);
    file.seekg(0);
    file.read(buffer.data(), fileSize);
    file.close();

    VkShaderModuleCreateInfo createInfo{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    createInfo.codeSize = buffer.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(buffer.data());

    VkShaderModule shaderModule;
    if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
        throw std::runtime_error("Failed to create shader module: " + path);

    return shaderModule;
}

void GraphicsModule::createRayTracingPipeline() {
    VkShaderModule rgenModule = createShaderModuleFromFile(SHADER_PATH "raygen.rgen.spv");
    VkShaderModule rmissModule = createShaderModuleFromFile(SHADER_PATH "miss.rmiss.spv");
    VkShaderModule rchitModule = createShaderModuleFromFile(SHADER_PATH "closesthit.rchit.spv");

    std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
    shaderStages.push_back({VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_RAYGEN_BIT_KHR, rgenModule, "main", nullptr});
    shaderStages.push_back({VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_MISS_BIT_KHR, rmissModule, "main", nullptr});
    shaderStages.push_back({VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR, rchitModule, "main", nullptr});

    std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups(3);
    shaderGroups[0] = {VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, 0, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    shaderGroups[1] = {VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, 1, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};
    shaderGroups[2] = {VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR, VK_SHADER_UNUSED_KHR, 2, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR};

    VkRayTracingPipelineCreateInfoKHR pipelineInfo{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
    pipelineInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
    pipelineInfo.pStages = shaderStages.data();
    pipelineInfo.groupCount = static_cast<uint32_t>(shaderGroups.size());
    pipelineInfo.pGroups = shaderGroups.data();
    pipelineInfo.maxPipelineRayRecursionDepth = 1;
    pipelineInfo.layout = m_rayTracingPipelineLayout;

    // CLEANUP: Removed local function pointer loading. We now use the globally loaded one.
    VkResult result = pfnCreateRayTracingPipelinesKHR(device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &rayTracingPipeline);

    if (result != VK_SUCCESS) {
        // You can add a breakpoint here and inspect the 'result' value in your debugger.
        throw std::runtime_error("Failed to create ray tracing pipeline!");
    }

    vkDestroyShaderModule(device, rgenModule, nullptr);
    vkDestroyShaderModule(device, rmissModule, nullptr);
    vkDestroyShaderModule(device, rchitModule, nullptr);
}


void GraphicsModule::createShaderBindingTable()
{
    // === Получаем размеры хендлов и выравнивание из свойств
    const uint32_t handleSize = rayTracingProperties.shaderGroupHandleSize;
    const uint32_t handleAlignment = rayTracingProperties.shaderGroupHandleAlignment;
    const uint32_t groupSizeAligned = (handleSize + handleAlignment - 1) & ~(handleAlignment - 1);

    // === У нас 3 группы: RayGen, Miss, ClosestHit
    const uint32_t groupCount = 3;
    const uint32_t sbtSize = groupCount * groupSizeAligned;

    // === Выделяем память под хендлы всех групп
    if (!pfnGetRayTracingShaderGroupHandlesKHR)
    {
        pfnGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(
            vkGetDeviceProcAddr(device, "vkGetRayTracingShaderGroupHandlesKHR"));
        if (!pfnGetRayTracingShaderGroupHandlesKHR)
            throw std::runtime_error("Failed to load vkGetRayTracingShaderGroupHandlesKHR");
    }

    assert(rayTracingPipeline != VK_NULL_HANDLE);

    std::vector<uint8_t> shaderHandleStorage(sbtSize);
    VkResult result = pfnGetRayTracingShaderGroupHandlesKHR(
        device,
        rayTracingPipeline,
        0,
        groupCount,
        sbtSize,
        shaderHandleStorage.data()
        );

    if (result != VK_SUCCESS)
        throw std::runtime_error("Failed to get ray tracing shader group handles!");

    // === Создаём SBT буфер в GPU памяти
    createBuffer(
        device,
        physicalDevice,
        sbtSize,
        VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        sbtBuffer,
        sbtMemory,
        true
        );

    // === Копируем хендлы в буфер
    void* mappedData;
    vkMapMemory(device, sbtMemory, 0, sbtSize, 0, &mappedData);
    std::memcpy(mappedData, shaderHandleStorage.data(), sbtSize);
    vkUnmapMemory(device, sbtMemory);

    // === Получаем адрес буфера
    VkBufferDeviceAddressInfo addrInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
    addrInfo.buffer = sbtBuffer;
    VkDeviceAddress sbtAddress = vkGetBufferDeviceAddress(device, &addrInfo);

    // === Разделяем на регионы
    raygenRegion.deviceAddress = sbtAddress;
    raygenRegion.stride = groupSizeAligned;
    raygenRegion.size = groupSizeAligned;

    missRegion.deviceAddress = sbtAddress + groupSizeAligned;
    missRegion.stride = groupSizeAligned;
    missRegion.size = groupSizeAligned;

    hitRegion.deviceAddress = sbtAddress + 2 * groupSizeAligned;
    hitRegion.stride = groupSizeAligned;
    hitRegion.size = groupSizeAligned;
}

uint32_t GraphicsModule::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
{
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        if ((typeFilter & (1 << i)) &&
            (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
        {
            return i;
        }
    }

    throw std::runtime_error("Failed to find suitable memory type!");
}

void GraphicsModule::traceRays(VkCommandBuffer commandBuffer) {
    // No need to load function pointer here anymore
    pfnCmdTraceRaysKHR(
        commandBuffer,
        &raygenRegion, &missRegion, &hitRegion, nullptr,
        swapchainExtent.width, swapchainExtent.height, 1
        );
}

void GraphicsModule::createBufferForSBT(VkDeviceSize size, VkBufferUsageFlags usage,
                                        VkMemoryPropertyFlags properties,
                                        VkBuffer& buffer, VkDeviceMemory& bufferMemory)
{
    VkBufferCreateInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create SBT buffer!");
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

    if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate SBT buffer memory!");
    }

    vkBindBufferMemory(device, buffer, bufferMemory, 0);
}

void GraphicsModule::CreateStagingBuffer()
{
    // Создаём staging buffer для загрузки данных
    VkBuffer stagingBuffer;
    VkDeviceMemory stagingBufferMemory;
    createBufferForSBT(sbtSize,
                       VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                       VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                       stagingBuffer, stagingBufferMemory);

    // Копируем хендлы в staging buffer
    void* mappedData = nullptr;
    vkMapMemory(device, stagingBufferMemory, 0, sbtSize, 0, &mappedData);
    std::memcpy(mappedData, shaderHandleStorage.data(), sbtSize);
    vkUnmapMemory(device, stagingBufferMemory);

    // Создаём буфер для SBT в устройстве
    createBufferForSBT(sbtSize,
                       VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                       sbtBuffer, sbtBufferMemory);

    // Копируем staging → device-local SBT buffer
    VkCommandBufferAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    allocInfo.commandPool = commandPool; // твой пул
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    VkBufferCopy copyRegion{ 0, 0, sbtSize };
    vkCmdCopyBuffer(commandBuffer, stagingBuffer, sbtBuffer, 1, &copyRegion);

    vkEndCommandBuffer(commandBuffer);

    // Сабмит и ожидание
    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);

    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingBufferMemory, nullptr);

}

void GraphicsModule::recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex)
{
    // === Начинаем запись команд
    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
        throw std::runtime_error("Failed to begin recording command buffer!");

    // === Переход изображения в GENERAL (для записи трассировки)
    VkImageMemoryBarrier barrierToGeneral{};
    barrierToGeneral.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrierToGeneral.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrierToGeneral.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrierToGeneral.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrierToGeneral.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrierToGeneral.image = swapchainImages[imageIndex];
    barrierToGeneral.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrierToGeneral.subresourceRange.baseMipLevel = 0;
    barrierToGeneral.subresourceRange.levelCount = 1;
    barrierToGeneral.subresourceRange.baseArrayLayer = 0;
    barrierToGeneral.subresourceRange.layerCount = 1;
    barrierToGeneral.srcAccessMask = 0;
    barrierToGeneral.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrierToGeneral
        );

    // === Запуск трассировки лучей
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, rayTracingPipeline);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, pipelineLayout,
                            0, 1, &descriptorSet, 0, nullptr);

    traceRays(commandBuffer); // ← тут твоя функция трассировки

    // === Переход изображения в PRESENT_SRC_KHR
    VkImageMemoryBarrier barrierToPresent{};
    barrierToPresent.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrierToPresent.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    barrierToPresent.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    barrierToPresent.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrierToPresent.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrierToPresent.image = swapchainImages[imageIndex];
    barrierToPresent.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrierToPresent.subresourceRange.baseMipLevel = 0;
    barrierToPresent.subresourceRange.levelCount = 1;
    barrierToPresent.subresourceRange.baseArrayLayer = 0;
    barrierToPresent.subresourceRange.layerCount = 1;
    barrierToPresent.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    barrierToPresent.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrierToPresent
        );

    // === Завершаем командный буфер
    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
        throw std::runtime_error("Failed to record command buffer!");
}

void GraphicsModule::createDescriptorSetLayout()
{
    VkDescriptorSetLayoutBinding imageBinding{};
    imageBinding.binding = 0;
    imageBinding.descriptorCount = 1;
    imageBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    imageBinding.pImmutableSamplers = nullptr;
    imageBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;

    VkDescriptorSetLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &imageBinding;

    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &rtDescriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create descriptor set layout!");
    }
}

void GraphicsModule::createOutputImage() {
    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = swapchainExtent.width;
    imageInfo.extent.height = swapchainExtent.height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = VK_FORMAT_B8G8R8A8_UNORM;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(device, &imageInfo, nullptr, &outputImage) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ray tracing output image!");

    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(device, outputImage, &memRequirements);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (vkAllocateMemory(device, &allocInfo, nullptr, &outputImageMemory) != VK_SUCCESS)
        throw std::runtime_error("Failed to allocate output image memory!");

    vkBindImageMemory(device, outputImage, outputImageMemory, 0);

    // === Image View
    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = outputImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = imageInfo.format;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device, &viewInfo, nullptr, &outputImageView) != VK_SUCCESS)
        throw std::runtime_error("Failed to create output image view!");
}

void GraphicsModule::createRayTracingDescriptorsAndLayout() {
    VkDescriptorSetLayoutBinding asLayoutBinding{};
    asLayoutBinding.binding = 0;
    asLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    asLayoutBinding.descriptorCount = 1;
    asLayoutBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    VkDescriptorSetLayoutBinding imageLayoutBinding{};
    imageLayoutBinding.binding = 1;
    imageLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    imageLayoutBinding.descriptorCount = 1;
    imageLayoutBinding.stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    std::array<VkDescriptorSetLayoutBinding, 2> bindings = { asLayoutBinding, imageLayoutBinding };
    VkDescriptorSetLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &rtDescriptorSetLayout) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ray tracing descriptor set layout");
    VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &rtDescriptorSetLayout;
    if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &m_rayTracingPipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("Failed to create ray tracing pipeline layout");
}

void GraphicsModule::recordCommandBufferRayTrace(VkCommandBuffer commandBuffer, uint32_t imageIndex) {
    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginInfo);

    // === Барьер: swapchainImage → TRANSFER_DST
    VkImageMemoryBarrier swapchainBarrier{};
    swapchainBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    swapchainBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    swapchainBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    swapchainBarrier.image = swapchainImages[imageIndex];
    swapchainBarrier.subresourceRange = {
        VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1
    };
    swapchainBarrier.srcAccessMask = 0;
    swapchainBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    // === Барьер: outputImage → GENERAL (для записи шейдерами)
    VkImageMemoryBarrier outputBarrier{};
    outputBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    outputBarrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    outputBarrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    outputBarrier.image = outputImage;
    outputBarrier.subresourceRange = {
        VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1
    };
    outputBarrier.srcAccessMask = 0;
    outputBarrier.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;

    std::array<VkImageMemoryBarrier, 2> barriers = { swapchainBarrier, outputBarrier };
    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR | VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0, nullptr,
        0, nullptr,
        static_cast<uint32_t>(barriers.size()),
        barriers.data()
        );

    // === Запуск трассировки лучей
    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, rayTracingPipeline);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_rayTracingPipelineLayout,
                            0, 1, &rtDescriptorSet, 0, nullptr);
    traceRays(commandBuffer);

    // === Барьер: outputImage → TRANSFER_SRC
    VkImageMemoryBarrier outputToTransfer{};
    outputToTransfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    outputToTransfer.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    outputToTransfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    outputToTransfer.image = outputImage;
    outputToTransfer.subresourceRange = {
        VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1
    };
    outputToTransfer.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    outputToTransfer.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;

    // === Барьер: swapchainImage → TRANSFER_DST (повторяем на случай)
    swapchainBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    swapchainBarrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    swapchainBarrier.srcAccessMask = 0;
    swapchainBarrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        0,
        0, nullptr,
        0, nullptr,
        1, &outputToTransfer
        );

    // === Копирование output → swapchain
    VkImageCopy copyRegion{};
    copyRegion.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
    copyRegion.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
    copyRegion.extent = { swapchainExtent.width, swapchainExtent.height, 1 };

    vkCmdCopyImage(commandBuffer,
                   outputImage, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                   swapchainImages[imageIndex], VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                   1, &copyRegion);

    // === Барьер: swapchain → PRESENT_SRC
    VkImageMemoryBarrier presentBarrier{};
    presentBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    presentBarrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    presentBarrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    presentBarrier.image = swapchainImages[imageIndex];
    presentBarrier.subresourceRange = {
        VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1
    };
    presentBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    presentBarrier.dstAccessMask = 0;

    vkCmdPipelineBarrier(
        commandBuffer,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
        0,
        0, nullptr,
        0, nullptr,
        1, &presentBarrier
        );

    vkEndCommandBuffer(commandBuffer);
}


void GraphicsModule::createOutputImageRayTrace() {
    // === Создание изображения
    VkImageCreateInfo imageInfo{ VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO };
    imageInfo.imageType = VK_IMAGE_TYPE_2D;
    imageInfo.extent.width = swapchainExtent.width;
    imageInfo.extent.height = swapchainExtent.height;
    imageInfo.extent.depth = 1;
    imageInfo.mipLevels = 1;
    imageInfo.arrayLayers = 1;
    imageInfo.format = swapchainImageFormat;
    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imageInfo.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;

    if (vkCreateImage(device, &imageInfo, nullptr, &outputImage) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create output image!");
    }

    // === Память под изображение
    VkMemoryRequirements memRequirements;
    vkGetImageMemoryRequirements(device, outputImage, &memRequirements);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (vkAllocateMemory(device, &allocInfo, nullptr, &outputImageMemory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate output image memory!");
    }

    vkBindImageMemory(device, outputImage, outputImageMemory, 0);

    // === Image View
    VkImageViewCreateInfo viewInfo{ VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO };
    viewInfo.image = outputImage;
    viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
    viewInfo.format = swapchainImageFormat;
    viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    viewInfo.subresourceRange.baseMipLevel = 0;
    viewInfo.subresourceRange.levelCount = 1;
    viewInfo.subresourceRange.baseArrayLayer = 0;
    viewInfo.subresourceRange.layerCount = 1;

    if (vkCreateImageView(device, &viewInfo, nullptr, &outputImageView) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create output image view!");
    }
}

void GraphicsModule::drawRayTracedFrame() {
    uint32_t imageIndex;
    VkResult result = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX, VK_NULL_HANDLE, VK_NULL_HANDLE, &imageIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        m_framebufferResized = true;
        return;
    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to acquire swapchain image for ray tracing");
    }

    VkCommandBuffer commandBuffer = commandBuffers[imageIndex];
    recordCommandBufferRayTrace(commandBuffer, imageIndex);

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);

    VkPresentInfoKHR presentInfo{ VK_STRUCTURE_TYPE_PRESENT_INFO_KHR };
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = &swapchain;
    presentInfo.pImageIndices = &imageIndex;

    result = vkQueuePresentKHR(graphicsQueue, &presentInfo);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        m_framebufferResized = true;
    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to present ray traced image!");
    }
}

void GraphicsModule::rebuildAccelerationStructures(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount) {
    vkDeviceWaitIdle(device); // Wait until previous rendering is done

    // Build the BLAS and TLAS with the new geometry
    accelManager->build(vertexBuffer, indexBuffer, vertexCount, indexCount);

    // We must recreate the descriptor set as it depends on the TLAS handle
    if (rtDescriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device, rtDescriptorPool, nullptr);
    }
    // Re-create the descriptor set with the new TLAS
    createRayTracingDescriptorSet(accelManager->getTLAS().handle);

    // Re-create the SBT as the pipeline might change in a more complex scenario
    // For now, we assume the pipeline is static, but recreating SBT is safer.
    if (sbtBuffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(device, sbtBuffer, nullptr);
        vkFreeMemory(device, sbtMemory, nullptr);
    }
    createShaderBindingTable();
}

/**
 * @brief Creates the descriptor pool and the descriptor set for the ray tracing pipeline.
 *
 * This set holds the bindings for the Top-Level Acceleration Structure (binding 0)
 * and the output storage image (binding 1), making them accessible to the shaders.
 *
 * @param topLevelAS The handle to the TLAS that the descriptor set should point to.
 */
void GraphicsModule::createRayTracingDescriptorSet(VkAccelerationStructureKHR topLevelAS) {
    // === 1. Create a Descriptor Pool ===
    // The pool must have enough space for the descriptors we plan to allocate.
    std::array<VkDescriptorPoolSize, 2> poolSizes{};
    // Space for 1 Acceleration Structure descriptor
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    poolSizes[0].descriptorCount = 1;
    // Space for 1 Storage Image descriptor
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = 1; // We are only allocating one set from this pool.
    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &rtDescriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create ray tracing descriptor pool!");
    }

    // === 2. Allocate the Descriptor Set ===
    // We allocate a set using our pre-made rtDescriptorSetLayout as the blueprint.
    VkDescriptorSetAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
    allocInfo.descriptorPool = rtDescriptorPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &rtDescriptorSetLayout;
    if (vkAllocateDescriptorSets(device, &allocInfo, &rtDescriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate ray tracing descriptor set!");
    }

    // === 3. Write the resource handles into the Descriptor Set ===
    // We need two "write" operations: one for the AS and one for the image.

    // First, prepare the write info for the Acceleration Structure
    VkWriteDescriptorSetAccelerationStructureKHR asWriteInfo{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR };
    asWriteInfo.accelerationStructureCount = 1;
    asWriteInfo.pAccelerationStructures = &topLevelAS; // The handle to our new TLAS

    VkWriteDescriptorSet asWrite{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    asWrite.dstSet = rtDescriptorSet;
    asWrite.dstBinding = 0; // Write to binding 0
    asWrite.descriptorCount = 1;
    asWrite.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    asWrite.pNext = &asWriteInfo; // Link the specific AS info

    // Second, prepare the write info for the output image
    VkDescriptorImageInfo imageWriteInfo{};
    imageWriteInfo.imageView = outputImageView;
    imageWriteInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL; // The layout the image will be in when accessed

    VkWriteDescriptorSet imageWrite{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
    imageWrite.dstSet = rtDescriptorSet;
    imageWrite.dstBinding = 1; // Write to binding 1
    imageWrite.descriptorCount = 1;
    imageWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    imageWrite.pImageInfo = &imageWriteInfo;

    // Put both write operations into an array and execute them.
    std::array<VkWriteDescriptorSet, 2> descriptorWrites = { asWrite, imageWrite };
    vkUpdateDescriptorSets(device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}
