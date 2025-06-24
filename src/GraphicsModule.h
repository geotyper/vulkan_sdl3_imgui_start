#pragma once

#include <SDL3/SDL.h>
#include <vulkan/vulkan.h>
#include <vector>
#include <stdexcept>
#include <string>
#include <functional>
#include <HelpStructures.h>
#include "AccelerationStructures.h"
#include "ArcBallCamera.h"
#include <glm/glm.hpp>
#include <memory>

class GraphicsModule {
public:
    void init();
    void cleanup();
    void pollEvents(); // Already exists
    bool shouldClose() const; // Already exists

    // Getters for ImGuiModule
    SDL_Window* getWindow() { return window; }
    VkInstance getVulkanInstance() const { return instance; }
    VkPhysicalDevice getPhysicalDevice() const { return physicalDevice; }
    VkDevice getDevice() const { return device; }
    VkQueue getGraphicsQueue() const { return graphicsQueue; }
    uint32_t getGraphicsQueueFamilyIndex() const { return graphicsQueueFamilyIndex; }
    VkRenderPass getRenderPass() const { return renderPass; }
    const std::vector<VkImageView>& getSwapchainImageViews() const { return swapchainImageViews; }
    VkCommandBuffer getCommandBuffer(uint32_t index) const {
        if (index >= commandBuffers.size()) {
            throw std::out_of_range("Command buffer index out of range.");
        }
        return commandBuffers[index];
    }


    // Frame handling
    void beginFrame();
    void draw(std::function<void(VkCommandBuffer)> renderCallback);
    void handleResizeIfNeeded();

    void createGraphicsPipeline();
    void drawSphere(VkCommandBuffer cmd);

    // === Public accessors for sphere geometry ===
    VkBuffer& getVertexBuffer() { return vertexBuffer; }
    VkDeviceMemory& getVertexMemory() { return vertexMemory; }
    VkBuffer& getIndexBuffer() { return indexBuffer; }
    VkDeviceMemory& getIndexMemory() { return indexMemory; }
    void setIndexCount(uint32_t count) { indexCount = count; }
    uint32_t getIndexCount() const { return indexCount; }

    void destroySphereBuffers();

    ArcBallCamera camera;

    std::unique_ptr<AccelerationStructureManager> accelManager;
    void drawRayTracedFrame();

    void rebuildAccelerationStructures(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount);
private:
    // SDL related
    SDL_Window* window = nullptr;
    bool m_windowShouldClose = false;
    bool m_framebufferResized = false;

    // Vulkan objects
    VkInstance instance = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue graphicsQueue = VK_NULL_HANDLE;
    uint32_t graphicsQueueFamilyIndex = 0;
    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainImageFormat;
    VkExtent2D swapchainExtent;
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainImageViews;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers;
    std::vector<VkFramebuffer> swapchainFramebuffers;


    // Private initialization steps
    void initSDL();
    void createVulkanInstance();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapchain();
    void createImageViews();
    void createRenderPass();
    void createCommandPoolAndBuffers();
    void createFramebuffers();

    bool wasFramebufferResized() const;
    void acknowledgeResize();
    void recreateSwapchain();

    // Shader pipeline members
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline graphicsPipeline = VK_NULL_HANDLE;
    VkPipeline rayTracingPipeline = VK_NULL_HANDLE;

    // Sphere geometry buffers
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;
    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;
    uint32_t indexCount = 0;

    bool mousePressed = false;
    int lastMouseX = 0;
    int lastMouseY = 0;

    void createRayTracingPipelineLayout();
    VkShaderModule createShaderModuleFromFile(const std::string &filename);

    VkDescriptorSetLayout rtDescriptorSetLayout{};

    void createShaderBindingTable();

    VkPhysicalDeviceRayTracingPipelinePropertiesKHR rayTracingProperties{};

    VkBuffer sbtBuffer;
    VkDeviceMemory sbtMemory;

    VkStridedDeviceAddressRegionKHR raygenRegion{};
    VkStridedDeviceAddressRegionKHR missRegion{};
    VkStridedDeviceAddressRegionKHR hitRegion{};

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    void traceRays(VkCommandBuffer commandBuffer);

    PFN_vkCmdTraceRaysKHR pfnCmdTraceRaysKHR = nullptr;
    PFN_vkGetRayTracingShaderGroupHandlesKHR pfnGetRayTracingShaderGroupHandlesKHR = nullptr;


    void createBufferForSBT(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory);
    void CreateStagingBuffer();

    // === SBT related ===
    //VkBuffer sbtBuffer = VK_NULL_HANDLE;
    VkDeviceMemory sbtBufferMemory = VK_NULL_HANDLE;

    uint32_t sbtSize = 0;
    std::vector<uint8_t> shaderHandleStorage;

    void recordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
    VkDescriptorSet descriptorSet = VK_NULL_HANDLE;

    void createDescriptorSetLayout();

    VkImage outputImage{};
    VkDeviceMemory outputImageMemory{};
    VkImageView outputImageView{};
    VkDescriptorSet rtDescriptorSet{};

    void createOutputImage();

    VkDescriptorPool rtDescriptorPool{};

    //void createRayTracingDescriptorSet(VkAccelerationStructureKHR topLevelAS);
    void recordCommandBufferRayTrace(VkCommandBuffer commandBuffer, uint32_t imageIndex);
    void createOutputImageRayTrace();
    void createRayTracingDescriptorsAndLayout();
    void createRayTracingPipeline();

    VkPipelineLayout m_graphicsPipelineLayout = VK_NULL_HANDLE;
    VkPipelineLayout m_rayTracingPipelineLayout = VK_NULL_HANDLE;


    void createRayTracingDescriptorSet(VkAccelerationStructureKHR topLevelAS);
    VkShaderModule createShaderModule(VkDevice device, const uint32_t *code, size_t codeSize);
};
