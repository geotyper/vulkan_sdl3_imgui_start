#pragma once

#include <SDL3/SDL.h>
#include <vulkan/vulkan.h>
#include <vector>
#include <stdexcept>
#include <string>
#include <functional>
#include <HelpStructures.h>
#include "ArcBallCamera.h"
#include <glm/glm.hpp>


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

    // Sphere geometry buffers
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;
    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;
    uint32_t indexCount = 0;

    bool mousePressed = false;
    int lastMouseX = 0;
    int lastMouseY = 0;

};
