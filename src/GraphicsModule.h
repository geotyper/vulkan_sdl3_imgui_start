#pragma once

#include "RayTracingModule.h"
#include "framework/camera.h"
#include <SDL3/SDL.h>
#include <vector>
#include <string>

// Forward declare to break circular dependencies
namespace rtx {
class RayTracingModule;
}

class GraphicsModule {
public:
    GraphicsModule();  // Constructor
    ~GraphicsModule(); // DESTRUCTOR DECLARED HERE

    void Initialize(SDL_Window* window, const std::string& appName);
    void Shutdown();
    void RenderFrame(const Camera& cam);
    void SignalResize();

private:
    // Initialization Steps
    void initVulkan(const std::string& appName);
    void createInstance(const std::string& appName);
    void setupDebugMessenger();
    void createSurface(SDL_Window* window);
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createCommandPool();
    void createSyncObjects();
    void initRayTracingModule();

    // Swapchain Management
    void createSwapchain();
    void cleanupSwapchain();
    void recreateSwapchain();

    // Frame Logic
    void recordCommandBuffer(uint32_t imageIndex, const Camera& cam);

    // Helpers
    void findQueueFamilies();

    // --- Member Variables ---

    // Use a unique_ptr to RayTracingModule to complete the PIMPL pattern
    // and fully resolve the incomplete type issue.
    std::unique_ptr<rtx::RayTracingModule> m_rtxModule;

    // Core Vulkan Objects
    VkInstance m_instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
    uint32_t m_graphicsFamilyIndex = 0;

    // Swapchain
    VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
    VkFormat m_swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D m_swapchainExtent{};
    std::vector<VkImage> m_swapchainImages;
    std::vector<VkImageView> m_swapchainImageViews;

    // Command Execution
    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> m_commandBuffers;

    // Synchronization
    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
    uint32_t m_currentFrame = 0;
    static const int MAX_FRAMES_IN_FLIGHT = 2;

    // State
    bool m_framebufferResized = false;
    SDL_Window* m_window = nullptr;
};
