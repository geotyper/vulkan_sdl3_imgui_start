#pragma once

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include "ImGuiModule.h"
#include <SDL3/SDL.h>
#include <memory>
#include <vector>
#include <stdexcept>


class Camera;
namespace rtx { class RayTracingModule; }

class GraphicsModule {
public:
    GraphicsModule();
    ~GraphicsModule();

    // Public API
    void Initialize(const std::string appName);
    void Shutdown();
    void RenderFrame(const Camera& cam);
    void SignalResize();
    bool ShouldClose() const;
    void PollEvents();

    // Getters for ImGuiModule
    SDL_Window* getWindow() { return m_window; }
    VkInstance getVulkanInstance() const { return m_instance; }
    VkPhysicalDevice getPhysicalDevice() const { return m_physicalDevice; }
    VkDevice getDevice() const { return m_device; }
    VkQueue getGraphicsQueue() const { return m_graphicsQueue; }
    uint32_t getGraphicsQueueFamilyIndex() const { return m_graphicsQueueFamilyIndex; }
    VkRenderPass getRenderPass() const { return m_renderPass; }
    const std::vector<VkImageView>& getSwapchainImageViews() const { return m_swapchainImageViews; }
    VkCommandBuffer getCommandBuffer(uint32_t index) const {
        if (index >= m_commandBuffers.size()) {
            throw std::out_of_range("Command buffer index out of range.");
        }
        return m_commandBuffers[index];
    }

    void createImGuiDescriptorPool();
    void loadRayTracingProcs();
    void CreateScene();

    float currentTime=0.0f;
private:
    // Initialization Steps
    void initSDL();
    void initVulkan(const std::string& appName);
    void initRayTracingModule();
    void initImGui();

    // Vulkan Core Setup
    void createInstance(const std::string& appName);
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void findQueueFamilies();
    void createLogicalDevice();
    void createCommandPool();
    void createSyncObjects();

    // Swapchain and Dependent Objects
    void createSwapchain();
    void createImageViews();
    void createRenderPass();
    void createImGuiRenderPass();
    void createImGuiFramebuffers();
    void cleanupSwapchain();
    void recreateSwapchain();

    void createFramebuffers();

    void createGraphicsPipeline();

    // Frame Logic
    void recordCommandBuffer(uint32_t imageIndex, const Camera& cam);

    void initImgui();

    // --- Member Variables ---

    // SDL & Application State
    SDL_Window* m_window = nullptr;
    bool m_framebufferResized = false;
    bool m_windowShouldClose = false;

    // Core Vulkan Objects
    VkInstance m_instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkDevice m_device = VK_NULL_HANDLE;
    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
    uint32_t m_graphicsQueueFamilyIndex = 0;

    // Swapchain
    VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
    VkFormat m_swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D m_swapchainExtent{};
    std::vector<VkImage> m_swapchainImages;
    std::vector<VkImageView> m_swapchainImageViews;
      VkFormat swapchainImageFormat;

    // Command Execution
    VkCommandPool m_commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> m_commandBuffers;
    std::vector<VkFramebuffer> m_framebuffers;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;

    VkRenderPass m_renderPass = VK_NULL_HANDLE;
    VkFormat m_swapchainImageFormat;

    // Synchronization
    std::vector<VkSemaphore> m_imageAvailableSemaphores;
    std::vector<VkSemaphore> m_renderFinishedSemaphores;
    std::vector<VkFence> m_inFlightFences;
    uint32_t m_currentFrame = 0;
    static const int MAX_FRAMES_IN_FLIGHT = 2;

    // Modules
    std::unique_ptr<rtx::RayTracingModule> m_rtxModule;
    //ImGuiModule m_imguiModule;

    // ImGui-specific Vulkan Objects
    VkRenderPass              m_imguiRenderPass  = VK_NULL_HANDLE;
    VkDescriptorPool          m_imguiPool        = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> m_imguiFramebuffers;

    ImGuiModule m_imguiModule;

    int instanceId = 0;
};

//// Forward declare to break circular dependencies
//namespace rtx {
//class RayTracingModule;
//}
//
//class GraphicsModule {
//public:
//    GraphicsModule();  // Constructor
//    ~GraphicsModule(); // DESTRUCTOR DECLARED HERE
//
//    void Initialize(const std::string appName);
//    void Shutdown();
//    void RenderFrame(const Camera& cam);
//    void SignalResize();
//
//
//    // Getters for ImGuiModule
//    SDL_Window* getWindow() { return m_window; }
//    VkInstance getVulkanInstance() const { return m_instance; }
//    VkPhysicalDevice getPhysicalDevice() const { return m_physicalDevice; }
//    VkDevice getDevice() const { return m_device; }
//    VkQueue getGraphicsQueue() const { return m_graphicsQueue; }
//    uint32_t getGraphicsQueueFamilyIndex() const { return m_graphicsQueueFamilyIndex; }
//    VkRenderPass getRenderPass() const { return m_renderPass; }
//    const std::vector<VkImageView>& getSwapchainImageViews() const { return m_swapchainImageViews; }
//    VkCommandBuffer getCommandBuffer(uint32_t index) const {
//        if (index >= m_commandBuffers.size()) {
//            throw std::out_of_range("Command buffer index out of range.");
//        }
//        return m_commandBuffers[index];
//    }
//private:
//    // Initialization Steps
//    void initVulkan(const std::string& appName);
//    void initSDL();
//
//
//    void createInstance(const std::string& appName);
//    void setupDebugMessenger();
//    void createSurface();
//    void pickPhysicalDevice();
//    void createLogicalDevice();
//    void createCommandPool();
//    void createSyncObjects();
//    void initRayTracingModule();
//
//    // Swapchain Management
//    void createSwapchain();
//    void cleanupSwapchain();
//    void recreateSwapchain();
//
//    // Frame Logic
//    void recordCommandBuffer(uint32_t imageIndex, const Camera& cam);
//
//    // Helpers
//    void findQueueFamilies();
//
//    void initImgui();
//
//
//    void createImguiRenderPass();
//
//
//
//    // --- Member Variables ---
//
//    // Use a unique_ptr to RayTracingModule to complete the PIMPL pattern
//    // and fully resolve the incomplete type issue.
//    std::unique_ptr<rtx::RayTracingModule> m_rtxModule;
//
//    // Core Vulkan Objects
//    VkInstance m_instance = VK_NULL_HANDLE;
//    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
//    VkSurfaceKHR m_surface = VK_NULL_HANDLE;
//    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
//    VkDevice m_device = VK_NULL_HANDLE;
//    VkQueue m_graphicsQueue = VK_NULL_HANDLE;
//    uint32_t m_graphicsQueueFamilyIndex = 0;
//
//    // Swapchain
//    VkSwapchainKHR m_swapchain = VK_NULL_HANDLE;
//    VkFormat m_swapchainFormat = VK_FORMAT_UNDEFINED;
//    VkExtent2D m_swapchainExtent{};
//    std::vector<VkImage> m_swapchainImages;
//    std::vector<VkImageView> m_swapchainImageViews;
//
//    // Command Execution
//    VkRenderPass m_renderPass = VK_NULL_HANDLE;
//    VkCommandPool m_commandPool = VK_NULL_HANDLE;
//    std::vector<VkCommandBuffer> m_commandBuffers;
//    std::vector<VkFramebuffer> m_swapchainFramebuffers;
//
//    // Synchronization
//    std::vector<VkSemaphore> m_imageAvailableSemaphores;
//    std::vector<VkSemaphore> m_renderFinishedSemaphores;
//    std::vector<VkFence> m_inFlightFences;
//    uint32_t m_currentFrame = 0;
//    static const int MAX_FRAMES_IN_FLIGHT = 2;
//
//    // State
//    bool m_framebufferResized = false;
//    SDL_Window* m_window = nullptr;
//
//    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
//    VkPipeline m_graphicsPipeline = VK_NULL_HANDLE;
//
//    int instanceId = 0;
//
//
//
//    // ImGui
//public:
//    ImGuiModule m_imguiModule;
//    VkRenderPass              m_imguiRenderPass= VK_NULL_HANDLE;
//    bool                      m_imguiFrameBegun = false;
//
//
//    void createImageViews();
//    void createRenderPass();
//    void createFramebuffers();
//    void createGraphicsPipeline();
//};
//
