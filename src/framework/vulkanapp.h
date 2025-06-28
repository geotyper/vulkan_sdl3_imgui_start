#pragma once


#include "framework/vulkanhelpers.h"

#include <SDL3/SDL.h>
#include <SDL3/SDL_vulkan.h>

#include "common.h"

// -----------------------------------------------------------------------------
struct AppSettings {
    std::string name;
    uint32_t    resolutionX = 640;
    uint32_t    resolutionY = 480;
    VkFormat    surfaceFormat = VK_FORMAT_B8G8R8A8_UNORM;
    bool        enableValidation          = false;
    bool        enableVSync               = true;
    bool        supportRaytracing         = false;
    bool        supportDescriptorIndexing = false;
};

struct FPSMeter {
    static constexpr size_t kFPSHistorySize = 128;

    float   fpsHistory[kFPSHistorySize]{};
    size_t  historyPointer   = 0;
    float   fpsAccumulator   = 0.0f;
    float   fps              = 0.0f;

    void  Update(float dt);
    float GetFPS()   const;
    float GetFrameTime() const;
};

// -----------------------------------------------------------------------------
class VulkanApp {
public:
    VulkanApp();
    virtual ~VulkanApp();

    void Run();

protected:
    // lifecycle -------------------------------------------------------------
    bool Initialize();
    void Loop();
    void Shutdown();

    // init steps ------------------------------------------------------------
    void InitializeSettings();
    bool InitializeVulkan();
    bool InitializeDevicesAndQueues();
    bool InitializeSurface();
    bool InitializeSwapchain();
    bool InitializeFencesAndCommandPool();
    bool InitializeOffscreenImage();
    bool InitializeCommandBuffers();
    bool InitializeSynchronization();
    void FillCommandBuffers();

    // frame processing ------------------------------------------------------
    void ProcessFrame(float dt);
    void FreeVulkan();

    // subclass hooks --------------------------------------------------------
    virtual void InitSettings();
    virtual void InitApp();
    virtual void FreeResources();
    virtual void FillCommandBuffer(VkCommandBuffer cmd, size_t imageIndex);

    virtual void OnMouseMove(float x, float y);
    virtual void OnMouseButton(int button, int action, int mods /* clicks */);
    virtual void OnKey(int key, int scancode, int action, int mods);
    virtual void Update(size_t imageIndex, float dt);

protected:
    // settings --------------------------------------------------------------
    AppSettings             mSettings;
    SDL_Window*             mWindow = nullptr;

    // core Vulkan -----------------------------------------------------------
    VkInstance              mInstance            = VK_NULL_HANDLE;
    VkPhysicalDevice        mPhysicalDevice      = VK_NULL_HANDLE;
    VkDevice                mDevice              = VK_NULL_HANDLE;

    // presentation ----------------------------------------------------------
    VkSurfaceFormatKHR      mSurfaceFormat{};
    VkSurfaceKHR            mSurface            = VK_NULL_HANDLE;
    VkSwapchainKHR          mSwapchain          = VK_NULL_HANDLE;
    Array<VkImage>          mSwapchainImages;
    Array<VkImageView>      mSwapchainImageViews;

    // per‑frame sync --------------------------------------------------------
    Array<VkFence>          mWaitForFrameFences;
    VkSemaphore             mSemaphoreImageAcquired   = VK_NULL_HANDLE;
    VkSemaphore             mSemaphoreRenderFinished  = VK_NULL_HANDLE;

    // commands --------------------------------------------------------------
    VkCommandPool           mCommandPool        = VK_NULL_HANDLE;
    Array<VkCommandBuffer>  mCommandBuffers;

    // off‑screen render target ---------------------------------------------
    vulkanhelpers::Image    mOffscreenImage;

    // queues ----------------------------------------------------------------
    uint32_t                mGraphicsQueueFamilyIndex = ~0u;
    uint32_t                mComputeQueueFamilyIndex  = ~0u;
    uint32_t                mTransferQueueFamilyIndex = ~0u;
    VkQueue                 mGraphicsQueue = VK_NULL_HANDLE;
    VkQueue                 mComputeQueue  = VK_NULL_HANDLE;
    VkQueue                 mTransferQueue = VK_NULL_HANDLE;

    // RTX -------------------------------------------------------------------
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR mRTProps{};

    // timing ----------------------------------------------------------------
    FPSMeter                mFPSMeter;
};
