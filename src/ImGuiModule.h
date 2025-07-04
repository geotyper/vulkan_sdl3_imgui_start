#pragma once

#include <iostream>
#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>
#include <SDL3/SDL.h>
#include <vector>

static void check_vk_result(VkResult err)
{
    if (err == 0)
        return;
    fprintf(stderr, "[vulkan] Error: VkResult = %d\n", err);
    if (err < 0)
        abort();
}

enum class SphereType { LowPoly, UVSphere, Icosphere };

class ImGuiModule {
public:
    void init(SDL_Window* window,
              VkInstance instance,
              VkPhysicalDevice physicalDevice,
              VkDevice device,
              VkQueue graphicsQueue,
              uint32_t queueFamilyIndex,
              VkFormat swapchainFormat,
              VkExtent2D swapchainExtent,
              const std::vector<VkImageView>& swapchainImageViews,
              VkRenderPass renderPass);

    void renderMenu(VkCommandBuffer commandBuffer);
    void cleanup();
    void uploadFonts(VkCommandBuffer cmd, VkQueue graphicsQueue);

    VkRenderPass getRenderPass() const { return m_renderPass; }

    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;

private:
    void createDescriptorPool();
    void createRenderPass(VkFormat swapchainFormat, VkExtent2D swapchainExtent, const std::vector<VkImageView>& swapchainImageViews);
    void createFramebuffers(VkExtent2D extent, const std::vector<VkImageView>& imageViews);

    SDL_Window*      m_window = nullptr;
    VkInstance       m_instance = VK_NULL_HANDLE;
    VkDevice         m_device = VK_NULL_HANDLE;
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
    VkQueue          m_graphicsQueue = VK_NULL_HANDLE;
    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkRenderPass     m_renderPass = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> m_framebuffers;
    VkExtent2D m_extent{};
};
