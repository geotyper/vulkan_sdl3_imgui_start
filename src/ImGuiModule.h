#pragma once

#include <iostream>
#include <vulkan/vulkan.h>
#include <SDL3/SDL.h>

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
              VkRenderPass renderPass,
              uint32_t imageCount);

    void renderMenu(VkCommandBuffer commandBuffer);
    void cleanup();

    void uploadFonts(VkCommandBuffer cmd, VkQueue queue);
    //void uploadFonts2(VkCommandBuffer cmd, VkQueue queue);

    // === Sphere selection ===


    SphereType currentType = SphereType::LowPoly;
    SphereType lastType = SphereType::LowPoly;

    int latDiv = 16, lonDiv = 16;
    int icoSubdiv = 1;

    bool geometryChanged = false;  // Set to true if user modifies sphere parameters

    // Getter for geometry change
    bool hasGeometryChanged() const { return geometryChanged; }
    SphereType getCurrentType() const { return currentType; }
    int getLatDiv() const { return latDiv; }
    int getLonDiv() const { return lonDiv; }
    int getSubdiv() const { return icoSubdiv; }
    void resetGeometryChanged() { geometryChanged = false; }

private:
    VkDevice device = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
};
