#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>

// Declarations for Vulkan helper methods

void createBuffer(
    VkDevice device,
    VkPhysicalDevice physicalDevice,
    VkDeviceSize size,
    VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties,
    VkBuffer& buffer,
    VkDeviceMemory& bufferMemory,
    bool needsDeviceAddress = false
    );

VkShaderModule createShaderModule(VkDevice device, const std::vector<uint32_t>& code);
std::vector<uint32_t> readSPIRV(const std::string& path);
uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties);

// --- ADD THESE NEW FUNCTION DECLARATIONS ---
VkCommandBuffer beginSingleTimeCommands(VkDevice device, VkCommandPool commandPool);
void endSingleTimeCommands(VkDevice device, VkCommandPool commandPool, VkQueue queue, VkCommandBuffer commandBuffer);

// Helper function to check if a physical device supports all required extensions.
bool checkDeviceExtensionSupport(VkPhysicalDevice device, const std::vector<const char*>& requiredExtensions);
