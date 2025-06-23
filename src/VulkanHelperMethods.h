#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>

void createBuffer(VkDevice device, VkPhysicalDevice physicalDevice,
                  VkDeviceSize size, VkBufferUsageFlags usage,
                  VkMemoryPropertyFlags properties,
                  VkBuffer& buffer, VkDeviceMemory& bufferMemory, bool needsDeviceAddress = false);


VkShaderModule createShaderModule(VkDevice device, const std::vector<uint32_t>& code);

std::vector<uint32_t> readSPIRV(const std::string& path);
