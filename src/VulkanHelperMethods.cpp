#include "VulkanHelperMethods.h"
#include <fstream>
#include <stdexcept>
#include <string>
#include <set>

//uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) {
//    VkPhysicalDeviceMemoryProperties memProperties;
//    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);
//    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
//        if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
//            return i;
//        }
//    }
//    throw std::runtime_error("Failed to find suitable memory type!");
//}

void createBuffer(VkDevice device,
                  VkPhysicalDevice physicalDevice,
                  VkDeviceSize size,
                  VkBufferUsageFlags usage,
                  VkMemoryPropertyFlags properties,
                  VkBuffer& buffer,
                  VkDeviceMemory& bufferMemory,
                  bool needsDeviceAddress) {
    VkBufferCreateInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufferInfo.size = size;
    bufferInfo.usage = usage;
    if (needsDeviceAddress) {
        bufferInfo.usage |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
    }
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create buffer");
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

    uint32_t memoryTypeIndex = findMemoryType(physicalDevice, memRequirements.memoryTypeBits, properties);

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = memoryTypeIndex;

    VkMemoryAllocateFlagsInfo allocFlagsInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO };
    if (needsDeviceAddress) {
        allocFlagsInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT_KHR;
        allocInfo.pNext = &allocFlagsInfo;
    }

    if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate buffer memory");
    }
    if (vkBindBufferMemory(device, buffer, bufferMemory, 0) != VK_SUCCESS) {
        throw std::runtime_error("Failed to bind buffer memory");
    }
}

VkShaderModule createShaderModule(VkDevice device, const std::vector<uint32_t>& code) {
    VkShaderModuleCreateInfo createInfo{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    createInfo.codeSize = code.size() * sizeof(uint32_t);
    createInfo.pCode = code.data();

    VkShaderModule shaderModule;
    if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create shader module.");
    }
    return shaderModule;
}

std::vector<uint32_t> readSPIRV(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) throw std::runtime_error("Failed to open SPIR-V file: " + path);

    size_t fileSize = static_cast<size_t>(file.tellg());
    std::vector<uint32_t> buffer(fileSize / sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char*>(buffer.data()), fileSize);
    return buffer;
}

VkCommandBuffer beginSingleTimeCommands(VkDevice device, VkCommandPool commandPool) {
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = commandPool;
    allocInfo.commandBufferCount = 1;

    VkCommandBuffer commandBuffer;
    if (vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate single-time command buffer!");
    }

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("Failed to begin single-time command buffer!");
    }
    return commandBuffer;
}

void endSingleTimeCommands(VkDevice device, VkCommandPool commandPool, VkQueue queue, VkCommandBuffer commandBuffer) {
    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("Failed to end single-time command buffer!");
    }

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &commandBuffer;

    if (vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit single-time command buffer!");
    }
    if (vkQueueWaitIdle(queue) != VK_SUCCESS) {
        throw std::runtime_error("Failed to wait on queue for single-time command buffer!");
    }

    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
}

/**
 * @brief Finds a suitable memory type on the physical device that matches the requirements.
 *
 * @param physicalDevice The physical device to query for memory properties.
 * @param typeFilter A bitmask where each set bit corresponds to a memory type that is suitable for the buffer or image. This is obtained from vkGet...MemoryRequirements.
 * @param properties A bitmask of memory property flags that the found memory type must have (e.g., DEVICE_LOCAL, HOST_VISIBLE).
 * @return The index of the suitable memory type.
 * @throws std::runtime_error if no suitable memory type can be found.
 */
uint32_t findMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    // Query the available memory properties of the physical device.
    VkPhysicalDeviceMemoryProperties memProperties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

    // Iterate through all available memory types.
    for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
        // Check if the current memory type (i) is suitable for the resource.
        // The 'typeFilter' bitmask has a bit set for each memory type that is valid.
        bool isTypeSuitable = typeFilter & (1 << i);

        // Check if the current memory type has all the required properties.
        // We use a bitwise AND and compare against the original properties to ensure all flags are present.
        bool hasAllProperties = (memProperties.memoryTypes[i].propertyFlags & properties) == properties;

        if (isTypeSuitable && hasAllProperties) {
            // Found a memory type that satisfies both conditions, return its index.
            return i;
        }
    }

    // If the loop completes without finding a suitable type, throw an error.
    throw std::runtime_error("Failed to find a suitable memory type!");
}


// Helper function to check if a physical device supports all required extensions.
bool checkDeviceExtensionSupport(VkPhysicalDevice device, const std::vector<const char*>& requiredExtensions) {
    uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);
    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

    std::set<std::string> required(requiredExtensions.begin(), requiredExtensions.end());
    for (const auto& extension : availableExtensions) {
        required.erase(extension.extensionName);
    }

    return required.empty();
}
