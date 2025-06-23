#pragma once

#include <vulkan/vulkan.h>
#include <vector>

extern PFN_vkDestroyAccelerationStructureKHR pfnDestroyAS;
extern PFN_vkCreateAccelerationStructureKHR pfnCreateAS;

extern PFN_vkGetAccelerationStructureBuildSizesKHR pfnGetASBuildSizes;
extern PFN_vkCmdBuildAccelerationStructuresKHR pfnCmdBuildAS;
extern PFN_vkGetAccelerationStructureDeviceAddressKHR pfnGetASDeviceAddress;



void loadRayTracingFunctions(VkDevice device);

struct AccelerationStructure {
    VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkBuffer buffer = VK_NULL_HANDLE;
};

class AccelerationStructureManager {
public:
    AccelerationStructureManager(VkDevice device, VkPhysicalDevice physDevice, VkCommandPool commandPool, VkQueue queue);
    ~AccelerationStructureManager();

    void createTriangleBLAS();
    void createTLAS();

    const AccelerationStructure& getBLAS() const { return blas; }
    const AccelerationStructure& getTLAS() const { return tlas; }



private:
    VkDevice device;
    VkPhysicalDevice physDevice;
    VkCommandPool commandPool;
    VkQueue queue;

    AccelerationStructure blas;
    AccelerationStructure tlas;

    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties,
                      VkBuffer& buffer, VkDeviceMemory& memory);
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
};
