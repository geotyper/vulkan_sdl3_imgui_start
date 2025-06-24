#pragma once

#include <vulkan/vulkan.h>
#include <memory>
#include <vector>

// Forward-declare to avoid including VulkanHelperMethods.h if not needed
struct Vertex;

// Represents a single Acceleration Structure (BLAS or TLAS)
struct AccelerationStructure {
    VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceAddress deviceAddress = 0;
};

class AccelerationStructureManager {
public:
    AccelerationStructureManager(VkDevice device, VkPhysicalDevice physDevice, VkCommandPool commandPool, VkQueue queue);
    ~AccelerationStructureManager();

    // Builds the BLAS and TLAS from the given vertex and index buffers
    void build(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount);

    const AccelerationStructure& getTLAS() const { return tlas; }

private:
    void cleanupAS(AccelerationStructure& as);
    void buildBLAS(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount);
    void buildTLAS();

    VkDevice device;
    VkPhysicalDevice physDevice;
    VkCommandPool commandPool;
    VkQueue queue;

    AccelerationStructure blas;
    AccelerationStructure tlas;
};

// Functions to be loaded dynamically
extern PFN_vkCreateAccelerationStructureKHR pfnCreateAS;
extern PFN_vkDestroyAccelerationStructureKHR pfnDestroyAS;
extern PFN_vkGetAccelerationStructureBuildSizesKHR pfnGetASBuildSizes;
extern PFN_vkCmdBuildAccelerationStructuresKHR pfnCmdBuildAS;
extern PFN_vkGetAccelerationStructureDeviceAddressKHR pfnGetASDeviceAddress;
extern PFN_vkCreateRayTracingPipelinesKHR pfnCreateRayTracingPipelinesKHR;
extern PFN_vkGetRayTracingShaderGroupHandlesKHR pfnGetRayTracingShaderGroupHandlesKHR;
extern PFN_vkCmdTraceRaysKHR pfnCmdTraceRaysKHR;

void loadRayTracingFunctions(VkDevice device);
