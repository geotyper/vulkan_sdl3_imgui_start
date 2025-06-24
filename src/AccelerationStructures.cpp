#include "AccelerationStructures.h"
#include "VulkanHelperMethods.h" // Using the generic createBuffer
#include "GeomCreate.h" // For Vertex struct definition
#include <stdexcept>
#include <cstring>
#include <array>

// Define global function pointers
PFN_vkDestroyAccelerationStructureKHR pfnDestroyAS = nullptr;
PFN_vkCreateAccelerationStructureKHR pfnCreateAS = nullptr;
PFN_vkGetAccelerationStructureBuildSizesKHR pfnGetASBuildSizes = nullptr;
PFN_vkCmdBuildAccelerationStructuresKHR pfnCmdBuildAS = nullptr;
PFN_vkGetAccelerationStructureDeviceAddressKHR pfnGetASDeviceAddress = nullptr;
PFN_vkCreateRayTracingPipelinesKHR pfnCreateRayTracingPipelinesKHR = nullptr;
PFN_vkGetRayTracingShaderGroupHandlesKHR pfnGetRayTracingShaderGroupHandlesKHR = nullptr;
PFN_vkCmdTraceRaysKHR pfnCmdTraceRaysKHR = nullptr;

void loadRayTracingFunctions(VkDevice device) {
    pfnDestroyAS = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkDestroyAccelerationStructureKHR"));
    pfnCreateAS = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(device, "vkCreateAccelerationStructureKHR"));
    pfnGetASBuildSizes = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(device, "vkGetAccelerationStructureBuildSizesKHR"));
    pfnCmdBuildAS = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(device, "vkCmdBuildAccelerationStructuresKHR"));
    pfnGetASDeviceAddress = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(device, "vkGetAccelerationStructureDeviceAddressKHR"));
    pfnCreateRayTracingPipelinesKHR = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(device, "vkCreateRayTracingPipelinesKHR"));
    pfnGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(device, "vkGetRayTracingShaderGroupHandlesKHR"));
    pfnCmdTraceRaysKHR = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(device, "vkCmdTraceRaysKHR"));

    if (!pfnDestroyAS || !pfnCreateAS || !pfnGetASBuildSizes || !pfnCmdBuildAS || !pfnGetASDeviceAddress ||
        !pfnCreateRayTracingPipelinesKHR || !pfnGetRayTracingShaderGroupHandlesKHR || !pfnCmdTraceRaysKHR) {
        throw std::runtime_error("Failed to load required ray tracing functions.");
    }
}

static VkDeviceAddress getBufferDeviceAddress(VkDevice device, VkBuffer buffer) {
    VkBufferDeviceAddressInfo addrInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
    addrInfo.buffer = buffer;
    return vkGetBufferDeviceAddress(device, &addrInfo);
}

AccelerationStructureManager::AccelerationStructureManager(VkDevice device, VkPhysicalDevice physDevice, VkCommandPool commandPool, VkQueue queue)
    : device(device), physDevice(physDevice), commandPool(commandPool), queue(queue) {}

AccelerationStructureManager::~AccelerationStructureManager() {
    cleanupAS(blas);
    cleanupAS(tlas);
}

void AccelerationStructureManager::cleanupAS(AccelerationStructure& as) {
    if (as.handle) pfnDestroyAS(device, as.handle, nullptr);
    if (as.buffer) vkDestroyBuffer(device, as.buffer, nullptr);
    if (as.memory) vkFreeMemory(device, as.memory, nullptr);
    as = {};
}

void AccelerationStructureManager::build(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount) {
    // Clean up previous structures before rebuilding
    cleanupAS(blas);
    cleanupAS(tlas);

    buildBLAS(vertexBuffer, indexBuffer, vertexCount, indexCount);
    buildTLAS();
}

void AccelerationStructureManager::buildBLAS(VkBuffer vertexBuffer, VkBuffer indexBuffer, uint32_t vertexCount, uint32_t indexCount) {
    VkDeviceAddress vertexAddress = getBufferDeviceAddress(device, vertexBuffer);
    VkDeviceAddress indexAddress = getBufferDeviceAddress(device, indexBuffer);

    VkAccelerationStructureGeometryTrianglesDataKHR trianglesData{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR };
    trianglesData.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
    trianglesData.vertexData.deviceAddress = vertexAddress;
    trianglesData.vertexStride = sizeof(Vertex);
    trianglesData.maxVertex = vertexCount - 1;
    trianglesData.indexType = VK_INDEX_TYPE_UINT32;
    trianglesData.indexData.deviceAddress = indexAddress;

    VkAccelerationStructureGeometryKHR geometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
    geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    geometry.geometry.triangles = trianglesData;

    VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo{};
    buildRangeInfo.primitiveCount = indexCount / 3;
    buildRangeInfo.primitiveOffset = 0;
    buildRangeInfo.firstVertex = 0;
    buildRangeInfo.transformOffset = 0;
    const VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &buildRangeInfo;

    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries = &geometry;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    pfnGetASBuildSizes(device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &buildInfo, &buildRangeInfo.primitiveCount, &sizeInfo);

    createBuffer(device, physDevice, sizeInfo.accelerationStructureSize,
                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, blas.buffer, blas.memory, true);

    VkAccelerationStructureCreateInfoKHR asCreateInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    asCreateInfo.buffer = blas.buffer;
    asCreateInfo.size = sizeInfo.accelerationStructureSize;
    asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    if (pfnCreateAS(device, &asCreateInfo, nullptr, &blas.handle) != VK_SUCCESS)
        throw std::runtime_error("Failed to create BLAS handle");

    blas.deviceAddress = getBufferDeviceAddress(device, blas.buffer);
    buildInfo.dstAccelerationStructure = blas.handle;

    VkBuffer scratchBuffer;
    VkDeviceMemory scratchMemory;
    createBuffer(device, physDevice, sizeInfo.buildScratchSize,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, scratchBuffer, scratchMemory, true);
    buildInfo.scratchData.deviceAddress = getBufferDeviceAddress(device, scratchBuffer);

    // Record and submit build command
    VkCommandBuffer cmd = beginSingleTimeCommands(device, commandPool);
    pfnCmdBuildAS(cmd, 1, &buildInfo, &pBuildRangeInfo);
    endSingleTimeCommands(device, commandPool, queue, cmd);

    vkDestroyBuffer(device, scratchBuffer, nullptr);
    vkFreeMemory(device, scratchMemory, nullptr);
}

void AccelerationStructureManager::buildTLAS() {
    VkAccelerationStructureDeviceAddressInfoKHR addressInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
    addressInfo.accelerationStructure = blas.handle;
    VkDeviceAddress blasAddress = pfnGetASDeviceAddress(device, &addressInfo);

    VkTransformMatrixKHR transformMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
    VkAccelerationStructureInstanceKHR instance{};
    instance.transform = transformMatrix;
    instance.instanceCustomIndex = 0;
    instance.mask = 0xFF;
    instance.instanceShaderBindingTableRecordOffset = 0;
    instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    instance.accelerationStructureReference = blasAddress;

    VkBuffer instanceBuffer;
    VkDeviceMemory instanceMemory;
    createBuffer(device, physDevice, sizeof(instance),
                 VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 instanceBuffer, instanceMemory, true);

    void* mapped;
    vkMapMemory(device, instanceMemory, 0, sizeof(instance), 0, &mapped);
    memcpy(mapped, &instance, sizeof(instance));
    vkUnmapMemory(device, instanceMemory);

    VkAccelerationStructureGeometryInstancesDataKHR instancesData{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
    instancesData.arrayOfPointers = VK_FALSE;
    instancesData.data.deviceAddress = getBufferDeviceAddress(device, instanceBuffer);

    VkAccelerationStructureGeometryKHR geometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geometry.geometry.instances = instancesData;
    geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

    VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo{ .primitiveCount = 1 };
    const VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &buildRangeInfo;

    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries = &geometry;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    pfnGetASBuildSizes(device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &buildInfo, &buildRangeInfo.primitiveCount, &sizeInfo);

    createBuffer(device, physDevice, sizeInfo.accelerationStructureSize,
                 VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, tlas.buffer, tlas.memory, true);

    VkAccelerationStructureCreateInfoKHR asCreateInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    asCreateInfo.buffer = tlas.buffer;
    asCreateInfo.size = sizeInfo.accelerationStructureSize;
    asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    if (pfnCreateAS(device, &asCreateInfo, nullptr, &tlas.handle) != VK_SUCCESS)
        throw std::runtime_error("Failed to create TLAS handle");

    tlas.deviceAddress = getBufferDeviceAddress(device, tlas.buffer);
    buildInfo.dstAccelerationStructure = tlas.handle;

    VkBuffer scratchBuffer;
    VkDeviceMemory scratchMemory;
    createBuffer(device, physDevice, sizeInfo.buildScratchSize,
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, scratchBuffer, scratchMemory, true);
    buildInfo.scratchData.deviceAddress = getBufferDeviceAddress(device, scratchBuffer);

    VkCommandBuffer cmd = beginSingleTimeCommands(device, commandPool);
    pfnCmdBuildAS(cmd, 1, &buildInfo, &pBuildRangeInfo);
    endSingleTimeCommands(device, commandPool, queue, cmd);

    vkDestroyBuffer(device, scratchBuffer, nullptr);
    vkFreeMemory(device, scratchMemory, nullptr);
    vkDestroyBuffer(device, instanceBuffer, nullptr);
    vkFreeMemory(device, instanceMemory, nullptr);
}
