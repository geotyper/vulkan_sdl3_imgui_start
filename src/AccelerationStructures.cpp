#include "AccelerationStructures.h"

#include <stdexcept>
#include <cstring>
#include <array>

#include <vulkan/vulkan.h>
#include <vulkan/vulkan_core.h>

#include "VulkanHelperMethods.h" // if you have helpers for buffer commands


PFN_vkDestroyAccelerationStructureKHR pfnDestroyAS = nullptr;
PFN_vkCreateAccelerationStructureKHR pfnCreateAS = nullptr;

PFN_vkGetAccelerationStructureBuildSizesKHR pfnGetASBuildSizes = nullptr;
PFN_vkCmdBuildAccelerationStructuresKHR pfnCmdBuildAS = nullptr;
PFN_vkGetAccelerationStructureDeviceAddressKHR pfnGetASDeviceAddress = nullptr;

void loadRayTracingFunctions(VkDevice device) {
    pfnDestroyAS = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(
        vkGetDeviceProcAddr(device, "vkDestroyAccelerationStructureKHR"));
    pfnCreateAS = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(
        vkGetDeviceProcAddr(device, "vkCreateAccelerationStructureKHR"));

    pfnGetASBuildSizes = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(
        vkGetDeviceProcAddr(device, "vkGetAccelerationStructureBuildSizesKHR"));
    pfnCmdBuildAS = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(
        vkGetDeviceProcAddr(device, "vkCmdBuildAccelerationStructuresKHR"));
    pfnGetASDeviceAddress = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(
        vkGetDeviceProcAddr(device, "vkGetAccelerationStructureDeviceAddressKHR"));

    if (!pfnDestroyAS || !pfnCreateAS || !pfnGetASBuildSizes || !pfnCmdBuildAS || !pfnGetASDeviceAddress) {
        throw std::runtime_error("Failed to load required ray tracing functions.");
    }
}

#define CHECK_VK(x) if ((x) != VK_SUCCESS) throw std::runtime_error("Vulkan error at " #x)

AccelerationStructureManager::AccelerationStructureManager(VkDevice device, VkPhysicalDevice physDevice,
                                                           VkCommandPool commandPool, VkQueue queue)
    : device(device), physDevice(physDevice), commandPool(commandPool), queue(queue)
{}

AccelerationStructureManager::~AccelerationStructureManager() {
    if (blas.handle) pfnDestroyAS(device, blas.handle, nullptr);
    if (blas.buffer) vkDestroyBuffer(device, blas.buffer, nullptr);
    if (blas.memory) vkFreeMemory(device, blas.memory, nullptr);

    if (tlas.handle) pfnDestroyAS(device, tlas.handle, nullptr);
    if (tlas.buffer) vkDestroyBuffer(device, tlas.buffer, nullptr);
    if (tlas.memory) vkFreeMemory(device, tlas.memory, nullptr);
}

void AccelerationStructureManager::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                                VkMemoryPropertyFlags properties, VkBuffer& buffer,
                                                VkDeviceMemory& memory) {
    VkBufferCreateInfo bufInfo{ VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO };
    bufInfo.size = size;
    bufInfo.usage = usage | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT; // убедись, что этот бит всегда установлен!
    bufInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    CHECK_VK(vkCreateBuffer(device, &bufInfo, nullptr, &buffer));

    VkMemoryRequirements memReq;
    vkGetBufferMemoryRequirements(device, buffer, &memReq);

    // 👇 Используем структуру с флагом DEVICE_ADDRESS
    VkMemoryAllocateFlagsInfo allocFlags{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO };
    allocFlags.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;

    VkMemoryAllocateInfo allocInfo{ VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO };
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = findMemoryType(memReq.memoryTypeBits, properties);
    allocInfo.pNext = &allocFlags; // 👈 подключаем расширение

    CHECK_VK(vkAllocateMemory(device, &allocInfo, nullptr, &memory));
    CHECK_VK(vkBindBufferMemory(device, buffer, memory, 0));
}


uint32_t AccelerationStructureManager::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physDevice, &memProps);

    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeFilter & (1 << i)) && (memProps.memoryTypes[i].propertyFlags & properties) == properties)
            return i;
    }

    throw std::runtime_error("Failed to find suitable memory type");
}

// -- Below are stubs. Next, we’ll implement createTriangleBLAS and createTLAS --

VkDeviceAddress getBufferDeviceAddress(VkDevice device, VkBuffer buffer) {
    VkBufferDeviceAddressInfo addrInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
    addrInfo.buffer = buffer;
    return vkGetBufferDeviceAddress(device, &addrInfo);
}

void AccelerationStructureManager::createTLAS() {
    // === 1. Get device address of BLAS ===
    VkAccelerationStructureDeviceAddressInfoKHR addressInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
    addressInfo.accelerationStructure = blas.handle;

    VkDeviceAddress blasAddress = pfnGetASDeviceAddress(device, &addressInfo);

    // === 2. Create TLAS instance ===
    VkTransformMatrixKHR transformMatrix = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    };

    VkAccelerationStructureInstanceKHR instance{};
    instance.transform = transformMatrix;
    instance.instanceCustomIndex = 0;  // Used in shaders
    instance.mask = 0xFF;
    instance.instanceShaderBindingTableRecordOffset = 0;
    instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
    instance.accelerationStructureReference = blasAddress;

    // === 3. Create buffer for instance ===
    VkBuffer instanceBuffer;
    VkDeviceMemory instanceMemory;

    createBuffer(
        sizeof(VkAccelerationStructureInstanceKHR),
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        instanceBuffer,
        instanceMemory
        );

    void* mapped;
    vkMapMemory(device, instanceMemory, 0, sizeof(VkAccelerationStructureInstanceKHR), 0, &mapped);
    memcpy(mapped, &instance, sizeof(VkAccelerationStructureInstanceKHR));
    vkUnmapMemory(device, instanceMemory);

    VkDeviceAddress instanceAddress = getBufferDeviceAddress(device, instanceBuffer);

    // === 4. Describe geometry ===
    VkAccelerationStructureGeometryInstancesDataKHR instancesData{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
    instancesData.arrayOfPointers = VK_FALSE;
    instancesData.data.deviceAddress = instanceAddress;

    VkAccelerationStructureGeometryKHR geometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geometry.geometry.instances = instancesData;
    geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

    // === 5. Build info and size ===
    VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo{};
    buildRangeInfo.primitiveCount = 1;
    const VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &buildRangeInfo;

    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries = &geometry;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    pfnGetASBuildSizes(
        device,
        VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
        &buildInfo,
        &buildRangeInfo.primitiveCount,
        &sizeInfo
        );

    // === 6. Create TLAS buffer ===
    createBuffer(
        sizeInfo.accelerationStructureSize,
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        tlas.buffer,
        tlas.memory
        );

    VkAccelerationStructureCreateInfoKHR asCreateInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    asCreateInfo.buffer = tlas.buffer;
    asCreateInfo.size = sizeInfo.accelerationStructureSize;
    asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;

    CHECK_VK(pfnCreateAS(device, &asCreateInfo, nullptr, &tlas.handle));

    buildInfo.dstAccelerationStructure = tlas.handle;

    // === 7. Scratch buffer ===
    VkBuffer scratchBuffer;
    VkDeviceMemory scratchMemory;

    createBuffer(
        sizeInfo.buildScratchSize,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        scratchBuffer,
        scratchMemory
        );

    VkDeviceAddress scratchAddress = getBufferDeviceAddress(device, scratchBuffer);
    buildInfo.scratchData.deviceAddress = scratchAddress;

    // === 8. Command buffer for TLAS ===
    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = commandPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;

    VkCommandBuffer cmd;
    CHECK_VK(vkAllocateCommandBuffers(device, &cmdAlloc, &cmd));

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    CHECK_VK(vkBeginCommandBuffer(cmd, &beginInfo));

    pfnCmdBuildAS(cmd, 1, &buildInfo, &pBuildRangeInfo);

    CHECK_VK(vkEndCommandBuffer(cmd));

    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;
    CHECK_VK(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));
    CHECK_VK(vkQueueWaitIdle(queue));

    vkFreeCommandBuffers(device, commandPool, 1, &cmd);
    vkDestroyBuffer(device, scratchBuffer, nullptr);
    vkFreeMemory(device, scratchMemory, nullptr);
    vkDestroyBuffer(device, instanceBuffer, nullptr);
    vkFreeMemory(device, instanceMemory, nullptr);
}


void AccelerationStructureManager::createTriangleBLAS() {
    // === Hardcoded triangle (3 vertices) ===
    struct Vertex { float pos[3]; };
    const std::array<Vertex, 3> triangleVertices = {{
        {{ 0.0f, -0.5f, 0.0f }},
        {{ 0.5f,  0.5f, 0.0f }},
        {{-0.5f,  0.5f, 0.0f }}
    }};

    VkDeviceSize vertexBufferSize = sizeof(triangleVertices);

    // === Create vertex buffer with DEVICE_ADDRESS usage ===
    createBuffer(
        vertexBufferSize,
        VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        blas.buffer,
        blas.memory
        );

    // === Upload vertex data ===
    void* mappedData = nullptr;
    vkMapMemory(device, blas.memory, 0, vertexBufferSize, 0, &mappedData);
    memcpy(mappedData, triangleVertices.data(), vertexBufferSize);
    vkUnmapMemory(device, blas.memory);

    // === Get GPU address of vertex buffer ===
    VkDeviceAddress vertexAddress = getBufferDeviceAddress(device, blas.buffer);

    // === Define geometry ===
    VkAccelerationStructureGeometryTrianglesDataKHR trianglesData{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR };
    trianglesData.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
    trianglesData.vertexData.deviceAddress = vertexAddress;
    trianglesData.vertexStride = sizeof(Vertex);
    trianglesData.maxVertex = 2; // index of last vertex
    trianglesData.indexType = VK_INDEX_TYPE_NONE_KHR;

    VkAccelerationStructureGeometryKHR geometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
    geometry.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
    geometry.geometry.triangles = trianglesData;

    // === Build range info ===
    VkAccelerationStructureBuildRangeInfoKHR buildRangeInfo{};
    buildRangeInfo.primitiveCount = 1;  // 1 triangle
    buildRangeInfo.primitiveOffset = 0;
    buildRangeInfo.firstVertex = 0;
    buildRangeInfo.transformOffset = 0;

    const VkAccelerationStructureBuildRangeInfoKHR* pBuildRangeInfo = &buildRangeInfo;

    // === Build sizes query ===
    VkAccelerationStructureBuildGeometryInfoKHR buildInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    buildInfo.geometryCount = 1;
    buildInfo.pGeometries = &geometry;
    buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;

    VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    pfnGetASBuildSizes(
        device,
        VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR,
        &buildInfo,
        &buildRangeInfo.primitiveCount,
        &sizeInfo
        );

    // === Allocate buffer for BLAS ===
    createBuffer(
        sizeInfo.accelerationStructureSize,
        VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        blas.buffer,
        blas.memory
        );

    // === Create acceleration structure object ===
    VkAccelerationStructureCreateInfoKHR asCreateInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    asCreateInfo.buffer = blas.buffer;
    asCreateInfo.size = sizeInfo.accelerationStructureSize;
    asCreateInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;

    CHECK_VK(pfnCreateAS(device, &asCreateInfo, nullptr, &blas.handle));

    buildInfo.dstAccelerationStructure = blas.handle;

    // === Scratch buffer ===
    VkBuffer scratchBuffer;
    VkDeviceMemory scratchMemory;

    createBuffer(
        sizeInfo.buildScratchSize,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        scratchBuffer,
        scratchMemory
        );

    VkDeviceAddress scratchAddress = getBufferDeviceAddress(device, scratchBuffer);
    buildInfo.scratchData.deviceAddress = scratchAddress;

    // === Record build command ===
    VkCommandBufferAllocateInfo cmdAlloc{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
    cmdAlloc.commandPool = commandPool;
    cmdAlloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cmdAlloc.commandBufferCount = 1;

    VkCommandBuffer cmd;
    CHECK_VK(vkAllocateCommandBuffers(device, &cmdAlloc, &cmd));

    VkCommandBufferBeginInfo beginInfo{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    CHECK_VK(vkBeginCommandBuffer(cmd, &beginInfo));

    pfnCmdBuildAS(cmd, 1, &buildInfo, &pBuildRangeInfo);

    CHECK_VK(vkEndCommandBuffer(cmd));

    // === Submit ===
    VkSubmitInfo submitInfo{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &cmd;
    CHECK_VK(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));
    CHECK_VK(vkQueueWaitIdle(queue));

    vkFreeCommandBuffers(device, commandPool, 1, &cmd);
    vkDestroyBuffer(device, scratchBuffer, nullptr);
    vkFreeMemory(device, scratchMemory, nullptr);
}

