#include "RayTracingModule.h"

#include <stdexcept>
#include <vector>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

//// Convenience macro for Vulkan error checking
//#define VK_CHECK(x, msg)                                            \
//do {                                                            \
//        VkResult err = (x);                                         \
//        if (err != VK_SUCCESS) {                                    \
//            throw std::runtime_error(std::string(msg) + " failed with error: " + std::to_string(err)); \
//    }                                                           \
//} while (0)

namespace rtx {


    static inline uint32_t AlignUp(uint32_t value, uint32_t alignment)
    {
        return (value + alignment - 1) & ~(alignment - 1);
    }

    void MeshData::Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device) {
        vertexBuffer.Destroy(context);
        indexBuffer.Destroy(context);
        blas.Destroy(context, device);
    }

    void Scene::Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device) {
        for (auto& mesh : meshes) {
            if (mesh) mesh->Destroy(context, device);
        }
        meshes.clear();
    }


    // --- AccelerationStructure Method Implementations ---
    void AccelerationStructure::Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device) {
        if (handle) {
            vkDestroyAccelerationStructureKHR(device, handle, nullptr);
            handle = VK_NULL_HANDLE;
        }
        buffer.Destroy(context);
    }

    RayTracingModule::~RayTracingModule() {
        // Cleanup is now handled explicitly by the GraphicsModule to ensure correct order
    }

    // --- RayTracingModule Public Method Implementations ---

    void RayTracingModule::Initialize(const vulkanhelpers::VulkanContext& context, const CreateInfo& createInfo) {
        m_context = context; // Store the context
        m_createInfo = createInfo;
        GetRayTracingProperties();
        CreateCameraBuffer();
        CreateDescriptorSetLayout();
        CreateDescriptorPool();
        CreatePipeline();
        CreateShaderBindingTable();
    }

    void RayTracingModule::Cleanup() {
        if (device() == VK_NULL_HANDLE) return;
        vkDeviceWaitIdle(device());

        m_tlas.Destroy(m_context, device());
        if (m_scene) {
            m_scene->Destroy(m_context, device());
            m_scene.reset();
        }

        m_storageImage.Destroy(m_context);
        m_cameraUBO.Destroy(m_context);
        m_sbt.Destroy(m_context);

        if (m_pipeline) vkDestroyPipeline(device(), m_pipeline, nullptr);
        if (m_pipelineLayout) vkDestroyPipelineLayout(device(), m_pipelineLayout, nullptr);
        if (m_descriptorPool) vkDestroyDescriptorPool(device(), m_descriptorPool, nullptr);
        if (m_descriptorSetLayout) vkDestroyDescriptorSetLayout(device(), m_descriptorSetLayout, nullptr);
    }

    void RayTracingModule::LoadScene(const std::string& objFilePath) {
        if (m_scene) {
            vkDeviceWaitIdle(device());
            m_tlas.Destroy(m_context, device());
            m_scene->Destroy(m_context, device());
        }

        m_scene = std::make_unique<Scene>();
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, objFilePath.c_str())) {
            throw std::runtime_error("Failed to load OBJ file: " + warn + err);
        }

        // --- THIS SECTION IS NOW CORRECT AND ROBUST ---

        auto mesh = std::make_unique<MeshData>();

        std::vector<glm::vec3> vertices;
        for (size_t i = 0; i < attrib.vertices.size() / 3; ++i) {
            vertices.emplace_back(attrib.vertices[3 * i + 0], attrib.vertices[3 * i + 1], attrib.vertices[3 * i + 2]);
        }
        mesh->vertexCount = static_cast<uint32_t>(vertices.size());

        std::vector<uint32_t> indices;
        for (const auto& shape : shapes) {
            for(const auto& index : shape.mesh.indices) {
                indices.push_back(index.vertex_index);
            }
        }
        mesh->indexCount = static_cast<uint32_t>(indices.size());

        const VkBufferUsageFlags bufferUsage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

        const VkBufferUsageFlags commonUsage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

        // --- THIS IS THE NEW, SIMPLIFIED LOGIC ---
        // The Create function now handles the staging buffer transfer automatically.

        VK_CHECK(mesh->vertexBuffer.Create(m_context, vertices.size() * sizeof(glm::vec3),
                                           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | commonUsage,
                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertices.data()), "Vertex buffer creation failed");

        VK_CHECK(mesh->indexBuffer.Create(m_context, indices.size() * sizeof(uint32_t),
                                          VK_BUFFER_USAGE_INDEX_BUFFER_BIT | commonUsage,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indices.data()), "Index buffer creation failed");

        m_scene->meshes.push_back(std::move(mesh));

        BuildAccelerationStructures();
        UpdateDescriptorSets();
    }


    void RayTracingModule::UpdateCamera(const Camera& camera) {
        CameraUBO ubo;
        ubo.viewInverse = glm::inverse(camera.GetTransform());
        ubo.projInverse = glm::inverse(camera.GetProjection());
        m_cameraUBO.UploadData(m_context, &ubo, sizeof(CameraUBO));
    }

    void RayTracingModule::OnResize(VkExtent2D newExtent) {
        if (m_storageImage.GetImage()) {
            vkDeviceWaitIdle(device());
            m_storageImage.Destroy(m_context);
        }
        m_storageImageExtent = newExtent;
        VkExtent3D extent3D = { newExtent.width, newExtent.height, 1 };
        m_storageImage.Create(
            m_context,
            VK_IMAGE_TYPE_2D, VK_FORMAT_B8G8R8A8_UNORM, extent3D,
            VK_IMAGE_TILING_OPTIMAL,
            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT
            );
        VkImageSubresourceRange range = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
        m_storageImage.CreateImageView(m_context, VK_IMAGE_VIEW_TYPE_2D, VK_FORMAT_B8G8R8A8_UNORM, range);

        // CORRECTED: Call the single, unified update function.
        UpdateDescriptorSets();
    }

    void RayTracingModule::LoadFromVerticesAndIndices(const std::vector<Vertex> &vertices, const std::vector<uint32_t> &indices)
    {
        // 1. Clean up any existing scene
        if (m_scene) {
            vkDeviceWaitIdle(device());
            m_tlas.Destroy(m_context, device());
            m_scene->Destroy(m_context, device());
        }
        m_scene = std::make_unique<Scene>();
        auto mesh = std::make_unique<MeshData>();

        // 2. Set vertex and index counts from the provided data
        mesh->vertexCount = static_cast<uint32_t>(vertices.size());
        mesh->indexCount = static_cast<uint32_t>(indices.size());

        // 3. Define buffer usage flags
        const VkBufferUsageFlags commonUsage = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;

        // 4. Create and upload the vertex and index buffers using the provided data
        VK_CHECK(mesh->vertexBuffer.Create(m_context, vertices.size() * sizeof(Vertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | commonUsage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertices.data()), "Procedural vertex buffer creation failed");
        VK_CHECK(mesh->indexBuffer.Create(m_context, indices.size() * sizeof(uint32_t), VK_BUFFER_USAGE_INDEX_BUFFER_BIT | commonUsage, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indices.data()), "Procedural index buffer creation failed");

        // 5. Add the new mesh to the scene
        m_scene->meshes.push_back(std::move(mesh));

        // 6. Build the acceleration structures and update descriptor sets for the new geometry
        BuildAccelerationStructures();
        UpdateDescriptorSets();
    }

    void RayTracingModule::RecordCommands(VkCommandBuffer cmd, VkImageView targetImageView, VkImage targetImage, VkExtent2D extent) {
        // Recreate storage image if needed (initial run or resize)
        if (!m_storageImage.GetImage() || m_storageImageExtent.width != extent.width || m_storageImageExtent.height != extent.height) {
            OnResize(extent);
        }

        if(m_descriptorSet == VK_NULL_HANDLE) {
            UpdateDescriptorSets();
        }

        VkImageSubresourceRange subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };

        // 1. Transition storage image to be writable by the shader
        vulkanhelpers::ImageBarrier(cmd, m_storageImage.GetImage(), subresourceRange,
                                    0, VK_ACCESS_SHADER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL);

        // 2. Bind pipeline and descriptor sets
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

        // 3. Define SBT regions and trace rays
        VkDeviceAddress baseAddr = GetBufferDeviceAddress(m_context, m_sbt).deviceAddress;

        VkStridedDeviceAddressRegionKHR rgenRegion{ baseAddr + 0 * m_sbtStride, m_sbtStride, m_sbtStride };
        VkStridedDeviceAddressRegionKHR missRegion{ baseAddr + 1 * m_sbtStride, m_sbtStride, m_sbtStride };
        VkStridedDeviceAddressRegionKHR hitRegion { baseAddr + 2 * m_sbtStride, m_sbtStride, m_sbtStride };
        VkStridedDeviceAddressRegionKHR callableRegion{ 0, 0, 0 };

        vkCmdTraceRaysKHR(cmd, &rgenRegion, &missRegion, &hitRegion, &callableRegion,
                          extent.width, extent.height, 1);

        // 4. Transition storage image for transfer and target image for receiving
        vulkanhelpers::ImageBarrier(cmd, m_storageImage.GetImage(), subresourceRange,
                                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                                    VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

        vulkanhelpers::ImageBarrier(cmd, targetImage, subresourceRange,
                                    0, VK_ACCESS_TRANSFER_WRITE_BIT,
                                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

        // 5. Copy from storage image to target (swapchain) image
        VkImageCopy copyRegion{};
        copyRegion.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        copyRegion.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        copyRegion.extent = { extent.width, extent.height, 1 };

        vkCmdCopyImage(cmd, m_storageImage.GetImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       targetImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

        // 6. Transition target image for presentation
        vulkanhelpers::ImageBarrier(cmd, targetImage, subresourceRange,
                                    VK_ACCESS_TRANSFER_WRITE_BIT, 0,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR);
    }

    // --- RayTracingModule Private Method Implementations ---

    void RayTracingModule::GetRayTracingProperties() {
        m_rtProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
        VkPhysicalDeviceProperties2 deviceProps2{};
        deviceProps2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
        deviceProps2.pNext = &m_rtProperties;
        vkGetPhysicalDeviceProperties2(m_createInfo.physicalDevice, &deviceProps2);
    }

    void RayTracingModule::CreateCameraBuffer()
    {
        m_cameraUBO.Create(
            m_context,
            sizeof(CameraUBO),

            /* usage — всё, что начинается с VK_BUFFER_USAGE_* */
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT
                | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,   // ← если вам действительно нужен device address
            //  или просто  VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT

                /* memoryProperties — только VK_MEMORY_PROPERTY_* */
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT
                | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
            );
    }

    void RayTracingModule::CreateDescriptorSetLayout() {
        VkDescriptorSetLayoutBinding tlasBinding{ 0, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1, VK_SHADER_STAGE_RAYGEN_BIT_KHR };
        VkDescriptorSetLayoutBinding imageBinding{ 1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_RAYGEN_BIT_KHR };
        VkDescriptorSetLayoutBinding cameraBinding{ 2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_RAYGEN_BIT_KHR };

        std::vector<VkDescriptorSetLayoutBinding> bindings = { tlasBinding, imageBinding, cameraBinding };
        VkDescriptorSetLayoutCreateInfo layoutInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
        layoutInfo.pBindings = bindings.data();

        // CORRECTED: Assign to the single m_descriptorSetLayout member
        VK_CHECK(vkCreateDescriptorSetLayout(device(), &layoutInfo, nullptr, &m_descriptorSetLayout), "Failed to create descriptor set layout");
    }

    void RayTracingModule::CreateDescriptorPool() {
        std::vector<VkDescriptorPoolSize> poolSizes = {
            { VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1 },
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1 },
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1 }
        };
        VkDescriptorPoolCreateInfo poolInfo{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();
        poolInfo.maxSets = 1; // We only need one descriptor set
        VK_CHECK(vkCreateDescriptorPool(device(), &poolInfo, nullptr, &m_descriptorPool), "Failed to create descriptor pool");
    }

    void RayTracingModule::CreatePipeline() {
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &m_descriptorSetLayout;
        VK_CHECK(vkCreatePipelineLayout(device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout), "Failed to create pipeline layout");

        // Shaders
        vulkanhelpers::Shader rgenShader, rmissShader, rchitShader;

        // --- ADDED ERROR CHECKING HERE ---
        std::string rgenPath = std::string(m_createInfo.shaderDir) + "raygen.rgen.spv";
        if (!rgenShader.LoadFromFile(m_context, rgenPath.c_str())) {
            throw std::runtime_error("Failed to load raygen shader: " + rgenPath);
        }

        std::string rmissPath = std::string(m_createInfo.shaderDir) + "miss.rmiss.spv";
        if (!rmissShader.LoadFromFile(m_context, rmissPath.c_str())) {
            throw std::runtime_error("Failed to load miss shader: " + rmissPath);
        }

        std::string rchitPath = std::string(m_createInfo.shaderDir) + "closesthit.rchit.spv";
        if (!rchitShader.LoadFromFile(m_context, rchitPath.c_str())) {
            throw std::runtime_error("Failed to load closest hit shader: " + rchitPath);
        }

        std::vector<VkPipelineShaderStageCreateInfo> stages = {
            rgenShader.GetShaderStage(VK_SHADER_STAGE_RAYGEN_BIT_KHR),
            rmissShader.GetShaderStage(VK_SHADER_STAGE_MISS_BIT_KHR),
            rchitShader.GetShaderStage(VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
        };

        // Shader Groups
        // --- THIS IS THE CORRECTED SHADER GROUP SETUP ---
        std::vector<VkRayTracingShaderGroupCreateInfoKHR> groups(3);

        // Group 0: Ray Generation
        groups[0].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[0].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[0].generalShader = 0; // Index of rgen shader in stages array
        groups[0].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].intersectionShader = VK_SHADER_UNUSED_KHR;

        // Group 1: Miss
        groups[1].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[1].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[1].generalShader = 1; // Index of rmiss shader in stages array
        groups[1].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].intersectionShader = VK_SHADER_UNUSED_KHR;

        // Group 2: Triangle Hit Group
        groups[2].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[2].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        groups[2].generalShader = VK_SHADER_UNUSED_KHR;
        groups[2].closestHitShader = 2; // Index of rchit shader in stages array
        groups[2].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[2].intersectionShader = VK_SHADER_UNUSED_KHR;

        // Ray Tracing Pipeline
        VkRayTracingPipelineCreateInfoKHR pipelineInfo{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
        pipelineInfo.stageCount = static_cast<uint32_t>(stages.size());
        pipelineInfo.pStages = stages.data();
        pipelineInfo.groupCount = static_cast<uint32_t>(groups.size());
        pipelineInfo.pGroups = groups.data();
        pipelineInfo.maxPipelineRayRecursionDepth = 1;
        pipelineInfo.layout = m_pipelineLayout;

        VK_CHECK(vkCreateRayTracingPipelinesKHR(device(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipeline), "Failed to create ray tracing pipeline");

        rgenShader.Destroy(m_context);
        rmissShader.Destroy(m_context);
        rchitShader.Destroy(m_context);
    }

    void RayTracingModule::CreateShaderBindingTable()
    {
        const uint32_t handleSize    = m_rtProperties.shaderGroupHandleSize;
        const uint32_t baseAlignment = m_rtProperties.shaderGroupBaseAlignment;

        // *** FIX 1: Correct SBT Stride Calculation ***
        // Шаг для каждой записи в SBT должен быть выровнен по shaderGroupBaseAlignment.
        m_sbtStride = AlignUp(handleSize, baseAlignment);

        const uint32_t groupCount = 3; // rgen, miss, hit
        const uint32_t sbtSize = groupCount * m_sbtStride;

        // Получаем хендлы из драйвера
        std::vector<uint8_t> rawHandles(groupCount * handleSize);
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(device(), m_pipeline, 0, groupCount, rawHandles.size(), rawHandles.data()), "Failed to get ray tracing shader group handles");

        // Вручную копируем хендлы в буфер с правильным выравниванием (padding)
        std::vector<uint8_t> sbtData(sbtSize, 0u);
        for(uint32_t i = 0; i < groupCount; ++i) {
            memcpy(sbtData.data() + i * m_sbtStride, rawHandles.data() + i * handleSize, handleSize);
        }

        const VkBufferUsageFlags usage = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
        const VkMemoryPropertyFlags memProps = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

        VK_CHECK(m_sbt.Create(m_context, sbtSize, usage, memProps, sbtData.data()), "SBT creation failed");
    }

    void rtx::RayTracingModule::UpdateDescriptorSets() {
        // Don't attempt to update if the core resources aren't ready yet.
        // This function will be called again when they are (e.g., after a scene load or resize).
        if (!m_scene || m_tlas.handle == VK_NULL_HANDLE || !m_storageImage.GetImageView()) {
            return;
        }

        // --- Allocate the single descriptor set if it hasn't been already ---
        if (m_descriptorSet == VK_NULL_HANDLE) {
            VkDescriptorSetAllocateInfo allocInfo{};
            allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
            allocInfo.descriptorPool = m_descriptorPool;
            allocInfo.descriptorSetCount = 1;
            allocInfo.pSetLayouts = &m_descriptorSetLayout;
            VK_CHECK(vkAllocateDescriptorSets(device(), &allocInfo, &m_descriptorSet), "Failed to allocate ray tracing descriptor set");
        }

        // --- Prepare descriptor writes for all bindings ---

        // 1. Top-Level Acceleration Structure (Binding 0)
        VkWriteDescriptorSetAccelerationStructureKHR tlasWriteInfo{};
        tlasWriteInfo.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
        tlasWriteInfo.accelerationStructureCount = 1;
        tlasWriteInfo.pAccelerationStructures = &m_tlas.handle;

        // 2. Storage Image (Binding 1)
        VkDescriptorImageInfo storageImageInfo{};
        storageImageInfo.imageView = m_storageImage.GetImageView();
        storageImageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

        // 3. Camera Uniform Buffer (Binding 2)
        VkDescriptorBufferInfo cameraBufferInfo{};
        cameraBufferInfo.buffer = m_cameraUBO.GetBuffer();
        cameraBufferInfo.offset = 0;
        cameraBufferInfo.range = VK_WHOLE_SIZE;

        // --- Create a list of write operations ---
        std::vector<VkWriteDescriptorSet> writes;

        // Write for TLAS
        VkWriteDescriptorSet tlasWrite{};
        tlasWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        tlasWrite.dstSet = m_descriptorSet;
        tlasWrite.dstBinding = 0;
        tlasWrite.descriptorCount = 1;
        tlasWrite.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        tlasWrite.pNext = &tlasWriteInfo; // Link the acceleration structure info
        writes.push_back(tlasWrite);

        // Write for Storage Image
        VkWriteDescriptorSet imageWrite{};
        imageWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        imageWrite.dstSet = m_descriptorSet;
        imageWrite.dstBinding = 1;
        imageWrite.descriptorCount = 1;
        imageWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        imageWrite.pImageInfo = &storageImageInfo;
        writes.push_back(imageWrite);

        // Write for Camera UBO
        VkWriteDescriptorSet cameraWrite{};
        cameraWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        cameraWrite.dstSet = m_descriptorSet;
        cameraWrite.dstBinding = 2;
        cameraWrite.descriptorCount = 1;
        cameraWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraWrite.pBufferInfo = &cameraBufferInfo;
        writes.push_back(cameraWrite);

        // --- Perform the update in a single, batched call ---
        vkUpdateDescriptorSets(device(), static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }

    void RayTracingModule::executeImmediateCommand(const std::function<void(VkCommandBuffer)>& command) {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = m_createInfo.commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        VK_CHECK(vkAllocateCommandBuffers(device(), &allocInfo, &commandBuffer), "Failed to allocate one-shot command buffer");

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        VK_CHECK(vkBeginCommandBuffer(commandBuffer, &beginInfo), "Failed to begin one-shot command buffer");

        command(commandBuffer);

        VK_CHECK(vkEndCommandBuffer(commandBuffer), "Failed to end one-shot command buffer");

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        VK_CHECK(vkQueueSubmit(m_createInfo.graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE), "Failed to submit one-shot command buffer");
        VK_CHECK(vkQueueWaitIdle(m_createInfo.graphicsQueue), "Queue wait idle failed");

        vkFreeCommandBuffers(device(), m_createInfo.commandPool, 1, &commandBuffer);
    }

    void RayTracingModule::BuildAccelerationStructures() {
        if (!m_scene || m_scene->meshes.empty()) return;

        for (auto& mesh : m_scene->meshes) {
            BuildBLAS(*mesh);
        }
        BuildTLAS();
    }

    void RayTracingModule::BuildBLAS(MeshData& mesh) {
        VkAccelerationStructureGeometryTrianglesDataKHR triangles{};
        triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
        triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;

        assert(mesh.vertexBuffer.GetBuffer() != VK_NULL_HANDLE && "Vertex buffer is null!");

        triangles.vertexData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, mesh.vertexBuffer).deviceAddress;
        triangles.maxVertex = mesh.vertexCount;
        triangles.vertexStride = sizeof(Vertex);
        triangles.indexType = VK_INDEX_TYPE_UINT32;
        triangles.indexData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, mesh.indexBuffer).deviceAddress;
        VkAccelerationStructureGeometryKHR asGeom{};
        asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        asGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        asGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;
        asGeom.geometry.triangles = triangles;

        VkAccelerationStructureBuildRangeInfoKHR rangeInfo;
        rangeInfo.primitiveCount = mesh.indexCount / 3;
        rangeInfo.primitiveOffset = 0;
        rangeInfo.firstVertex = 0;
        rangeInfo.transformOffset = 0;

        VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
        buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        buildInfo.geometryCount = 1;
        buildInfo.pGeometries = &asGeom;

        VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
        sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        vkGetAccelerationStructureBuildSizesKHR(device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &buildInfo, &rangeInfo.primitiveCount, &sizeInfo);

        VK_CHECK(mesh.blas.buffer.Create(m_context, sizeInfo.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT), "Failed to create BLAS buffer");

        VkAccelerationStructureCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        createInfo.buffer = mesh.blas.buffer.GetBuffer();
        createInfo.size = sizeInfo.accelerationStructureSize;
        createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
        VK_CHECK(vkCreateAccelerationStructureKHR(device(), &createInfo, nullptr, &mesh.blas.handle), "Failed to create BLAS");

        vulkanhelpers::Buffer scratchBuffer;
        scratchBuffer.Create(m_context, sizeInfo.buildScratchSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        buildInfo.dstAccelerationStructure = mesh.blas.handle;
        buildInfo.scratchData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, scratchBuffer).deviceAddress;

        const VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

        executeImmediateCommand([&](VkCommandBuffer cmd) {
            vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, &pRangeInfo);
        });

        VkAccelerationStructureDeviceAddressInfoKHR addressInfo{};
        addressInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
        addressInfo.accelerationStructure = mesh.blas.handle;
        mesh.blas.deviceAddress = vkGetAccelerationStructureDeviceAddressKHR(device(), &addressInfo);

        scratchBuffer.Destroy(m_context);
    }

    void RayTracingModule::BuildTLAS() {
        std::vector<VkAccelerationStructureInstanceKHR> instances;
        for (const auto& mesh : m_scene->meshes) {
            VkAccelerationStructureInstanceKHR instance{};
            instance.transform = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f };
            instance.instanceCustomIndex = 0;
            instance.mask = 0xFF;
            instance.instanceShaderBindingTableRecordOffset = 0;
            instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;
            instance.accelerationStructureReference = mesh->blas.deviceAddress;
            instances.push_back(instance);
        }

        vulkanhelpers::Buffer instanceBuffer;
        instanceBuffer.Create(m_context, sizeof(VkAccelerationStructureInstanceKHR) * instances.size(), VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        instanceBuffer.UploadData(m_context, instances.data(), sizeof(VkAccelerationStructureInstanceKHR) * instances.size());

        VkAccelerationStructureGeometryInstancesDataKHR instancesData{};
        instancesData.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
        instancesData.arrayOfPointers = VK_FALSE;
        instancesData.data.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, instanceBuffer).deviceAddress;

        VkAccelerationStructureGeometryKHR asGeom{};
        asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        asGeom.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
        asGeom.geometry.instances = instancesData;

        VkAccelerationStructureBuildGeometryInfoKHR buildInfo{};
        buildInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
        buildInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        buildInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
        buildInfo.geometryCount = 1;
        buildInfo.pGeometries = &asGeom;

        uint32_t primitive_count = static_cast<uint32_t>(instances.size());
        VkAccelerationStructureBuildSizesInfoKHR sizeInfo{};
        sizeInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
        vkGetAccelerationStructureBuildSizesKHR(device(), VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &buildInfo, &primitive_count, &sizeInfo);

        m_tlas.buffer.Create(m_context, sizeInfo.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        VkAccelerationStructureCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
        createInfo.buffer = m_tlas.buffer.GetBuffer();
        createInfo.size = sizeInfo.accelerationStructureSize;
        createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
        vkCreateAccelerationStructureKHR(device(), &createInfo, nullptr, &m_tlas.handle);

        vulkanhelpers::Buffer scratchBuffer;
        scratchBuffer.Create(m_context, sizeInfo.buildScratchSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        buildInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
        buildInfo.dstAccelerationStructure = m_tlas.handle;
        buildInfo.scratchData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, scratchBuffer).deviceAddress;

        VkAccelerationStructureBuildRangeInfoKHR rangeInfo{};
        rangeInfo.primitiveCount = primitive_count;
        const VkAccelerationStructureBuildRangeInfoKHR* pRangeInfo = &rangeInfo;

        executeImmediateCommand([&](VkCommandBuffer cmd) {
            vkCmdBuildAccelerationStructuresKHR(cmd, 1, &buildInfo, &pRangeInfo);
        });

        scratchBuffer.Destroy(m_context);
        instanceBuffer.Destroy(m_context);
    }

} // namespace rtx
