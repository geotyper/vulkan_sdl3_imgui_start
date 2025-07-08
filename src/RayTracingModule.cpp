#include "RayTracingModule.h"
#include "shared_with_shaders.h"

#include <stdexcept>
#include <vector>
#include <iostream>
#include <glm/gtc/type_ptr.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>


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
        CreateUniformDataBuffer();
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
        m_uniformDataUBO.Destroy(m_context);
        m_sbt.Destroy(m_context);

        if (m_pipeline) vkDestroyPipeline(device(), m_pipeline, nullptr);
        if (m_pipelineLayout) vkDestroyPipelineLayout(device(), m_pipelineLayout, nullptr);
        if (m_descriptorPool) vkDestroyDescriptorPool(device(), m_descriptorPool, nullptr);
        if (m_descriptorSetLayout) vkDestroyDescriptorSetLayout(device(), m_descriptorSetLayout, nullptr);
    }


    void RayTracingModule::CreateUniformDataBuffer() // <<< NEW
    {
        m_uniformDataUBO.Create(
            m_context,
            sizeof(UniformData),
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT
            );
    }

    void RayTracingModule::UpdateUniforms(float time, const glm::vec3& lightColor, float lightIntensity) {
        UniformData ubo;
        ubo.uTime = time;
        ubo.lightColor = lightColor;
        ubo.lightIntensity = lightIntensity;

        // The rest of the function remains the same
        m_uniformDataUBO.UploadData(m_context, &ubo, sizeof(UniformData));
    }

    void RayTracingModule::LoadScene(const std::string& objFilePath) {
        if (m_scene) {
            vkDeviceWaitIdle(device());
            m_tlas.Destroy(m_context, device());
            m_scene->Destroy(m_context, device());
        }

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, objFilePath.c_str())) {
            throw std::runtime_error("Failed to load OBJ file: " + warn + err);
        }

        std::vector<Vertex> vertices;
        std::vector<uint32_t> indices;
        std::map<tinyobj::index_t, uint32_t, bool(*)(const tinyobj::index_t&, const tinyobj::index_t&)> uniqueVertices([](const tinyobj::index_t& a, const tinyobj::index_t& b) {
            if (a.vertex_index < b.vertex_index) return true;
            if (a.vertex_index > b.vertex_index) return false;
            if (a.normal_index < b.normal_index) return true;
            if (a.normal_index > b.normal_index) return false;
            return a.texcoord_index < b.texcoord_index;
        });

        for (const auto& shape : shapes) {
            for (const auto& index : shape.mesh.indices) {
                if (uniqueVertices.count(index) == 0) {
                    uniqueVertices[index] = static_cast<uint32_t>(vertices.size());

                    Vertex vertex{};
                    vertex.position = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2],
                        1.0f
                    };

                    if (!attrib.normals.empty() && index.normal_index >= 0) {
                        vertex.normal = {
                            attrib.normals[3 * index.normal_index + 0],
                            attrib.normals[3 * index.normal_index + 1],
                            attrib.normals[3 * index.normal_index + 2],
                            0.0f
                        };
                    }

                   // if (!attrib.texcoords.empty() && index.texcoord_index >= 0) {
                   //     vertex.texCoord = {
                   //         attrib.texcoords[2 * index.texcoord_index + 0],
                   //         1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
                   //     };
                   // }

                    vertex.color = {1.0f, 1.0f, 1.0f, 1.0f};
                    vertices.push_back(vertex);
                }
                indices.push_back(uniqueVertices[index]);
            }
        }

       // LoadFromVerticesAndIndices(vertices, indices);
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

    void RayTracingModule::LoadFromSingleMesh(
        const std::vector<Vertex>& vertices,
        const std::vector<uint32_t>& indices,
        const std::vector<glm::mat4>& transforms)
    {
        if (m_scene) {
            vkDeviceWaitIdle(device());
            m_tlas.Destroy(m_context, device());
            m_scene->Destroy(m_context, device());
        }

        m_scene = std::make_unique<Scene>();
        auto mesh = std::make_unique<MeshData>();

        mesh->vertexCount  = static_cast<uint32_t>(vertices.size());
        mesh->indexCount   = static_cast<uint32_t>(indices.size());
        mesh->vertexStride = sizeof(Vertex);

        const VkBufferUsageFlags commonUsage =
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
            VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

        VK_CHECK(mesh->vertexBuffer.Create(
                     m_context,
                     vertices.size() * mesh->vertexStride,  // 64-байтный шаг
                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | commonUsage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                     vertices.data()),
                 "Procedural vertex buffer creation failed");

        VK_CHECK(mesh->indexBuffer.Create(
                     m_context,
                     indices.size() * sizeof(uint32_t),
                     VK_BUFFER_USAGE_INDEX_BUFFER_BIT | commonUsage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                     indices.data()),
                 "Procedural index buffer creation failed");

        m_scene->meshes.push_back(std::move(mesh));
        m_instanceTransforms = transforms;

        BuildAccelerationStructures();
        UpdateDescriptorSets();
    }

    void RayTracingModule::LoadFromMultipleMeshes(const std::vector<rtx::MeshLoadData>& meshData)
    {
        if (m_scene) {
            vkDeviceWaitIdle(device());
            m_tlas.Destroy(m_context, device());
            m_scene->Destroy(m_context, device()); // Destroys all old mesh data
        }

        m_scene = std::make_unique<Scene>();
        m_instances.clear(); // Clear old instance data

        uint32_t meshId = 0;
        for (const auto& data : meshData) {
            auto mesh = std::make_unique<MeshData>();
            mesh->vertexCount  = static_cast<uint32_t>(data.vertices.size());
            mesh->indexCount   = static_cast<uint32_t>(data.indices.size());
            mesh->vertexStride = sizeof(Vertex);

            const VkBufferUsageFlags commonUsage =
                VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR |
                VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT |
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

            // Create vertex and index buffers for the current mesh
            VK_CHECK(mesh->vertexBuffer.Create(
                         m_context, data.vertices.size() * mesh->vertexStride,
                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | commonUsage,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, data.vertices.data()), "Vertex buffer creation failed");

            VK_CHECK(mesh->indexBuffer.Create(
                         m_context, data.indices.size() * sizeof(uint32_t),
                         VK_BUFFER_USAGE_INDEX_BUFFER_BIT | commonUsage,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, data.indices.data()), "Index buffer creation failed");

            // Store instances and associate them with the current mesh ID
            for (const auto& instanceData : data.instances) {
                m_instances.push_back({instanceData.transform, meshId});
            }

            m_scene->meshes.push_back(std::move(mesh));
            meshId++;
        }

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
        // 1. Барьер перед трассировкой: убедиться, что шейдер может писать в storage-image.
        // Источник: нет предыдущей операции. Цель: запись в шейдере трассировки.
        vulkanhelpers::ImageBarrier(cmd, m_storageImage.GetImage(),
                                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                                    subresourceRange,
                                    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR,
                                    0, VK_ACCESS_SHADER_WRITE_BIT);

        // 2. Bind pipeline and descriptor sets
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, m_pipelineLayout, 0, 1, &m_descriptorSet, 0, nullptr);

        // 3. Define SBT regions and trace rays
        VkDeviceAddress baseAddr = vulkanhelpers::GetBufferDeviceAddress(m_context, m_sbt).deviceAddress;

        // Указывает на Группу 0. Размер = 1 шейдер.
        VkStridedDeviceAddressRegionKHR rgenRegion{
            baseAddr + 0 * m_sbtStride,
            m_sbtStride,
            1 * m_sbtStride
        };

        // Указывает на Группу 1 и должен покрывать 3 шейдера (группы 1, 2, 3).
        VkStridedDeviceAddressRegionKHR missRegion{
            baseAddr + 1 * m_sbtStride, // Начало - группа 1
            m_sbtStride,
            3 * m_sbtStride             // Размер - 3 шейдера
        };

        // Указывает на Группу 4 и должен покрывать 1 группу (группу 4).
        VkStridedDeviceAddressRegionKHR hitRegion {
            baseAddr + 4 * m_sbtStride, // Начало - группа 4
            m_sbtStride,
            2 * m_sbtStride             // Размер - 2 shaders
        };
        VkStridedDeviceAddressRegionKHR callableRegion{ 0, 0, 0 };
       // VkStridedDeviceAddressRegionKHR callableRegion{};

        vkCmdTraceRaysKHR(cmd,
                          &rgenRegion,
                          &missRegion,
                          &hitRegion,
                          &callableRegion,
                          extent.width, extent.height,
                          1);

        // 4. Transition storage image for transfer and target image for receiving
        // 3. Барьеры перед копированием:
        //    - Ждем, пока трассировка закончит писать в storage-image, и готовим его к чтению (копированию ИЗ него).
        //    - Готовим swapchain-image к записи (копированию В него).
        vulkanhelpers::ImageBarrier(cmd, m_storageImage.GetImage(),
                                    VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                    subresourceRange,
                                    VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_KHR, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                    VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);

        vulkanhelpers::ImageBarrier(cmd, targetImage,
                                    VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                    subresourceRange,
                                    VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                    0, VK_ACCESS_TRANSFER_WRITE_BIT);

        // 5. Copy from storage image to target (swapchain) image
        VkImageCopy copyRegion{};
        copyRegion.srcSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        copyRegion.dstSubresource = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1 };
        copyRegion.extent = { extent.width, extent.height, 1 };

        vkCmdCopyImage(cmd, m_storageImage.GetImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       targetImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);

        // 6. Transition target image for presentation
        // 5. Финальный барьер: готовим swapchain-image к показу на экране.
        vulkanhelpers::ImageBarrier(cmd, targetImage,
                                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                                    subresourceRange,
                                    VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
                                    VK_ACCESS_TRANSFER_WRITE_BIT, 0);
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

    //------------------------------------------------------------------------------
    //  RayTracingModule::CreateDescriptorSetLayout
    //  – один-единственный набор, доступный из RGEN, MISS и CHIT-стадий
    //------------------------------------------------------------------------------
    void RayTracingModule::CreateDescriptorSetLayout()
    {
        /* 0 ─ TLAS -------------------------------------------------------------- */
        VkDescriptorSetLayoutBinding tlasBinding{};
        tlasBinding.binding         = SWS_SCENE_AS_BINDING;
        tlasBinding.descriptorType  = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        tlasBinding.descriptorCount = 1;
        tlasBinding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR      |
                                      VK_SHADER_STAGE_MISS_BIT_KHR        |  // теневой miss
                                      VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;    // hit-шейдер

        /* 1 ─ Storage-image (куда пишем кадр) ---------------------------------- */
        VkDescriptorSetLayoutBinding imageBinding{};
        imageBinding.binding         = SWS_RESULT_IMAGE_BINDING;
        imageBinding.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        imageBinding.descriptorCount = 1;
        imageBinding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR     |
                                       VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;  // если читаете в hit

        /* 2 ─ Camera UBO (матрицы) – нужен только в raygen --------------------- */
        VkDescriptorSetLayoutBinding cameraBinding{};
        cameraBinding.binding         = SWS_CAMERA_BINDING;
        cameraBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraBinding.descriptorCount = 1;
        cameraBinding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR;


        /* UniformData UBO (time, etc.) - for raygen and hit shaders */
        VkDescriptorSetLayoutBinding uniformDataBinding{};
        uniformDataBinding.binding         = SWS_UNIFORM_DATA_BINDING; // Use new binding point
        uniformDataBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uniformDataBinding.descriptorCount = 1;
        uniformDataBinding.stageFlags      = VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;


        const uint32_t MAX_MESHES = 2;

        /* 3 ─ Vertex-buffer (as SSBO) – только для CHIT ------------------------ */
        VkDescriptorSetLayoutBinding verticesBinding{};
        verticesBinding.binding         = SWS_VERTICES_BINDING;
        verticesBinding.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        verticesBinding.descriptorCount = MAX_MESHES;
        verticesBinding.stageFlags      = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;

        /* 4 ─ Index-buffer (as SSBO) – только для CHIT ------------------------- */

        VkDescriptorSetLayoutBinding indicesBinding{};
        indicesBinding.binding          = SWS_INDICES_BINDING;
        indicesBinding.descriptorType   = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        indicesBinding.descriptorCount  = MAX_MESHES;
        indicesBinding.stageFlags       = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;

        const std::array bindings{
            tlasBinding, imageBinding, cameraBinding,
            verticesBinding, indicesBinding, uniformDataBinding
        };

        VkDescriptorSetLayoutCreateInfo info{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        info.bindingCount = static_cast<uint32_t>(bindings.size());
        info.pBindings    = bindings.data();

        VK_CHECK(vkCreateDescriptorSetLayout(device(), &info, nullptr, &m_descriptorSetLayout),
                 "Failed to create descriptor set layout");
    }


    void RayTracingModule::CreateDescriptorPool() {
        std::vector<VkDescriptorPoolSize> poolSizes = {
            { VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR,    1 },
            { VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,                 1 },
            { VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,                2 },
            { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,                2* SWS_NUM_GEOMETRY_BUFFERS }  //2?
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
        vulkanhelpers::Shader rgenShader, rmissShader, rchitShader, shadowMissShader, shadowAhitShader;

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

        std::string shadowMissPath = std::string(m_createInfo.shaderDir) + "shadow.rmiss.spv";
        if (!shadowMissShader.LoadFromFile(m_context, shadowMissPath.c_str())) {
            throw std::runtime_error("Failed to load shadow miss shader: " + shadowMissPath);
        }

        std::string shadowAhitPath = std::string(m_createInfo.shaderDir) + "shadow.rahit.spv";
        if (!shadowAhitShader.LoadFromFile(m_context, shadowAhitPath.c_str())) {
            throw std::runtime_error("Failed to load shadow any-hit shader: " + shadowAhitPath);
        }

        vulkanhelpers::Shader secondaryMissShader;
        std::string secondaryMissPath = std::string(m_createInfo.shaderDir) + "miss_secondary.rmiss.spv";
        if (!secondaryMissShader.LoadFromFile(m_context, secondaryMissPath.c_str())) {
            throw std::runtime_error("Failed to load secondary miss shader: " + secondaryMissPath);
        }

        vulkanhelpers::Shader achitShader;
        std::string achitShaderPath = std::string(m_createInfo.shaderDir) + "anyhit.rahit.spv";
        if (!achitShader.LoadFromFile(m_context, achitShaderPath.c_str())) {
            throw std::runtime_error("Failed to load secondary miss shader: " + achitShaderPath);
        }


        auto s_rgen       = rgenShader.GetShaderStage(VK_SHADER_STAGE_RAYGEN_BIT_KHR);
        auto s_miss       = rmissShader.GetShaderStage(VK_SHADER_STAGE_MISS_BIT_KHR);
        auto s_shadowMiss = shadowMissShader.GetShaderStage(VK_SHADER_STAGE_MISS_BIT_KHR);
        auto s_chit       = rchitShader.GetShaderStage(VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR);
        auto sa_chit       = shadowAhitShader.GetShaderStage(VK_SHADER_STAGE_ANY_HIT_BIT_KHR);
        auto s_secondaryMiss = secondaryMissShader.GetShaderStage(VK_SHADER_STAGE_MISS_BIT_KHR);
        auto a_chit       = achitShader.GetShaderStage(VK_SHADER_STAGE_ANY_HIT_BIT_KHR);


        std::vector<VkPipelineShaderStageCreateInfo> stages = {
            s_rgen,
            s_miss,
            s_shadowMiss,
            s_secondaryMiss,
            s_chit,
            a_chit,
            sa_chit,
        };


        // Shader Groups
        // --- THIS IS THE CORRECTED SHADER GROUP SETUP ---

        std::vector<VkRayTracingShaderGroupCreateInfoKHR> groups(SWS_NUM_GROUPS);

        // Group 0: Ray Generation
        groups[0].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[0].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[0].generalShader = 0; // s_rgen
        groups[0].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[0].intersectionShader = VK_SHADER_UNUSED_KHR;

        // Group 1: Miss
        groups[1].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[1].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[1].generalShader = 1; // s_primaryMiss
        groups[1].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[1].intersectionShader = VK_SHADER_UNUSED_KHR;

        groups[2].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[2].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[2].generalShader = 2; // s_shadowMiss
        groups[2].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[2].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[2].intersectionShader = VK_SHADER_UNUSED_KHR;

        //// Group 5: Secondary Miss Shader
        groups[3].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[3].type =  VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
        groups[3].generalShader = 3; // s_secondaryMiss
        groups[3].closestHitShader = VK_SHADER_UNUSED_KHR;
        groups[3].anyHitShader = VK_SHADER_UNUSED_KHR;
        groups[3].intersectionShader = VK_SHADER_UNUSED_KHR;

        //// Group 3: Triangle Hit Group
        groups[4].sType = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[4].type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        groups[4].generalShader = VK_SHADER_UNUSED_KHR;
        groups[4].closestHitShader = 4; // s_chit
        groups[4].anyHitShader =    5;
        groups[4].intersectionShader = VK_SHADER_UNUSED_KHR;

        // Group 4: Shadow Hit Group (for shadow rays)
        groups[5].sType              = VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR;
        groups[5].type               = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
        groups[5].closestHitShader   = VK_SHADER_UNUSED_KHR; // нам не нужен closest-hit
        groups[5].anyHitShader       = 6;  // s_sahit  (= shadow.rahit)
        groups[5].intersectionShader = VK_SHADER_UNUSED_KHR;
        groups[5].generalShader      = VK_SHADER_UNUSED_KHR;

        // Ray Tracing Pipeline
        VkRayTracingPipelineCreateInfoKHR pipelineInfo{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
        pipelineInfo.stageCount = static_cast<uint32_t>(stages.size());
        pipelineInfo.pStages = stages.data();
        pipelineInfo.groupCount = static_cast<uint32_t>(groups.size());
        pipelineInfo.pGroups = groups.data();
        pipelineInfo.maxPipelineRayRecursionDepth = 5;
        pipelineInfo.layout = m_pipelineLayout;

        VK_CHECK(vkCreateRayTracingPipelinesKHR(device(), VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &m_pipeline), "Failed to create ray tracing pipeline");

        for (size_t i = 0; i < groups.size(); ++i)
            std::cout << "Group[" << i << "]: type=" << groups[i].type
                      << ", general=" << groups[i].generalShader
                      << ", chit=" << groups[i].closestHitShader
                      << ", ahit=" << groups[i].anyHitShader << std::endl;

        rgenShader.Destroy(m_context);
        rmissShader.Destroy(m_context);
        rchitShader.Destroy(m_context);
        shadowMissShader.Destroy(m_context);
        shadowAhitShader.Destroy(m_context);
        secondaryMissShader.Destroy(m_context);
        achitShader.Destroy(m_context);
    }

    void RayTracingModule::CreateShaderBindingTable()
    {
        const uint32_t handleSize    = m_rtProperties.shaderGroupHandleSize;
        const uint32_t baseAlignment = m_rtProperties.shaderGroupBaseAlignment;

        // *** FIX 1: Correct SBT Stride Calculation ***
        // Шаг для каждой записи в SBT должен быть выровнен по shaderGroupBaseAlignment.
        m_sbtStride = AlignUp(handleSize, baseAlignment);

        const uint32_t groupCount = SWS_NUM_GROUPS; // rgen, miss, hit
        const uint32_t sbtSize = groupCount * m_sbtStride;

        // Получаем хендлы из драйвера
        std::vector<uint8_t> rawHandles(groupCount * handleSize);
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(device(), m_pipeline, 0, groupCount, rawHandles.size(), rawHandles.data()), "Failed to get ray tracing shader group handles");

        // Вручную копируем хендлы в буфер с правильным выравниванием (padding)
        std::vector<uint8_t> sbtData(sbtSize, 0u);
        for(uint32_t i = 0; i < groupCount; ++i) {
            memcpy(sbtData.data() + i * m_sbtStride,
                   rawHandles.data() + i * handleSize,
                   handleSize);
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

        // UniformData UBO
        VkDescriptorBufferInfo uniformDataBufferInfo{};
        uniformDataBufferInfo.buffer = m_uniformDataUBO.GetBuffer();
        uniformDataBufferInfo.offset = 0;
        uniformDataBufferInfo.range  = VK_WHOLE_SIZE;

        // --- Create a list of write operations ---
        std::vector<VkWriteDescriptorSet> writes;

        // Write for TLAS
        VkWriteDescriptorSet tlasWrite{};
        tlasWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        tlasWrite.dstSet = m_descriptorSet;
        tlasWrite.dstBinding = SWS_SCENE_AS_BINDING;
        tlasWrite.descriptorCount = 1;
        tlasWrite.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
        tlasWrite.pNext = &tlasWriteInfo; // Link the acceleration structure info
        writes.push_back(tlasWrite);

        // Write for Storage Image
        VkWriteDescriptorSet imageWrite{};
        imageWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        imageWrite.dstSet = m_descriptorSet;
        imageWrite.dstBinding = SWS_RESULT_IMAGE_BINDING;
        imageWrite.descriptorCount = 1;
        imageWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        imageWrite.pImageInfo = &storageImageInfo;
        writes.push_back(imageWrite);

        // Write for Camera UBO
        VkWriteDescriptorSet cameraWrite{};
        cameraWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        cameraWrite.dstSet = m_descriptorSet;
        cameraWrite.dstBinding = SWS_CAMERA_BINDING;
        cameraWrite.descriptorCount = 1;
        cameraWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        cameraWrite.pBufferInfo = &cameraBufferInfo;
        writes.push_back(cameraWrite);

         // Write for UniformData UBO
        VkWriteDescriptorSet uniformDataWrite{};
        uniformDataWrite.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        uniformDataWrite.dstSet          = m_descriptorSet;
        uniformDataWrite.dstBinding      = SWS_UNIFORM_DATA_BINDING;
        uniformDataWrite.descriptorCount = 1;
        uniformDataWrite.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        uniformDataWrite.pBufferInfo     = &uniformDataBufferInfo;
        writes.push_back(uniformDataWrite);

        // --- Prepare descriptor info for multiple mesh buffers ---
        std::vector<VkDescriptorBufferInfo> vertexBufferInfos;
        std::vector<VkDescriptorBufferInfo> indexBufferInfos;

        // Create a descriptor info for each mesh's vertex and index buffer
        for (const auto& mesh : m_scene->meshes) {
            vertexBufferInfos.push_back({mesh->vertexBuffer.GetBuffer(), 0, VK_WHOLE_SIZE});
            indexBufferInfos.push_back({mesh->indexBuffer.GetBuffer(), 0, VK_WHOLE_SIZE});
        }

        // Only add the writes if there are buffers to bind
        if (!vertexBufferInfos.empty()) {
            // Write for Vertex Buffers (Binding 3)
            VkWriteDescriptorSet vbWrite{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            vbWrite.dstSet          = m_descriptorSet;
            vbWrite.dstBinding      = SWS_VERTICES_BINDING;
            vbWrite.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            vbWrite.descriptorCount = static_cast<uint32_t>(vertexBufferInfos.size());
            vbWrite.pBufferInfo     = vertexBufferInfos.data();
            writes.push_back(vbWrite);

            // Write for Index Buffers (Binding 4)
            VkWriteDescriptorSet ibWrite{ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
            ibWrite.dstSet          = m_descriptorSet;
            ibWrite.dstBinding      = SWS_INDICES_BINDING;
            ibWrite.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            ibWrite.descriptorCount = static_cast<uint32_t>(indexBufferInfos.size());
            ibWrite.pBufferInfo     = indexBufferInfos.data();
            writes.push_back(ibWrite);
        }

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
        triangles.vertexFormat = VK_FORMAT_R32G32B32A32_SFLOAT;

        assert(mesh.vertexBuffer.GetBuffer() != VK_NULL_HANDLE && "Vertex buffer is null!");

        triangles.vertexData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, mesh.vertexBuffer).deviceAddress;
        triangles.maxVertex = mesh.vertexCount;
        triangles.vertexStride = mesh.vertexStride;
        triangles.indexType = VK_INDEX_TYPE_UINT32;
        triangles.indexData.deviceAddress = vulkanhelpers::GetBufferDeviceAddress(m_context, mesh.indexBuffer).deviceAddress;


        VkAccelerationStructureGeometryKHR asGeom{};
        asGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
        asGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
        asGeom.flags = 0;//VK_GEOMETRY_OPAQUE_BIT_KHR;
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

        std::vector<VkAccelerationStructureInstanceKHR> vkInstances;
        vkInstances.reserve(m_instances.size());

        for (const auto& instanceData : m_instances) {
            VkAccelerationStructureInstanceKHR instance{};
            const glm::mat4 tm = glm::transpose(instanceData.transform);
            memcpy(&instance.transform, glm::value_ptr(tm), sizeof(VkTransformMatrixKHR));

           // instance.instanceCustomIndex = static_cast<uint32_t>(vkInstances.size());
            instance.instanceCustomIndex = instanceData.meshId;
            instance.mask = 0xFF;
            instance.instanceShaderBindingTableRecordOffset = SWS_DEFAULT_HIT_IDX;
            instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;

            // *** KEY CHANGE HERE ***
            // Reference the BLAS corresponding to the meshId of the instance
            assert(instanceData.meshId < m_scene->meshes.size());
            instance.accelerationStructureReference = m_scene->meshes[instanceData.meshId]->blas.deviceAddress;

            vkInstances.push_back(instance);
        }

        vulkanhelpers::Buffer instanceBuffer;
        instanceBuffer.Create(m_context, sizeof(VkAccelerationStructureInstanceKHR) * vkInstances.size(), VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        instanceBuffer.UploadData(m_context, vkInstances.data(), sizeof(VkAccelerationStructureInstanceKHR) * vkInstances.size());

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

        uint32_t primitive_count = static_cast<uint32_t>(vkInstances.size());
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
