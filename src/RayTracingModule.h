#pragma once

#include "HelpStructures.h"

#include "framework/vulkanhelpers.h"
#include "framework/camera.h"
#include <string>
#include <vector>
#include <memory>
#include <functional>


namespace rtx {

// A RAII wrapper for Vulkan's Acceleration Structure
struct AccelerationStructure {
    vulkanhelpers::Buffer buffer;
    VkAccelerationStructureKHR handle = VK_NULL_HANDLE;
    VkDeviceAddress deviceAddress = 0;

    void Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device);
};

// --- Full definitions are now in the header to solve incomplete type errors ---
struct MeshData {
    vulkanhelpers::Buffer vertexBuffer;
    vulkanhelpers::Buffer indexBuffer;
    uint32_t vertexCount = 0;
    uint32_t indexCount = 0;
    uint32_t vertexStride  = sizeof(Vertex);
    AccelerationStructure blas;

    void Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device);
};

struct Scene {
    std::vector<std::unique_ptr<MeshData>> meshes;
    void Destroy(const vulkanhelpers::VulkanContext& context, VkDevice device);
};


class RayTracingModule {
public:
    struct CreateInfo {
        VkDevice device = VK_NULL_HANDLE;
        VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
        VkCommandPool commandPool = VK_NULL_HANDLE;
        VkQueue graphicsQueue = VK_NULL_HANDLE;
        const char* shaderDir = "_data/shaders/";
    };

    // Default constructor/destructor are now sufficient
    RayTracingModule() = default;
    ~RayTracingModule();

    void Initialize(const vulkanhelpers::VulkanContext& context, const CreateInfo& createInfo);
    void Cleanup();
    void LoadScene(const std::string& objFilePath);
    void UpdateCamera(const Camera& camera);
    void RecordCommands(VkCommandBuffer cmd, VkImageView targetImageView, VkImage targetImage, VkExtent2D extent);
    void OnResize(VkExtent2D newExtent);

    //void LoadFromVerticesAndIndices(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);
    void LoadFromSingleMesh(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, const std::vector<glm::mat4>& transforms);

private:
    void GetRayTracingProperties();
    void CreateDescriptorSetLayout();
    void CreatePipeline();
    void CreateShaderBindingTable();
    void CreateDescriptorPool();
    void CreateCameraBuffer();
    void BuildAccelerationStructures();
    void BuildBLAS(MeshData& mesh);
    void BuildTLAS();
    void UpdateDescriptorSets();
    void executeImmediateCommand(const std::function<void(VkCommandBuffer)>& command);
    VkDevice device() const { return m_createInfo.device; }

    CreateInfo m_createInfo{};
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR m_rtProperties{};


    vulkanhelpers::VulkanContext m_context;

    VkPipelineLayout m_pipelineLayout = VK_NULL_HANDLE;
    VkPipeline m_pipeline = VK_NULL_HANDLE;
    vulkanhelpers::Buffer m_sbt;
    uint32_t m_sbtStride = 0;
    uint32_t m_sbtRaygenOffset  { 0 }; // смещение секции RayGen   (кратно shaderGroupBaseAlignment)
    uint32_t m_sbtMissOffset    { 0 }; // смещение секции Miss     (кратно shaderGroupBaseAlignment)
    uint32_t m_sbtHitOffset     { 0 }; // смещение секции Hit      (кратно shaderGroupBaseAlignment)

    VkDescriptorPool m_descriptorPool = VK_NULL_HANDLE;
    VkDescriptorSetLayout m_descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorSet m_descriptorSet = VK_NULL_HANDLE;

    std::unique_ptr<Scene> m_scene;
    AccelerationStructure m_tlas;

    struct CameraUBO {
        glm::mat4 viewInverse;
        glm::mat4 projInverse;
    };
    vulkanhelpers::Buffer m_cameraUBO;

    vulkanhelpers::Image m_storageImage;
    VkExtent2D m_storageImageExtent{};

    std::vector<glm::mat4> m_instanceTransforms;



};

} // namespace rtx
