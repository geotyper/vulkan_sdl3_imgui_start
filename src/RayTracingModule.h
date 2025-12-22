#pragma once

#include "HelpStructures.h"

#include "framework/vulkanhelpers.h"
#include "framework/camera.h"
#include <string>
#include <vector>
#include <memory>
#include <deque>
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

struct LayerSpec { glm::ivec3 axis; int layerCoord; float sign; float radians; };

enum class Move { U, D, L, R, F, B, U_PRIME, D_PRIME, L_PRIME, R_PRIME, F_PRIME, B_PRIME, U2, D2, L2, R2, F2, B2, None };

struct Cubelet {
    int x, y, z;          // grid coords in {-1,0,1}
    uint32_t inst;        // index into m_instances
};


static LayerSpec MoveSpec(Move m) {
    // axis: which axis to rotate around (x,y,z ∈ {1,0,0} etc.), layerCoord: which slice,
    // sign: +1 or -1 for direction, radians: target angle (±π/2 or π).
    switch (m) {
    case Move::R:       return {{1,0,0}, +1, +1,  glm::half_pi<float>()};
    case Move::R_PRIME: return {{1,0,0}, +1, -1,  glm::half_pi<float>()};
    case Move::R2:      return {{1,0,0}, +1, +1,  glm::pi<float>()};

    case Move::L:       return {{1,0,0}, -1, -1,  glm::half_pi<float>()};
    case Move::L_PRIME: return {{1,0,0}, -1, +1,  glm::half_pi<float>()};
    case Move::L2:      return {{1,0,0}, -1, +1,  glm::pi<float>()};

    case Move::U:       return {{0,1,0}, +1, +1,  glm::half_pi<float>()};
    case Move::U_PRIME: return {{0,1,0}, +1, -1,  glm::half_pi<float>()};
    case Move::U2:      return {{0,1,0}, +1, +1,  glm::pi<float>()};

    case Move::D:       return {{0,1,0}, -1, -1,  glm::half_pi<float>()};
    case Move::D_PRIME: return {{0,1,0}, -1, +1,  glm::half_pi<float>()};
    case Move::D2:      return {{0,1,0}, -1, +1,  glm::pi<float>()};

    case Move::F:       return {{0,0,1}, +1, +1,  glm::half_pi<float>()};
    case Move::F_PRIME: return {{0,0,1}, +1, -1,  glm::half_pi<float>()};
    case Move::F2:      return {{0,0,1}, +1, +1,  glm::pi<float>()};

    case Move::B:       return {{0,0,1}, -1, -1,  glm::half_pi<float>()};
    case Move::B_PRIME: return {{0,0,1}, -1, +1,  glm::half_pi<float>()};
    case Move::B2:      return {{0,0,1}, -1, +1,  glm::pi<float>()};

    default:            return {{0,0,0}, 0, 0, 0};
    }
}

static bool InLayer(const Cubelet& c, const LayerSpec& s) {
    if (s.axis.x) return c.x == s.layerCoord;
    if (s.axis.y) return c.y == s.layerCoord;
    if (s.axis.z) return c.z == s.layerCoord;
    return false;
}



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


    void LoadFromSingleMesh(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, const std::vector<glm::mat4>& transforms);

    void LoadFromMultipleMeshes(const std::vector<rtx::MeshLoadData> &meshData);
    void UpdateUniforms(float time, const glm::vec3& lightColor, float lightIntensity, int step, int paletteID, int samplesPerFrame, int maxBounces, float saturation, float absorption, float ior, float lens, float bias, float chromatic, const glm::vec3& backTop, const glm::vec3& backBot, float pointIntensity, const glm::vec3& pointColor, const glm::vec3& lightPos);
    void AnimateInstances(float time, bool orbitAroundWorldZ);
    void UpdateInstances(const std::vector<InstanceData>& newInstances);

    void InitPerInstanceSpin(uint32_t seed = 1337);
    void Build3x3x3(float spacing);

    void AnimateRubik(float dt);
    void QueueScramble();
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

    std::vector<InstanceData> m_instances;


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

    vulkanhelpers::Buffer m_uniformDataUBO;

public:
    vulkanhelpers::Image m_storageImage;
    VkExtent2D m_storageImageExtent{};

    std::vector<glm::mat4> m_instanceTransforms;

    struct SpinParams {
        glm::vec3 axis;   // unit axis
        float     speed;  // radians per second
        float     phase;  // radians (start offset)
    };

    std::vector<SpinParams> m_spin;

    std::vector<glm::mat4> m_baseInstanceTransforms;


    std::vector<Cubelet> m_cubelets;          // size 27


    struct MoveAnim {
        Move move = Move::None;
        float t = 0.0f;          // seconds elapsed in this move
        float duration = 1.25f;  // time for 90° (or 180°) turn
        bool active() const { return move != Move::None; }
    };
    MoveAnim m_anim;
    std::deque<Move> m_queue;    // optional: queue of moves

    void CreateUniformDataBuffer();
};

} // namespace rtx
