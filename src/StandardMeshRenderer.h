#pragma once
#include <vulkan/vulkan.h>
#include <vector>
#include "HelpStructures.h"   // Vertex, PushConstants

class StandardMeshRenderer {
public:
    void Initialize(VkDevice dev, VkPhysicalDevice phys, VkRenderPass rp, uint32_t queueFamily);
    void Cleanup(VkDevice dev);

    void SetMesh(const std::vector<Vertex>& verts, const std::vector<uint32_t>& indices);
    void SetLines(const std::vector<glm::vec3>& linePositions, glm::vec3 color);

    void OnResize(VkDevice dev, VkRenderPass rp);

    void Draw(VkCommandBuffer cmd, VkExtent2D extent) const; // Added extent for dynamic viewport/scissor

    VkPipelineLayout pipelineLayout() const { return m_layout; }

private:
    // Vulkan handles
    VkDevice m_dev{VK_NULL_HANDLE};
    VkPhysicalDevice m_phys{VK_NULL_HANDLE};
    VkPhysicalDeviceMemoryProperties m_memProps{};

    // Resources
    VkPipeline       m_pipeTriangles{VK_NULL_HANDLE};
    VkPipeline       m_pipeLines{VK_NULL_HANDLE};
    VkPipelineLayout m_layout{VK_NULL_HANDLE};

    VkBuffer m_vb{VK_NULL_HANDLE}, m_ib{VK_NULL_HANDLE};
    VkDeviceMemory m_vbMem{VK_NULL_HANDLE}, m_ibMem{VK_NULL_HANDLE};
    uint32_t m_indexCount = 0;

    VkBuffer m_lineVB{VK_NULL_HANDLE};
    VkDeviceMemory m_lineMem{VK_NULL_HANDLE};
    uint32_t m_lineVertCount = 0;
};
