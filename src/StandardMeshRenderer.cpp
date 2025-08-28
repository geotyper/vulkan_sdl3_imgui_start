// StandardMeshRenderer.cpp
#include "volk.h"
#include "StandardMeshRenderer.h"
#include "VulkanCheck.h"

#include <cstring>
#include <fstream>

// -------------------- helpers --------------------
static std::vector<char> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f)
        throw std::runtime_error("StandardMeshRenderer: can't open " + path);
    const size_t sz = size_t(f.tellg());
    std::vector<char> data(sz);
    f.seekg(0);
    f.read(data.data(), sz);
    return data;
}

static uint32_t findMemoryType(uint32_t typeBits,
                               VkMemoryPropertyFlags props,
                               const VkPhysicalDeviceMemoryProperties& memProps)
{
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    throw std::runtime_error("StandardMeshRenderer: no suitable memory type");
}

// -------------------- init -----------------------
void StandardMeshRenderer::Initialize(VkDevice dev,
                                      VkPhysicalDevice phys,
                                      VkRenderPass rp,
                                      uint32_t /*queueFamily*/)
{

    if (dev == VK_NULL_HANDLE || phys == VK_NULL_HANDLE || rp == VK_NULL_HANDLE) {
        std::fprintf(stderr,
                     "SMR::Initialize error: dev=%p phys=%p rp=%p (one of them is NULL). "
                     "Call initRasterRenderers() AFTER device+renderpass are created.\n",
                     (void*)dev, (void*)phys, (void*)rp);
        throw std::runtime_error("StandardMeshRenderer::Initialize: null handle(s)");
    }

    m_dev  = dev;
    m_phys = phys;
    vkGetPhysicalDeviceMemoryProperties(m_phys, &m_memProps);

    // pipeline layout (push-constants only)
    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pcr.offset = 0;
    pcr.size   = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pli{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pli.pushConstantRangeCount = 1;
    pli.pPushConstantRanges    = &pcr;
    VK_CHECK(vkCreatePipelineLayout(m_dev, &pli, nullptr, &m_layout),
             "StandardMeshRenderer: vkCreatePipelineLayout failed");

    // shaders
    const char* shaderDir = "shaders/";
    auto vert = readFile(std::string(shaderDir) + std::string("debug_draw.vert.spv"));
    auto frag = readFile(std::string(shaderDir) + std::string("debug_draw.frag.spv"));

    VkShaderModuleCreateInfo smi{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    smi.codeSize = vert.size();
    smi.pCode    = reinterpret_cast<const uint32_t*>(vert.data());
    VkShaderModule vs; VK_CHECK(vkCreateShaderModule(m_dev, &smi, nullptr, &vs),
             "StandardMeshRenderer: vert module");

    smi.codeSize = frag.size();
    smi.pCode    = reinterpret_cast<const uint32_t*>(frag.data());
    VkShaderModule fs; VK_CHECK(vkCreateShaderModule(m_dev, &smi, nullptr, &fs),
             "StandardMeshRenderer: frag module");

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vs;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = fs;
    stages[1].pName  = "main";

    // vertex input (matches Vertex)
    VkVertexInputBindingDescription bind{};
    bind.binding   = 0;
    bind.stride    = sizeof(Vertex);
    bind.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputAttributeDescription attrs[3]{};
    attrs[0].location = 0; attrs[0].binding = 0;
    attrs[0].format   = VK_FORMAT_R32G32B32A32_SFLOAT;
    attrs[0].offset   = offsetof(Vertex, position);

    attrs[1].location = 1; attrs[1].binding = 0;
    attrs[1].format   = VK_FORMAT_R32G32B32A32_SFLOAT;
    attrs[1].offset   = offsetof(Vertex, normal);

    attrs[2].location = 2; attrs[2].binding = 0;
    attrs[2].format   = VK_FORMAT_R32G32B32A32_SFLOAT;
    attrs[2].offset   = offsetof(Vertex, color);

    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vi.vertexBindingDescriptionCount   = 1;
    vi.pVertexBindingDescriptions      = &bind;
    vi.vertexAttributeDescriptionCount = 3;
    vi.pVertexAttributeDescriptions    = attrs;

    VkPipelineInputAssemblyStateCreateInfo ia_tri{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia_tri.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineInputAssemblyStateCreateInfo ia_line{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia_line.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount  = 1;

    VkDynamicState dynStates[] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo dyn{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dyn.dynamicStateCount = 2;
    dyn.pDynamicStates    = dynStates;

    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_BACK_BIT;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState att{};
    att.colorWriteMask = VK_COLOR_COMPONENT_R_BIT|VK_COLOR_COMPONENT_G_BIT|
                         VK_COLOR_COMPONENT_B_BIT|VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments    = &att;

    auto makePipe = [&](const VkPipelineInputAssemblyStateCreateInfo& ia, VkPipeline* out) {
        VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        gp.stageCount          = 2;
        gp.pStages             = stages;
        gp.pVertexInputState   = &vi;
        gp.pInputAssemblyState = &ia;
        gp.pViewportState      = &vp;
        gp.pRasterizationState = &rs;
        gp.pMultisampleState   = &ms;
        gp.pColorBlendState    = &cb;
        gp.pDynamicState       = &dyn;
        gp.layout              = m_layout;
        gp.renderPass          = rp;
        gp.subpass             = 0;
        VK_CHECK(vkCreateGraphicsPipelines(m_dev, VK_NULL_HANDLE, 1, &gp, nullptr, out),
                 "StandardMeshRenderer: vkCreateGraphicsPipelines");
    };

    makePipe(ia_tri,  &m_pipeTriangles);
    makePipe(ia_line, &m_pipeLines);

    vkDestroyShaderModule(m_dev, fs, nullptr);
    vkDestroyShaderModule(m_dev, vs, nullptr);
}

void StandardMeshRenderer::Cleanup(VkDevice dev)
{
    if (!dev) dev = m_dev;

    if (m_ib)      { vkDestroyBuffer(dev, m_ib, nullptr);      m_ib = VK_NULL_HANDLE; }
    if (m_vb)      { vkDestroyBuffer(dev, m_vb, nullptr);      m_vb = VK_NULL_HANDLE; }
    if (m_ibMem)   { vkFreeMemory(dev,  m_ibMem, nullptr);     m_ibMem = VK_NULL_HANDLE; }
    if (m_vbMem)   { vkFreeMemory(dev,  m_vbMem, nullptr);     m_vbMem = VK_NULL_HANDLE; }

    if (m_lineVB)  { vkDestroyBuffer(dev, m_lineVB, nullptr);  m_lineVB = VK_NULL_HANDLE; }
    if (m_lineMem) { vkFreeMemory(dev,  m_lineMem, nullptr);   m_lineMem = VK_NULL_HANDLE; }

    if (m_pipeTriangles) { vkDestroyPipeline(dev, m_pipeTriangles, nullptr); m_pipeTriangles = VK_NULL_HANDLE; }
    if (m_pipeLines)     { vkDestroyPipeline(dev, m_pipeLines,     nullptr); m_pipeLines     = VK_NULL_HANDLE; }
    if (m_layout)        { vkDestroyPipelineLayout(dev, m_layout,  nullptr); m_layout        = VK_NULL_HANDLE; }

    m_indexCount    = 0;
    m_lineVertCount = 0;
    m_dev = VK_NULL_HANDLE;
    m_phys = VK_NULL_HANDLE;
}

// Dynamic viewport/scissor → nothing to rebuild
void StandardMeshRenderer::OnResize(VkDevice /*dev*/, VkRenderPass /*rp*/)
{
}

void StandardMeshRenderer::SetMesh(const std::vector<Vertex>& verts,
                                   const std::vector<uint32_t>& indices)
{
    const VkDeviceSize vbSize = sizeof(Vertex)   * verts.size();
    const VkDeviceSize ibSize = sizeof(uint32_t) * indices.size();

    auto makeBuffer = [&](VkDeviceSize size, VkBufferUsageFlags usage,
                          VkBuffer& buf, VkDeviceMemory& mem) {
        if (buf) { vkDestroyBuffer(m_dev, buf, nullptr); buf = VK_NULL_HANDLE; }
        if (mem) { vkFreeMemory(m_dev, mem, nullptr);    mem = VK_NULL_HANDLE; }

        VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size  = size;
        bi.usage = usage;
        bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        VK_CHECK(vkCreateBuffer(m_dev, &bi, nullptr, &buf), "SMR: vkCreateBuffer");

        VkMemoryRequirements req{};
        vkGetBufferMemoryRequirements(m_dev, buf, &req);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize  = req.size;
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
                                            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                            m_memProps);
        VK_CHECK(vkAllocateMemory(m_dev, &ai, nullptr, &mem), "SMR: vkAllocateMemory");
        vkBindBufferMemory(m_dev, buf, mem, 0);
    };

    if (vbSize) makeBuffer(vbSize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, m_vb, m_vbMem);
    if (ibSize) makeBuffer(ibSize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT,  m_ib, m_ibMem);

    if (vbSize) {
        void* p = nullptr; vkMapMemory(m_dev, m_vbMem, 0, vbSize, 0, &p);
        std::memcpy(p, verts.data(), size_t(vbSize));
        vkUnmapMemory(m_dev, m_vbMem);
    }
    if (ibSize) {
        void* p = nullptr; vkMapMemory(m_dev, m_ibMem, 0, ibSize, 0, &p);
        std::memcpy(p, indices.data(), size_t(ibSize));
        vkUnmapMemory(m_dev, m_ibMem);
    }

    m_indexCount = static_cast<uint32_t>(indices.size());
}

void StandardMeshRenderer::SetLines(const std::vector<glm::vec3>& linePositions,
                                    glm::vec3 color)
{
    // Pack as Vertex (pos+color)
    std::vector<Vertex> tmp;
    tmp.reserve(linePositions.size());
    for (auto& p : linePositions) {
        Vertex v{};
        v.position = { p.x, p.y, p.z, 1.0f };
        v.normal   = { 0,0,1,0 };
        v.color    = { color.r, color.g, color.b, 1.0f };
        tmp.push_back(v);
    }

    const VkDeviceSize sz = sizeof(Vertex) * tmp.size();

    if (m_lineVB) { vkDestroyBuffer(m_dev, m_lineVB, nullptr); m_lineVB = VK_NULL_HANDLE; }
    if (m_lineMem){ vkFreeMemory(m_dev,  m_lineMem, nullptr);  m_lineMem = VK_NULL_HANDLE; }

    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size  = sz;
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VK_CHECK(vkCreateBuffer(m_dev, &bi, nullptr, &m_lineVB), "SMR: create line VB");

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(m_dev, m_lineVB, &req);

    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize  = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                        m_memProps);
    VK_CHECK(vkAllocateMemory(m_dev, &ai, nullptr, &m_lineMem), "SMR: alloc line mem");
    vkBindBufferMemory(m_dev, m_lineVB, m_lineMem, 0);

    if (sz) {
        void* p = nullptr; vkMapMemory(m_dev, m_lineMem, 0, sz, 0, &p);
        std::memcpy(p, tmp.data(), size_t(sz));
        vkUnmapMemory(m_dev, m_lineMem);
    }
    m_lineVertCount = static_cast<uint32_t>(tmp.size());
}

// Draw with dynamic viewport/scissor
void StandardMeshRenderer::Draw(VkCommandBuffer cmd, VkExtent2D extent) const
{
    VkViewport viewport{};
    viewport.x = 0.0f; viewport.y = 0.0f;
    viewport.width  = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0001f;
    viewport.maxDepth = 15.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);

    VkRect2D scissor{{0,0}, extent};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    if (m_vb && m_ib && m_indexCount > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeTriangles);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &m_vb, &off);
        vkCmdBindIndexBuffer(cmd, m_ib, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, m_indexCount, 1, 0, 0, 0);
    }

    if (m_lineVB && m_lineVertCount >= 2) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeLines);
        VkDeviceSize off = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &m_lineVB, &off);
        vkCmdDraw(cmd, m_lineVertCount, 1, 0, 0);
    }
}

void StandardMeshRenderer::SetColoredLines(const std::vector<Vertex>& lineVerts)
{
    const VkDeviceSize sz = sizeof(Vertex) * lineVerts.size();

    if (m_lineVB)  { vkDestroyBuffer(m_dev, m_lineVB, nullptr);  m_lineVB  = VK_NULL_HANDLE; }
    if (m_lineMem) { vkFreeMemory  (m_dev, m_lineMem, nullptr);  m_lineMem = VK_NULL_HANDLE; }

    if (sz == 0) { m_lineVertCount = 0; return; }

    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size  = sz;
    bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    VK_CHECK(vkCreateBuffer(m_dev, &bi, nullptr, &m_lineVB), "SMR: create colored line VB");

    VkMemoryRequirements req{};
    vkGetBufferMemoryRequirements(m_dev, m_lineVB, &req);

    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize  = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                                        m_memProps);
    VK_CHECK(vkAllocateMemory(m_dev, &ai, nullptr, &m_lineMem), "SMR: alloc colored line mem");
    vkBindBufferMemory(m_dev, m_lineVB, m_lineMem, 0);

    void* p = nullptr; vkMapMemory(m_dev, m_lineMem, 0, sz, 0, &p);
    std::memcpy(p, lineVerts.data(), size_t(sz));
    vkUnmapMemory(m_dev, m_lineMem);

    m_lineVertCount = static_cast<uint32_t>(lineVerts.size());
}

