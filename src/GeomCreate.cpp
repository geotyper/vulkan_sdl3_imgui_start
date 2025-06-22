#include "GeomCreate.h"
#include <cmath>
#include <stdexcept>
#include <cstring>
#include <map>
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>
#include "VulkanHelperMethods.h"

// === Vertex Input Descriptions ===
VkVertexInputBindingDescription GeomCreate::getBindingDescription() {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(Vertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return binding;
}

std::vector<VkVertexInputAttributeDescription> GeomCreate::getAttributeDescriptions() {
    std::vector<VkVertexInputAttributeDescription> attrs(2);
    attrs[0].binding = 0;
    attrs[0].location = 0;
    attrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attrs[0].offset = offsetof(Vertex, position);

    attrs[1].binding = 0;
    attrs[1].location = 1;
    attrs[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    attrs[1].offset = offsetof(Vertex, normal);

    return attrs;
}

// === UV Sphere ===
void GeomCreate::createUVSphere(uint32_t latDiv, uint32_t lonDiv,
                                std::vector<Vertex>& outVertices,
                                std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();

    for (uint32_t lat = 0; lat <= latDiv; ++lat) {
        float theta = static_cast<float>(lat) * glm::pi<float>() / static_cast<float>(latDiv);
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        for (uint32_t lon = 0; lon <= lonDiv; ++lon) {
            float phi = static_cast<float>(lon) * glm::two_pi<float>() / static_cast<float>(lonDiv);
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            glm::vec3 pos = glm::vec3(cosPhi * sinTheta, cosTheta, sinPhi * sinTheta);
            outVertices.push_back({ pos, glm::normalize(pos) });
        }
    }

    for (uint32_t lat = 0; lat < latDiv; ++lat) {
        for (uint32_t lon = 0; lon < lonDiv; ++lon) {
            uint32_t first = lat * (lonDiv + 1) + lon;
            uint32_t second = first + lonDiv + 1;

            outIndices.push_back(first);
            outIndices.push_back(second);
            outIndices.push_back(first + 1);

            outIndices.push_back(second);
            outIndices.push_back(second + 1);
            outIndices.push_back(first + 1);
        }
    }
}

// === Low-Poly Sphere (dodecahedron-like for test) ===
void GeomCreate::createLowPolySphere(std::vector<Vertex>& outVertices,
                                     std::vector<uint32_t>& outIndices) {
    std::vector<glm::vec3> positions = {
        { 0.0f,  0.0f,  1.0f}, { 0.894f,  0.0f,  0.447f}, { 0.276f,  0.851f,  0.447f},
        {-0.724f,  0.526f,  0.447f}, {-0.724f, -0.526f,  0.447f}, { 0.276f, -0.851f,  0.447f},
        { 0.724f,  0.526f, -0.447f}, {-0.276f,  0.851f, -0.447f}, {-0.894f,  0.0f, -0.447f},
        {-0.276f, -0.851f, -0.447f}, { 0.724f, -0.526f, -0.447f}, { 0.0f,  0.0f, -1.0f}
    };

    outVertices.clear();
    for (const auto& pos : positions) {
        outVertices.push_back({ pos, glm::normalize(pos) });
    }

    outIndices = {
        0, 1, 2,  0, 2, 3,  0, 3, 4,  0, 4, 5,  0, 5, 1,
        1, 6, 2,  2, 7, 3,  3, 8, 4,  4, 9, 5,  5,10, 1,
        6, 7, 2,  7, 8, 3,  8, 9, 4,  9,10, 5, 10, 6, 1,
        6,11, 7,  7,11, 8,  8,11, 9,  9,11,10, 10,11, 6
    };
}

// === Icosphere (basic recursive subdivision of icosahedron) ===
namespace {
glm::vec3 normalize(const glm::vec3& v) {
    return glm::normalize(v);
}

uint32_t addVertex(glm::vec3 v, std::vector<Vertex>& verts,
                   std::map<std::pair<uint32_t, uint32_t>, uint32_t>& midpointCache) {
    verts.push_back({ normalize(v), normalize(v) });
    return static_cast<uint32_t>(verts.size() - 1);
}

uint32_t getMidpoint(uint32_t a, uint32_t b,
                     std::vector<Vertex>& verts,
                     std::map<std::pair<uint32_t, uint32_t>, uint32_t>& cache) {
    auto edge = std::minmax(a, b);
    auto it = cache.find(edge);
    if (it != cache.end()) return it->second;

    glm::vec3 mid = (verts[a].position + verts[b].position) * 0.5f;
    uint32_t idx = addVertex(mid, verts, cache);
    cache[edge] = idx;
    return idx;
}
}

void GeomCreate::createIcosphere(uint32_t subdivisions,
                                 std::vector<Vertex>& outVertices,
                                 std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();

    const float X = 0.525731f;
    const float Z = 0.850651f;
    const glm::vec3 vdata[] = {
        {-X, 0, Z}, {X, 0, Z}, {-X, 0, -Z}, {X, 0, -Z},
        {0, Z, X}, {0, Z, -X}, {0, -Z, X}, {0, -Z, -X},
        {Z, X, 0}, {-Z, X, 0}, {Z, -X, 0}, {-Z, -X, 0}
    };

    const uint32_t tdata[][3] = {
        {0, 4, 1}, {0, 9, 4}, {9, 5, 4}, {4, 5, 8}, {4, 8, 1},
        {8, 10, 1}, {8, 3, 10}, {5, 3, 8}, {5, 2, 3}, {2, 7, 3},
        {7, 10, 3}, {7, 6, 10}, {7, 11, 6}, {11, 0, 6}, {0, 1, 6},
        {6, 1, 10}, {9, 0, 11}, {9, 11, 2}, {9, 2, 5}, {7, 2, 11}
    };

    std::map<std::pair<uint32_t, uint32_t>, uint32_t> midpointCache;
    for (auto& v : vdata)
        outVertices.push_back({ normalize(v), normalize(v) });

    std::vector<std::array<uint32_t, 3>> faces;
    for (auto& tri : tdata)
        faces.push_back({tri[0], tri[1], tri[2]});

    for (uint32_t i = 0; i < subdivisions; ++i) {
        std::vector<std::array<uint32_t, 3>> newFaces;
        for (auto& tri : faces) {
            uint32_t a = getMidpoint(tri[0], tri[1], outVertices, midpointCache);
            uint32_t b = getMidpoint(tri[1], tri[2], outVertices, midpointCache);
            uint32_t c = getMidpoint(tri[2], tri[0], outVertices, midpointCache);

            newFaces.push_back({tri[0], a, c});
            newFaces.push_back({tri[1], b, a});
            newFaces.push_back({tri[2], c, b});
            newFaces.push_back({a, b, c});
        }
        faces = std::move(newFaces);
    }

    for (auto& tri : faces) {
        outIndices.push_back(tri[0]);
        outIndices.push_back(tri[1]);
        outIndices.push_back(tri[2]);
    }
}

void GeomCreate::createVertexBuffer(VkDevice device, VkPhysicalDevice physicalDevice,
                                    const std::vector<Vertex>& vertices,
                                    VkBuffer& vertexBuffer, VkDeviceMemory& vertexMemory) {
    VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();
    createBuffer(device, physicalDevice, bufferSize,
                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 vertexBuffer, vertexMemory);

    void* data;
    vkMapMemory(device, vertexMemory, 0, bufferSize, 0, &data);
    memcpy(data, vertices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, vertexMemory);
}

void GeomCreate::createIndexBuffer(VkDevice device, VkPhysicalDevice physicalDevice,
                                   const std::vector<uint32_t>& indices,
                                   VkBuffer& indexBuffer, VkDeviceMemory& indexMemory) {
    VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();
    createBuffer(device, physicalDevice, bufferSize,
                 VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 indexBuffer, indexMemory);

    void* data;
    vkMapMemory(device, indexMemory, 0, bufferSize, 0, &data);
    memcpy(data, indices.data(), (size_t)bufferSize);
    vkUnmapMemory(device, indexMemory);
}

