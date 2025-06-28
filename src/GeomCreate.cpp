#include "GeomCreate.h"
#include <cmath>
#include <map>
#include <array>
#include <algorithm>
#include <glm/gtc/constants.hpp>

// Note: The Vulkan-specific functions like createVertexBuffer, getBindingDescription, etc.,
// have been removed as they are no longer the responsibility of this class.

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
            glm::vec3 normal = glm::normalize(pos);
            glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f); // Default white color
            glm::vec2 texCoord = glm::vec2(static_cast<float>(lon) / lonDiv, static_cast<float>(lat) / latDiv);

            // Push the complete vertex with all attributes
            outVertices.push_back({
                glm::vec4(pos, 1.0f),
                glm::vec4(normal, 0.0f),
                glm::vec4(color, 1.0f),
                texCoord
            });
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


// === Icosphere (basic recursive subdivision of icosahedron) ===
namespace {
// Helper function to find or create a midpoint vertex to avoid duplicates
uint32_t getMidpoint(uint32_t p1, uint32_t p2, std::vector<Vertex>& vertices, std::map<int64_t, uint32_t>& cache) {
    int64_t smallerIndex = std::min(p1, p2);
    int64_t greaterIndex = std::max(p1, p2);
    int64_t key = (smallerIndex << 32) + greaterIndex;

    auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }

    const Vertex& v1 = vertices[p1];
    const Vertex& v2 = vertices[p2];

    // Interpolate all vertex attributes
    glm::vec3 pos =    glm::normalize((v1.position + v2.position) * 0.5f);
    glm::vec3 normal = glm::normalize((v1.normal + v2.normal) * 0.5f); // Same as pos for a sphere
    glm::vec3 color = (v1.color + v2.color) * 0.5f;
    glm::vec2 texCoord= (v1.texCoord + v2.texCoord) * 0.5f;

    //vertices.push_back({pos, norm, col, tc});

    vertices.push_back({
        glm::vec4(pos, 1.0f),
        glm::vec4(normal, 0.0f),
        glm::vec4(color, 1.0f),
        texCoord
    });

    uint32_t newIndex = static_cast<uint32_t>(vertices.size() - 1);
    cache[key] = newIndex;
    return newIndex;
}
}

void GeomCreate::createIcosphere(uint32_t subdivisions,
                                 std::vector<Vertex>& outVertices,
                                 std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();
    std::map<int64_t, uint32_t> midpointCache;

    const float t = (1.0f + sqrt(5.0f)) / 2.0f;

    // Create 12 vertices of an icosahedron
    auto add = [&](const glm::vec3& p){
        glm::vec3 normal = glm::normalize(p);
        glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);
        // Spherical projection for UVs
        glm::vec2 texCoord(0.5f + atan2(normal.z, normal.x) / (2.0f * glm::pi<float>()), 0.5f - asin(normal.y) / glm::pi<float>());
        outVertices.push_back({
            glm::vec4(normal, 1.0f),
            glm::vec4(normal, 0.0f),
            glm::vec4(color, 1.0f),
            texCoord
        });
    };

    add({-1,  t,  0}); add({ 1,  t,  0}); add({-1, -t,  0}); add({ 1, -t,  0});
    add({ 0, -1,  t}); add({ 0,  1,  t}); add({ 0, -1, -t}); add({ 0,  1, -t});
    add({ t,  0, -1}); add({ t,  0,  1}); add({-t,  0, -1}); add({-t,  0,  1});

    std::vector<std::array<uint32_t, 3>> faces = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };

    for (uint32_t i = 0; i < subdivisions; i++) {
        std::vector<std::array<uint32_t, 3>> faces2;
        for (const auto& tri : faces) {
            uint32_t a = getMidpoint(tri[0], tri[1], outVertices, midpointCache);
            uint32_t b = getMidpoint(tri[1], tri[2], outVertices, midpointCache);
            uint32_t c = getMidpoint(tri[2], tri[0], outVertices, midpointCache);
            faces2.push_back({tri[0], a, c});
            faces2.push_back({tri[1], b, a});
            faces2.push_back({tri[2], c, b});
            faces2.push_back({a, b, c});
        }
        faces = faces2;
    }

    outIndices.clear();
    for(const auto& tri : faces) {
        outIndices.push_back(tri[0]);
        outIndices.push_back(tri[1]);
        outIndices.push_back(tri[2]);
    }
}

