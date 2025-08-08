#include "GeomCreate.h"
#include <cmath>
#include <map>
#include <array>
#include <algorithm>
#include <glm/gtc/constants.hpp>

// Note: The Vulkan-specific functions like createVertexBuffer, getBindingDescription, etc.,
// have been removed as they are no longer the responsibility of this class.


VkVertexInputBindingDescription GeomCreate::getBindingDescription2() {
    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(Vertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return binding;
}

std::vector<VkVertexInputAttributeDescription> GeomCreate::getAttributeDescriptions2() {
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
            glm::vec3 normal = glm::normalize(pos);
            glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f); // Default white color
            glm::vec2 texCoord = glm::vec2(static_cast<float>(lon) / lonDiv, static_cast<float>(lat) / latDiv);
            glm::vec2 pad = glm::vec2(0, 0);

            // Push the complete vertex with all attributes
            outVertices.push_back({
                glm::vec4(pos, 1.0f),
                glm::vec4(normal, 0.0f),
                glm::vec4(color, 1.0f),
                //texCoord

                //,pad
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
// Helper function to find or create a midpoint vertex, ensuring correct vec4 layout
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

    // Correctly interpolate all vec4 attributes
    glm::vec3 pos = glm::normalize(glm::vec3(v1.position + v2.position) * 0.5f);
    glm::vec3 norm = pos; // For a perfect sphere, normal is the same as normalized position
    glm::vec3 col = glm::vec3(v1.color + v2.color) * 0.5f;
   // glm::vec2 tc = (v1.texCoord + v2.texCoord) * 0.5f;

    // Create a new vertex with the correct vec4 format
    vertices.push_back({
        glm::vec4(pos, 1.0f),
        glm::vec4(norm, 0.0f),
        glm::vec4(col, 1.0f)
        //,
       // tc,
       // {} // Padding to match the struct layout
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

    // A helper lambda to add a new vertex with all attributes in the correct vec4 format
    auto addVertex = [&](const glm::vec3& p){
        glm::vec3 normal = glm::normalize(p);
        // Use the position itself to generate UVs for a sphere
        glm::vec2 texCoord(0.5f + atan2(normal.z, normal.x) / (2.0f * glm::pi<float>()), 0.5f - asin(normal.y) / glm::pi<float>());

        outVertices.push_back({
            glm::vec4(normal, 1.0f),                    // position (w=1 for points)
            glm::vec4(normal, 0.0f),                    // normal (w=0 for vectors)
            glm::vec4(1.0f, 1.0f, 1.0f, 1.0f)
            //,          // color (white)
            //texCoord,                                   // uv
            //{}                                          // padding to match the struct layout
        });
    };

    // Create the 12 base vertices of the icosahedron
    addVertex({-1,  t,  0}); addVertex({ 1,  t,  0}); addVertex({-1, -t,  0}); addVertex({ 1, -t,  0});
    addVertex({ 0, -1,  t}); addVertex({ 0,  1,  t}); addVertex({ 0, -1, -t}); addVertex({ 0,  1, -t});
    addVertex({ t,  0, -1}); addVertex({ t,  0,  1}); addVertex({-t,  0, -1}); addVertex({-t,  0,  1});

    // Create the 20 base faces
    std::vector<std::array<uint32_t, 3>> faces = {
        {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
        {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
        {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
        {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
    };

    // Subdivide the faces recursively
    for (uint32_t i = 0; i < subdivisions; i++) {
        std::vector<std::array<uint32_t, 3>> faces2;
        faces2.reserve(faces.size() * 4);
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

    // Populate the final index buffer
    outIndices.clear();
    outIndices.reserve(faces.size() * 3);
    for(const auto& tri : faces) {
        outIndices.push_back(tri[0]);
        outIndices.push_back(tri[1]);
        outIndices.push_back(tri[2]);
    }
}

void GeomCreate::createCube(std::vector<Vertex>& outVertices, std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();

    glm::vec3 positions[] = {
        // Front face
        {-0.5f, -0.5f,  0.5f},  // 0
        { 0.5f, -0.5f,  0.5f},  // 1
        { 0.5f,  0.5f,  0.5f},  // 2
        {-0.5f,  0.5f,  0.5f},  // 3

        // Back face
        {-0.5f, -0.5f, -0.5f},  // 4
        { 0.5f, -0.5f, -0.5f},  // 5
        { 0.5f,  0.5f, -0.5f},  // 6
        {-0.5f,  0.5f, -0.5f},  // 7
    };

    glm::vec3 normals[] = {
        { 0,  0,  1},  // Front
        { 0,  0, -1},  // Back
        { 1,  0,  0},  // Right
        {-1,  0,  0},  // Left
        { 0,  1,  0},  // Top
        { 0, -1,  0},  // Bottom
    };

    auto addQuad = [&](int i0, int i1, int i2, int i3, const glm::vec3& normal) {
        uint32_t baseIndex = static_cast<uint32_t>(outVertices.size());

        outVertices.push_back({ glm::vec4(positions[i0], 1.0f), glm::vec4(normal, 0.0f), glm::vec4(1.0f), });
        outVertices.push_back({ glm::vec4(positions[i1], 1.0f), glm::vec4(normal, 0.0f), glm::vec4(1.0f), });
        outVertices.push_back({ glm::vec4(positions[i2], 1.0f), glm::vec4(normal, 0.0f), glm::vec4(1.0f), });
        outVertices.push_back({ glm::vec4(positions[i3], 1.0f), glm::vec4(normal, 0.0f), glm::vec4(1.0f), });

        outIndices.push_back(baseIndex);
        outIndices.push_back(baseIndex + 1);
        outIndices.push_back(baseIndex + 2);

        outIndices.push_back(baseIndex);
        outIndices.push_back(baseIndex + 2);
        outIndices.push_back(baseIndex + 3);
    };

    // Faces: front, back, right, left, top, bottom
    addQuad(0, 1, 2, 3, normals[0]);  // Front
    addQuad(5, 4, 7, 6, normals[1]);  // Back
    addQuad(1, 5, 6, 2, normals[2]);  // Right
    addQuad(4, 0, 3, 7, normals[3]);  // Left
    addQuad(3, 2, 6, 7, normals[4]);  // Top
    addQuad(4, 5, 1, 0, normals[5]);  // Bottom
}


void GeomCreate::createCube2(std::vector<Vertex>& outVertices, std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();

    const glm::vec3 faceNormals[] = {
        { 0,  0,  1},  // Front
        { 0,  0, -1},  // Back
        { 1,  0,  0},  // Right
        {-1,  0,  0},  // Left
        { 0,  1,  0},  // Top
        { 0, -1,  0}   // Bottom
    };

    const glm::vec3 faceVertices[][4] = {
        // Front (+Z)
        { {-0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f} },
        // Back (-Z)
        { { 0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f} },
        // Right (+X)
        { { 0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f,  0.5f} },
        // Left (-X)
        { {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f, -0.5f} },
        // Top (+Y)
        { {-0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f} },
        // Bottom (-Y)
        { {-0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f,  0.5f}, {-0.5f, -0.5f,  0.5f} },
        };

    for (int face = 0; face < 6; ++face) {
        glm::vec4 normal = glm::vec4(faceNormals[face], 0.0f);
        glm::vec4 color  = glm::vec4(1.0f); // White color

        uint32_t baseIndex = static_cast<uint32_t>(outVertices.size());

        // Add 4 vertices per face
        for (int i = 0; i < 4; ++i) {
            outVertices.push_back({
                glm::vec4(faceVertices[face][i], 1.0f),
                normal,
                color
            });
        }

        // Two triangles per face (0-1-2 and 0-2-3)
        outIndices.push_back(baseIndex + 0);
        outIndices.push_back(baseIndex + 1);
        outIndices.push_back(baseIndex + 2);

        outIndices.push_back(baseIndex + 0);
        outIndices.push_back(baseIndex + 2);
        outIndices.push_back(baseIndex + 3);
    }
}

// === Единичный куб ===
// Создает единичный куб с центром в начале координат и длиной ребра 1.0.
// Генерирует 24 вершины, чтобы у каждой грани были правильные нормали.
void GeomCreate::createCube3(std::vector<Vertex>& outVertices,
                            std::vector<uint32_t>& outIndices) {
    outVertices.clear();
    outIndices.clear();

    // Определяем 24 вершины. Каждая грань имеет 4 уникальные вершины
    // с одинаковой нормалью, направленной перпендикулярно грани.
    // Позиции вершин находятся в диапазоне от -0.5 до 0.5.
    outVertices = {
        // Передняя грань (+Z)
        { {-0.5f, -0.5f,  0.5f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f,  0.5f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f,  0.5f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f,  0.5f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Задняя грань (-Z)
        { {-0.5f, -0.5f, -0.5f, 1.0f}, {0.0f, 0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f, -0.5f, 1.0f}, {0.0f, 0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f, -0.5f, 1.0f}, {0.0f, 0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f, -0.5f, 1.0f}, {0.0f, 0.0f, -1.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Левая грань (-X)
        { {-0.5f, -0.5f, -0.5f, 1.0f}, {-1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f, -0.5f,  0.5f, 1.0f}, {-1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f,  0.5f, 1.0f}, {-1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f, -0.5f, 1.0f}, {-1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Правая грань (+X)
        { { 0.5f, -0.5f, -0.5f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f, -0.5f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f,  0.5f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f,  0.5f, 1.0f}, {1.0f, 0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Нижняя грань (-Y)
        { {-0.5f, -0.5f, -0.5f, 1.0f}, {0.0f, -1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f, -0.5f, 1.0f}, {0.0f, -1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f, -0.5f,  0.5f, 1.0f}, {0.0f, -1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f, -0.5f,  0.5f, 1.0f}, {0.0f, -1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },

        // Верхняя грань (+Y)
        { {-0.5f,  0.5f, -0.5f, 1.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { {-0.5f,  0.5f,  0.5f, 1.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f,  0.5f, 1.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        { { 0.5f,  0.5f, -0.5f, 1.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f, 1.0f} },
        };

    // Определяем 36 индексов для создания 12 треугольников (2 на грань)
    outIndices = {
        0, 1, 2,   2, 3, 0,       // Передняя грань
        4, 5, 6,   6, 7, 4,       // Задняя грань
        8, 9, 10,  10, 11, 8,      // Левая грань
        12, 13, 14, 14, 15, 12,    // Правая грань
        16, 17, 18, 18, 19, 16,    // Нижняя грань
        20, 21, 22, 22, 23, 20     // Верхняя грань
    };
}

void GeomCreate::createCubeGrid(std::vector<Vertex>& outVertices, std::vector<uint32_t>& outIndices, int N) {
    outVertices.clear();
    outIndices.clear();

    const glm::vec3 faceNormals[] = {
        { 0,  0,  1},  // Front
        { 0,  0, -1},  // Back
        { 1,  0,  0},  // Right
        {-1,  0,  0},  // Left
        { 0,  1,  0},  // Top
        { 0, -1,  0}   // Bottom
    };

    const glm::vec3 faceCorners[][4] = {
        // Front (+Z)
        { {-0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f} },
        // Back (-Z)
        { { 0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f} },
        // Right (+X)
        { { 0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f,  0.5f} },
        // Left (-X)
        { {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f, -0.5f} },
        // Top (+Y)
        { {-0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f} },
        // Bottom (-Y)
        { {-0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f,  0.5f}, {-0.5f, -0.5f,  0.5f} },
        };

    for (int face = 0; face < 6; ++face) {
        glm::vec3 normal = faceNormals[face];
        glm::vec4 color  = glm::vec4(1.0f); // white

        glm::vec3 v00 = faceCorners[face][0]; // bottom-left
        glm::vec3 v10 = faceCorners[face][1]; // bottom-right
        glm::vec3 v11 = faceCorners[face][2]; // top-right
        glm::vec3 v01 = faceCorners[face][3]; // top-left

        uint32_t baseIndex = static_cast<uint32_t>(outVertices.size());

        // Generate (N+1)*(N+1) grid points per face
        for (int y = 0; y <= N; ++y) {
            float fy = static_cast<float>(y) / N;
            for (int x = 0; x <= N; ++x) {
                float fx = static_cast<float>(x) / N;

                // Bilinear interpolation
                glm::vec3 bottom = glm::mix(v00, v10, fx);
                glm::vec3 top    = glm::mix(v01, v11, fx);
                glm::vec3 pos    = glm::mix(bottom, top, fy);

                outVertices.push_back({
                    glm::vec4(pos, 1.0f),
                    glm::vec4(normal, 0.0f),
                    color
                });
            }
        }

        // Generate 2 triangles per cell
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                uint32_t i0 = baseIndex + (y    ) * (N + 1) + x;
                uint32_t i1 = baseIndex + (y    ) * (N + 1) + x + 1;
                uint32_t i2 = baseIndex + (y + 1) * (N + 1) + x + 1;
                uint32_t i3 = baseIndex + (y + 1) * (N + 1) + x;

                outIndices.push_back(i0);
                outIndices.push_back(i1);
                outIndices.push_back(i2);

                outIndices.push_back(i0);
                outIndices.push_back(i2);
                outIndices.push_back(i3);
            }
        }
    }
}



void GeomCreate::createHollowCube(std::vector<Vertex>& outVertices, std::vector<uint32_t>& outIndices, int N, float holeSize) {
    outVertices.clear();
    outIndices.clear();

    if (N < 3) N = 3;

    // ========================================================================
    // Часть 1: Внешние грани с отверстиями (этот блок остался без изменений)
    // ========================================================================
    const glm::vec3 faceNormals[] = {
        { 0,  0,  1}, { 0,  0, -1}, { 1,  0,  0}, {-1,  0,  0}, { 0,  1,  0}, { 0, -1,  0}
    };
    const glm::vec3 faceCorners[][4] = {
                                        { {-0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f} },
                                        { { 0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f} },
                                        { { 0.5f, -0.5f,  0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f,  0.5f, -0.5f}, { 0.5f,  0.5f,  0.5f} },
                                        { {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f,  0.5f}, {-0.5f,  0.5f,  0.5f}, {-0.5f,  0.5f, -0.5f} },
                                        { {-0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f,  0.5f}, { 0.5f,  0.5f, -0.5f}, {-0.5f,  0.5f, -0.5f} },
                                        { {-0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f, -0.5f}, { 0.5f, -0.5f,  0.5f}, {-0.5f, -0.5f,  0.5f} },
                                        };

    int startHole = static_cast<int>(N * (1.0f - holeSize) / 2.0f);
    int endHole = N - startHole;

    for (int face = 0; face < 6; ++face) {
        uint32_t baseIndex = static_cast<uint32_t>(outVertices.size());
        for (int y = 0; y <= N; ++y) {
            for (int x = 0; x <= N; ++x) {
                float fx = static_cast<float>(x) / N;
                float fy = static_cast<float>(y) / N;
                glm::vec3 p = glm::mix(glm::mix(faceCorners[face][0], faceCorners[face][1], fx),
                                       glm::mix(faceCorners[face][3], faceCorners[face][2], fx), fy);
                outVertices.push_back({glm::vec4(p, 1.0f), glm::vec4(faceNormals[face], 0.0f), glm::vec4(1.0f)});
            }
        }
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {
                if (x >= startHole && x < endHole && y >= startHole && y < endHole) {
                    continue;
                }
                uint32_t i0 = baseIndex + (y) * (N + 1) + x;
                uint32_t i1 = baseIndex + (y) * (N + 1) + x + 1;
                uint32_t i2 = baseIndex + (y + 1) * (N + 1) + x + 1;
                uint32_t i3 = baseIndex + (y + 1) * (N + 1) + x;
                outIndices.push_back(i0); outIndices.push_back(i1); outIndices.push_back(i2);
                outIndices.push_back(i0); outIndices.push_back(i2); outIndices.push_back(i3);
            }
        }
    }

    // ========================================================================
    // Часть 2: Внутренние стенки туннелей (ИСПРАВЛЕННАЯ ЛОГИКА)
    // ========================================================================
    float h = holeSize / 2.0f;

    // ✅ ИСПРАВЛЕННЫЕ ВЕРШИНЫ И НОРМАЛИ
    const glm::vec3 innerVertices[][4] = {
        // Туннель по Z (Front/Back)
        { {-h,  h, -0.5f}, { h,  h, -0.5f}, { h,  h,  0.5f}, {-h,  h,  0.5f} }, // Верхняя стенка
        { {-h, -h,  0.5f}, { h, -h,  0.5f}, { h, -h, -0.5f}, {-h, -h, -0.5f} }, // Нижняя стенка
        { { h, -h,  0.5f}, { h,  h,  0.5f}, { h,  h, -0.5f}, { h, -h, -0.5f} }, // Правая стенка
        { {-h, -h, -0.5f}, {-h,  h, -0.5f}, {-h,  h,  0.5f}, {-h, -h,  0.5f} }, // Левая стенка

        // Туннель по Y (Top/Bottom)
        { {-h,  0.5f,  h}, { h,  0.5f,  h}, { h, -0.5f,  h}, {-h, -0.5f,  h} }, // Передняя стенка (+Z)
        { { h,  0.5f, -h}, {-h,  0.5f, -h}, {-h, -0.5f, -h}, { h, -0.5f, -h} }, // Задняя стенка (-Z)
        { { h,  0.5f,  h}, { h,  0.5f, -h}, { h, -0.5f, -h}, { h, -0.5f,  h} }, // Правая стенка (+X)
        { {-h,  0.5f, -h}, {-h,  0.5f,  h}, {-h, -0.5f,  h}, {-h, -0.5f, -h} }, // Левая стенка (-X)

        // Туннель по X (Left/Right)
        { { 0.5f,  h, -h}, { 0.5f,  h,  h}, { 0.5f, -h,  h}, { 0.5f, -h, -h} }, // Правая передняя стенка (+Z)
        { {-0.5f,  h,  h}, {-0.5f,  h, -h}, {-0.5f, -h, -h}, {-0.5f, -h,  h} }, // Левая передняя стенка (+Z)
        { {-0.5f,  h,  h}, { 0.5f,  h,  h}, { 0.5f,  h, -h}, {-0.5f,  h, -h} }, // Верхняя стенка (+Y)
        { {-0.5f, -h, -h}, { 0.5f, -h, -h}, { 0.5f, -h,  h}, {-0.5f, -h,  h} }  // Нижняя стенка (-Y)
    };
    const glm::vec3 innerNormals[] = {
        { 0, -1,  0}, { 0,  1,  0}, {-1,  0,  0}, { 1,  0,  0}, // Туннель Z
        { 0,  0, -1}, { 0,  0,  1}, {-1,  0,  0}, { 1,  0,  0}, // Туннель Y
        { 0,  0, -1}, { 0,  0,  1}, { 0, -1,  0}, { 0,  1,  0}  // Туннель X
    };

    for (int i = 0; i < 12; ++i) {
        uint32_t baseIndex = static_cast<uint32_t>(outVertices.size());
        glm::vec3 normal = innerNormals[i];
        for (int j = 0; j < 4; ++j) {
            outVertices.push_back({glm::vec4(innerVertices[i][j], 1.0f), glm::vec4(normal, 0.0f), glm::vec4(1.0f)});
        }
        outIndices.push_back(baseIndex); outIndices.push_back(baseIndex + 1); outIndices.push_back(baseIndex + 2);
        outIndices.push_back(baseIndex); outIndices.push_back(baseIndex + 2); outIndices.push_back(baseIndex + 3);
    }
}

// Вырезает одно квадратное «окно» на каждой стороне куба.
//  - N            : число ячеек по стороне (должно быть чётным или кратным 3 — см. комментарий).
//  - holeScale    : относительная ширина окна (0.0f..1.0f). 0.5f = окно в половину стороны.
void GeomCreate::createCubeWithSquareHole(std::vector<Vertex>& outVertices,
                                          std::vector<uint32_t>& outIndices,
                                          int N,
                                          float holeScale /*=0.4f*/)
{
    outVertices.clear();
    outIndices .clear();

    /* --- исходные данные ваших граней --- */
    const glm::vec3 faceNormals[6] = {
        { 0,  0,  1}, { 0,  0, -1}, { 1,  0,  0},
        {-1,  0,  0}, { 0,  1,  0}, { 0, -1,  0}
    };
    const glm::vec3 faceCorners[6][4] = {
        {{-0.5f,-0.5f, 0.5f},{ 0.5f,-0.5f, 0.5f},{ 0.5f, 0.5f, 0.5f},{-0.5f, 0.5f, 0.5f}}, // +Z
        {{ 0.5f,-0.5f,-0.5f},{-0.5f,-0.5f,-0.5f},{-0.5f, 0.5f,-0.5f},{ 0.5f, 0.5f,-0.5f}}, // -Z
        {{ 0.5f,-0.5f, 0.5f},{ 0.5f,-0.5f,-0.5f},{ 0.5f, 0.5f,-0.5f},{ 0.5f, 0.5f, 0.5f}}, // +X
        {{-0.5f,-0.5f,-0.5f},{-0.5f,-0.5f, 0.5f},{-0.5f, 0.5f, 0.5f},{-0.5f, 0.5f,-0.5f}}, // -X
        {{-0.5f, 0.5f, 0.5f},{ 0.5f, 0.5f, 0.5f},{ 0.5f, 0.5f,-0.5f},{-0.5f, 0.5f,-0.5f}}, // +Y
        {{-0.5f,-0.5f,-0.5f},{ 0.5f,-0.5f,-0.5f},{ 0.5f,-0.5f, 0.5f},{-0.5f,-0.5f, 0.5f}}, // -Y
    };

    /* --- как много ячеек оставить под окно --- */
    holeScale      = glm::clamp(holeScale, 0.05f, 0.95f);
    int holeCells  = std::max(1, int(N * holeScale + 0.5f));   // ширина окна в ячейках
    if (holeCells >= N) holeCells = N - 1;                     // окно не может полностью съесть грань
    int holeOffset = (N - holeCells) / 2;                      // смещение окна от левого/нижнего краёв
    int hx0 = holeOffset,          hy0 = holeOffset;           // начало окна по X/Y (ячейка)
    int hx1 = hx0 + holeCells;     int hy1 = hy0 + holeCells;  // конец   окна (EXclusive!)

    for (int face = 0; face < 6; ++face) {
        glm::vec3 normal = faceNormals[face];
        glm::vec4 color  = glm::vec4(1.0f);

        glm::vec3 v00 = faceCorners[face][0], v10 = faceCorners[face][1];
        glm::vec3 v11 = faceCorners[face][2], v01 = faceCorners[face][3];

        uint32_t base = uint32_t(outVertices.size());

        /* --- вершины ─ всё как раньше --- */
        for (int y = 0; y <= N; ++y) {
            float fy = float(y) / N;
            for (int x = 0; x <= N; ++x) {
                float fx = float(x) / N;

                glm::vec3 pos = glm::mix(glm::mix(v00, v10, fx),
                                         glm::mix(v01, v11, fx), fy);
                outVertices.push_back({ glm::vec4(pos,1.0f),
                                       glm::vec4(normal,0.0f),
                                       color });
            }
        }

        /* --- индексы: пропускаем центральную «дырку» --- */
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {

                // Если текущая ячейка внутри окна – не создаём треугольники
                if (x >= hx0 && x < hx1 && y >= hy0 && y < hy1)
                    continue;

                uint32_t i0 = base +  y      * (N + 1) +  x;
                uint32_t i1 = base +  y      * (N + 1) + (x + 1);
                uint32_t i2 = base + (y + 1) * (N + 1) + (x + 1);
                uint32_t i3 = base + (y + 1) * (N + 1) +  x;

                outIndices.insert(outIndices.end(),
                                  { i0, i1, i2,   i0, i2, i3 });
            }
        }
    }
}

//---------------------------------------------------------------------
// Создаёт куб, на каждой грани которого вырезан центральный квадрат-окно.
//  N          : сетка N×N ячеек на грани.
//  holeCells  : ширина окна в целых ячейках (должна быть < N
//               и иметь ту же чётность, что и N).
void GeomCreate::createCubeCenterHole(std::vector<Vertex>& outVertices,
                                      std::vector<uint32_t>& outIndices,
                                      int N,
                                      int holeCells)
{
    outVertices.clear();
    outIndices .clear();

    /* --- проверки параметров --- */
    //if (N <= 0)
    //    throw std::runtime_error("N must be > 0");
    //if (holeCells <= 0 || holeCells >= N)
    //    throw std::runtime_error("holeCells must be in range (0, N)");
    //if ((N - holeCells) & 1)
    //    throw std::runtime_error("N - holeCells must be even for a centered hole");

    const glm::vec3 faceNormals[6] = {
        { 0,  0,  1}, { 0,  0, -1}, { 1,  0,  0},
        {-1,  0,  0}, { 0,  1,  0}, { 0, -1,  0}
    };
    const glm::vec3 faceCorners[6][4] = {
        {{-0.5f,-0.5f, 0.5f},{ 0.5f,-0.5f, 0.5f},{ 0.5f, 0.5f, 0.5f},{-0.5f, 0.5f, 0.5f}}, // +Z
        {{ 0.5f,-0.5f,-0.5f},{-0.5f,-0.5f,-0.5f},{-0.5f, 0.5f,-0.5f},{ 0.5f, 0.5f,-0.5f}}, // -Z
        {{ 0.5f,-0.5f, 0.5f},{ 0.5f,-0.5f,-0.5f},{ 0.5f, 0.5f,-0.5f},{ 0.5f, 0.5f, 0.5f}}, // +X
        {{-0.5f,-0.5f,-0.5f},{-0.5f,-0.5f, 0.5f},{-0.5f, 0.5f, 0.5f},{-0.5f, 0.5f,-0.5f}}, // -X
        {{-0.5f, 0.5f, 0.5f},{ 0.5f, 0.5f, 0.5f},{ 0.5f, 0.5f,-0.5f},{-0.5f, 0.5f,-0.5f}}, // +Y
        {{-0.5f,-0.5f,-0.5f},{ 0.5f,-0.5f,-0.5f},{ 0.5f,-0.5f, 0.5f},{-0.5f,-0.5f, 0.5f}}, // -Y
    };

    /* --- диапазон ячеек, которые надо вырезать --- */
    int border   = (N - holeCells) / 2; // толщина рамки в ячейках
    int hx0 = border;                   // первая вырезаемая ячейка по X
    int hx1 = hx0 + holeCells;          // EXclusive
    int hy0 = border;
    int hy1 = hy0 + holeCells;

    for (int face = 0; face < 6; ++face)
    {
        glm::vec3 normal = faceNormals[face];
        glm::vec4 color  = glm::vec4(1.0f);

        glm::vec3 v00 = faceCorners[face][0];
        glm::vec3 v10 = faceCorners[face][1];
        glm::vec3 v11 = faceCorners[face][2];
        glm::vec3 v01 = faceCorners[face][3];

        uint32_t base = static_cast<uint32_t>(outVertices.size());

        /* ----- вершины ----- */
        for (int y = 0; y <= N; ++y) {
            float fy = static_cast<float>(y) / N;
            for (int x = 0; x <= N; ++x) {
                float fx = static_cast<float>(x) / N;

                glm::vec3 pos = glm::mix(glm::mix(v00, v10, fx),
                                         glm::mix(v01, v11, fx), fy);

                outVertices.push_back({
                    glm::vec4(pos, 1.0f),
                    glm::vec4(normal, 0.0f),
                    color
                });
            }
        }

        /* ----- индексы: пропускаем centrale окно ----- */
        for (int y = 0; y < N; ++y) {
            for (int x = 0; x < N; ++x) {

                bool inHole = (x >= hx0 && x < hx1 &&
                               y >= hy0 && y < hy1);
                if (inHole) continue;

                uint32_t i0 = base +  y      * (N + 1) +  x;
                uint32_t i1 = base +  y      * (N + 1) + (x + 1);
                uint32_t i2 = base + (y + 1) * (N + 1) + (x + 1);
                uint32_t i3 = base + (y + 1) * (N + 1) +  x;

                outIndices.insert(outIndices.end(), { i0,i1,i2,  i0,i2,i3 });
            }
        }
    }
}
