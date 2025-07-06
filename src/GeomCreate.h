// GeomCreate.h
#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <glm/glm.hpp>
#include "HelpStructures.h"
#define GLM_ENABLE_EXPERIMENTAL

class GeomCreate {
public:
    // === Sphere Generators ===

    // UV Sphere (latitude-longitude grid)
    static void createUVSphere(uint32_t latDiv, uint32_t lonDiv,
                               std::vector<Vertex>& outVertices,
                               std::vector<uint32_t>& outIndices);

    // Icosphere (based on subdivided icosahedron)
    static void createIcosphere(uint32_t subdivisions,
                                std::vector<Vertex>& outVertices,
                                std::vector<uint32_t>& outIndices);

    // Hardcoded low-poly sphere for testing
    static void createLowPolySphere(
        std::vector<Vertex>& outVertices,
        std::vector<uint32_t>& outIndices);

    static VkVertexInputBindingDescription getBindingDescription2();
    static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions2();

    static void createCube(std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices);
    static void createCube2(std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices);
    static void createCube3(std::vector<Vertex> &outVertices, std::vector<uint32_t> &outIndices);
};
