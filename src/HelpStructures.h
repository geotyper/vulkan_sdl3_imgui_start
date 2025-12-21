#ifndef HELPSTRUCTURES_H
#define HELPSTRUCTURES_H


#include <glm/glm.hpp>


struct Vertex {
    glm::vec4 position;
    glm::vec4 normal;
    glm::vec4 color;

};

struct PushConstants {
    glm::mat4 mvp;
    glm::mat4 model;
};

struct SolverParameters {
    bool drawPolyMesh = false;
    float discRadius = 0.6f;
    bool requestRebuild = false;
    float fov = 35.0f;
    float lightIntensity = 3.0f;
    int shapeType = 0; // 0 = Disc, 1 = Poly
    int numDiscs = 7;
    int numLayers = 3;
    bool useKaleidoscope = false;
};

namespace rtx {
struct InstanceData {
    glm::mat4 transform;
    uint32_t  meshId;
    uint32_t  colorID{0}; // New field for material color index
};

struct MeshLoadData {
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<InstanceData> instances;
};
}

#endif // HELPSTRUCTURES_H
