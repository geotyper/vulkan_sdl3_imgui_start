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
    bool makePyramid = false;
    int paletteID = 0;
    float mirrorHeight = 10.0f; // 0 = Default, 1 = Neon, 2 = Warm
    bool animate = false;
    bool paused = false;
    bool requestRestart = false;

    // Recording / Manual Animation Mode
    bool recording = false;
    float frameDelay = 0.5f; // Seconds to wait between captures
    float timeSinceLastCapture = 0.0f;
    int captureIndex = 0;
    bool triggerStep = false; // Internal: Execute one physics step
    int subdivisionIterations = 0;
    int boundarySides = 6;
    float rotationSpeed = 0.5f;

    int samplesPerFrame = 1;
    int maxBounces = 32;

    float colorSaturation = 1.0f;
    float absorptionFactor = 1.5f;

    float boundaryScale = 1.0f;
    float shapeHeight = 0.3f;
    float layerSpacing = 0.5f;
    bool solidTetris = true;
    bool randomRotation = true;
    float layerStagger = 0.0f;

    float iorParameter = 1.25f;
    float lensDistortion = 0.0f;
    float refractionBias = 0.0f;
    float chromaticAberration = 0.0f;
    glm::vec3 backColorTop = glm::vec3(0.6f, 0.75f, 1.0f);
    glm::vec3 backColorBot = glm::vec3(1.0f, 0.9f, 0.85f);
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
