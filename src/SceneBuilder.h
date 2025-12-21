#pragma once

#include <vector>
#include <memory> 
#include "HelpStructures.h" 

// Forward declarations
namespace rtx {
    class RayTracingModule;
}
class StandardMeshRenderer;

#include <box2d/box2d.h>

class SceneBuilder {
public:
    SceneBuilder();
    ~SceneBuilder();

    // Builds a new scene (resets physics world)
    void BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer, const SolverParameters& params);
    
    // Steps the physics simulation and updates RTX instances
    void UpdatePhysics(float dt, rtx::RayTracingModule* rtxModule, const SolverParameters& params);
    
    void RestartSimulation();

private:
    void CleanupPhysics();

    b2WorldId m_worldId = b2_nullWorldId;
    std::vector<b2BodyId> m_discBodies;
    b2BodyId m_boundaryBody = b2_nullBodyId;
    
    bool m_physicsInitialized = false;
    float m_accumTime = 0.0f;
    uint32_t m_loadedMeshCount = 0;
    
    // For mapping bodies to initial mesh instances
    struct BodyInfo {
        b2BodyId bodyId;
        int layer;
        uint32_t colorID;
        uint32_t meshID; 
        glm::vec2 initialPos;
    };
    std::vector<BodyInfo> m_bodyInfos;
};
