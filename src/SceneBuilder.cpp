#include "SceneBuilder.h"

#include "RayTracingModule.h"
#include "StandardMeshRenderer.h"
#include "GeomCreate.h"
#include "CgalMeshBuilder.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <random>

#include <box2d/box2d.h>

void SceneBuilder::BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer) {
    if (!rtxModule) return;

    // -------------------------------------------------------------------------
    // Box2D Simulation Setup for Packing
    // -------------------------------------------------------------------------

    // Parameters
    const float outerRadius = 4.0f;  // Size of the container
    const float discRadius  = 0.45f; // Size of small discs
    const int   numDiscs    = 7;

    // 1. Setup World
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = {0.0f, 0.0f}; // Zero gravity for separation only
    b2WorldId worldId = b2CreateWorld(&worldDef);

    // 2. Create Static Outer Boundary (Chain Loop)
    // Approximate a circle with 32 segments
    const int segments = 32;
    std::vector<b2Vec2> boundaryPoints(segments);
    for(int i=0; i<segments; ++i) {
        float theta = 2.0f * 3.14159f * float(i) / float(segments);
        boundaryPoints[i] = {outerRadius * cosf(theta), outerRadius * sinf(theta)};
    }
    
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    b2BodyId groundId = b2CreateBody(worldId, &groundBodyDef);

    b2ChainDef chainDef = b2DefaultChainDef();
    chainDef.points = boundaryPoints.data();
    chainDef.count = segments;
    chainDef.isLoop = true;
    
    b2CreateChain(groundId, &chainDef);


    // 3. Create Dynamic Discs
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.linearDamping = 5.0f; // High damping to stop them from bouncing forever

    b2Circle circleShape = {0};
    circleShape.radius = discRadius;
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.restitution = 0.5f;

    std::vector<b2BodyId> discBodies(numDiscs);
    std::mt19937 rng(42); // Fixed seed
    std::uniform_real_distribution<float> distPos(-outerRadius/2.0f, outerRadius/2.0f);

    for(int i=0; i<numDiscs; ++i) {
        bodyDef.position = { distPos(rng), distPos(rng) };
        discBodies[i] = b2CreateBody(worldId, &bodyDef);
        b2CreateCircleShape(discBodies[i], &shapeDef, &circleShape);
    }

    // 4. Run Simulation
    // 120 steps should be enough to push them apart
    for(int i=0; i<120; ++i) {
        b2World_Step(worldId, 1.0f/60.0f, 4);
    }

    // -------------------------------------------------------------------------
    // Visual Mesh Generation
    // -------------------------------------------------------------------------

    // 1. Small Disc Mesh (Instanced)
    // Geometry
    SurfaceMesh discMesh;
    float containerThickness = 0.1f; // Shared thickness
    CgalMeshBuilder::buildThickDisc(discMesh, discRadius, containerThickness, 24);
    CgalMeshBuilder::triangulateAll(discMesh);
    std::vector<Vertex> discVertices; std::vector<uint32_t> discIndices;
    CgalMeshBuilder::toVertexIndexFlat(discMesh, discVertices, discIndices);

    // Instances
    std::vector<rtx::InstanceData> discInstances;
    std::uniform_int_distribution<int> distColor(0, 4); // 5 colors: 0..4

    for(b2BodyId bid : discBodies) {
        b2Vec2 pos = b2Body_GetPosition(bid);
        
        glm::vec3 pos3d(pos.x, 0.0f, pos.y); 
        
        b2Rot rot = b2Body_GetRotation(bid);
        float angleRad = b2Rot_GetAngle(rot);

        glm::mat4 M = glm::translate(glm::mat4(1.0f), pos3d);
        M = glm::rotate(M, -angleRad, glm::vec3(0,1,0)); // Rotation around Y axis

        // Assign random color ID (0-4)
        // uint32_t colId = static_cast<uint32_t>(distColor(rng));
        uint32_t colId = 2; // Force Blue for debugging

        discInstances.push_back({M, 0, colId}); 
    }


    // 2. Large Container Ring (Static Mesh)
    SurfaceMesh containerMesh;
    // ... (Container generation skipped for debug) ...
    // ...
    CgalMeshBuilder::buildThickDisc(containerMesh, outerRadius*1.05f, containerThickness, 64);
    CgalMeshBuilder::triangulateAll(containerMesh);
    std::vector<Vertex> containerVertices; std::vector<uint32_t> containerIndices;
    CgalMeshBuilder::toVertexIndexFlat(containerMesh, containerVertices, containerIndices);
    
    // Static instance for container
    std::vector<rtx::InstanceData> containerInstances;
    // DEBUG: Don't add instance
    // glm::mat4 containerM = glm::translate(glm::mat4(1.0f), glm::vec3(0, -containerThickness, 0)); // Exactly below
    // containerInstances.push_back({containerM});

    // Cleanup Box2D
    b2DestroyWorld(worldId);

    // -------------------------------------------------------------------------
    // Upload to RTX
    // -------------------------------------------------------------------------
    rtxModule->LoadFromMultipleMeshes({
        { discVertices,      discIndices,      discInstances },      // ID 0: Small Discs
        // { containerVertices, containerIndices, containerInstances }  // ID 1: Container REMOVED
    });

    if (meshRenderer) {
        // Debug lines if needed
    }
}
