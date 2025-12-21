#include "SceneBuilder.h"

#include "RayTracingModule.h"
#include "StandardMeshRenderer.h"
#include "GeomCreate.h"
#include "CgalMeshBuilder.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <random>

#include <box2d/box2d.h>

void SceneBuilder::BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer, const SolverParameters& params) {
    if (!rtxModule) return;

    // -------------------------------------------------------------------------
    // Box2D Simulation Setup for Packing
    // -------------------------------------------------------------------------

    // Parameters
    const float outerRadius = 4.0f;
    const int   numDiscs    = params.numDiscs;
    const int   numLayers   = 2;     // Two layers
    float containerThickness = 0.1f;

    std::random_device rd;
    std::mt19937 rng(rd()); 

    std::uniform_real_distribution<float> distPos(-outerRadius/2.0f, outerRadius/2.0f);
    std::uniform_int_distribution<int> distColor(0, 6); // 7 colors

    struct SimResult {
        glm::mat4 transform;
        uint32_t colorID;
    };
    std::vector<SimResult> simulationResults;

    for(int layer=0; layer < numLayers; ++layer) {
        // --- 1. Setup World ---
        b2WorldDef worldDef = b2DefaultWorldDef();
        worldDef.gravity = {0.0f, 0.0f}; 
        b2WorldId worldId = b2CreateWorld(&worldDef);

        // --- 2. Create Static Outer Boundary ---
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

        // --- 3. Create Dynamic Discs ---
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.linearDamping = 5.0f; 

        b2Circle circleShape = {0};
        circleShape.radius = params.discRadius;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = 1.0f;
        shapeDef.restitution = 0.5f;

        std::vector<b2BodyId> discBodies(numDiscs);
        for(int i=0; i<numDiscs; ++i) {
            bodyDef.position = { distPos(rng), distPos(rng) };
            discBodies[i] = b2CreateBody(worldId, &bodyDef);
            b2CreateCircleShape(discBodies[i], &shapeDef, &circleShape);
        }

        // --- 4. Run Simulation ---
        for(int i=0; i<120; ++i) {
            b2World_Step(worldId, 1.0f/60.0f, 4);
        }

        // --- 5. Extract Instances ---
        float yOffset = layer * containerThickness; // Stack layers

        for(b2BodyId bid : discBodies) {
            b2Vec2 pos = b2Body_GetPosition(bid);
            
            // Map Box2D (X, Y) -> 3D (X, yOffset, Z=Y)
            glm::vec3 pos3d(pos.x, yOffset, pos.y); 
            
            b2Rot rot = b2Body_GetRotation(bid);
            float angleRad = b2Rot_GetAngle(rot);

            glm::mat4 M = glm::translate(glm::mat4(1.0f), pos3d);
            M = glm::rotate(M, -angleRad, glm::vec3(0,1,0)); 

            // Assign random color ID (0-6)
            uint32_t colId = static_cast<uint32_t>(distColor(rng));
            
            simulationResults.push_back({M, colId});
        }

        b2DestroyWorld(worldId);
    }

    std::vector<rtx::MeshLoadData> allMeshes;

    if (params.shapeType == 0) {
        // --- DISC MODE ---
        SurfaceMesh discMesh;
        CgalMeshBuilder::buildThickDisc(discMesh, params.discRadius, containerThickness, 24);
        CgalMeshBuilder::triangulateAll(discMesh);
        std::vector<Vertex> discVertices; std::vector<uint32_t> discIndices;
        CgalMeshBuilder::toVertexIndexFlat(discMesh, discVertices, discIndices);

        std::vector<rtx::InstanceData> instances;
        instances.reserve(simulationResults.size());
        for (const auto& res : simulationResults) {
            instances.push_back({res.transform, 0, res.colorID}); // meshId 0 will be assigned by loader relative to this batch, but actually loader assigns global ID?
            // Actually LoadFromMultipleMeshes assigns meshID based on loop index.
            // Wait, rtx::InstanceData definition has meshID? yes.
            // The loader normally overrides it or we set it?
            // In RayTracingModule::LoadFromMultipleMeshes:
            // for (int i=0; i<data.size(); i++) { ... uint32_t meshId = m_scene->meshes.size() - 1; ... m_instances.push_back({..., meshId, ...}) }
            // So the loader sets the meshId. We can pass 0 here.
        }
        
        allMeshes.push_back({discVertices, discIndices, instances});

    } else {
        // --- POLY MODE ---
        std::uniform_int_distribution<int> distSides(5, 7);

        for (const auto& res : simulationResults) {
             SurfaceMesh polyMesh;
             int sides = distSides(rng);
             // Variation 1.5
             CgalMeshBuilder::buildThickPolygon(polyMesh, params.discRadius, 1.5, containerThickness, sides, rng());
             CgalMeshBuilder::triangulateAll(polyMesh);
             
             std::vector<Vertex> v; std::vector<uint32_t> i;
             CgalMeshBuilder::toVertexIndexFlat(polyMesh, v, i);
             
             // Create 1 instance for this unique mesh
             std::vector<rtx::InstanceData> oneInst;
             oneInst.push_back({res.transform, 0, res.colorID});
             
             allMeshes.push_back({v, i, oneInst});
        }
    }

    // --- Kaleidoscope Mirrors ---
    if (params.useKaleidoscope) {
        SurfaceMesh mirrorMesh;
        // Build 3 separate overlapping quads to prevent light leaks at corners
        float r = 3.5f; 
        float h = 10.0f;
        float overlap = 0.5f; // Extend each mirror by 0.5 units at each end

        for(int i=0; i<3; ++i) {
            // Ideal corners for this segment
            float ang1 = 2.0f * M_PI * i / 3.0f + M_PI / 6.0f; 
            float ang2 = 2.0f * M_PI * ((i+1)%3) / 3.0f + M_PI / 6.0f;

            float x1 = r * cos(ang1);
            float z1 = r * sin(ang1);
            float x2 = r * cos(ang2);
            float z2 = r * sin(ang2);

            // Compute direction vector for this side
            float dx = x2 - x1;
            float dz = z2 - z1;
            float len = sqrt(dx*dx + dz*dz);
            dx /= len; dz /= len;

            // Extended vertices
            float ex1 = x1 - dx * overlap;
            float ez1 = z1 - dz * overlap;
            float ex2 = x2 + dx * overlap;
            float ez2 = z2 + dz * overlap;

            // Create 4 vertices for this isolated mirror panel
            // Vertices MUST use unique indices for each panel so they don't share edges!
            auto v0 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex1, 0.2f, ez1));
            auto v1 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex2, 0.2f, ez2));
            auto v2 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex2, 0.2f+h, ez2));
            auto v3 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex1, 0.2f+h, ez1));

            // Face Inward: v1 -> v0 -> v3 -> v2
            mirrorMesh.add_face(v1, v0, v3, v2);
        }

        CgalMeshBuilder::triangulateAll(mirrorMesh);
        std::vector<Vertex> mv; std::vector<uint32_t> mi;
        CgalMeshBuilder::toVertexIndexFlat(mirrorMesh, mv, mi);
        
        // Add as a separate mesh instance with ColorID 99 (Mirror)
        std::vector<rtx::InstanceData> mInst;
        mInst.push_back({glm::mat4(1.0f), 0, 99}); // MeshID 0 (relative), ColorID 99
        
        allMeshes.push_back({mv, mi, mInst});
    }

    rtxModule->LoadFromMultipleMeshes(allMeshes);
}
