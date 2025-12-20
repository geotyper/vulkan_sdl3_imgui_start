#include "SceneBuilder.h"

#include "RayTracingModule.h"
#include "StandardMeshRenderer.h"
#include "GeomCreate.h"
#include "CgalMeshBuilder.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

void SceneBuilder::BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer) {
    if (!rtxModule) return;

    // 1) Geometry Generation
    // Sphere
    std::vector<Vertex> sphereVertices; 
    std::vector<uint32_t> sphereIndices;
    GeomCreate::createIcosphere(4, sphereVertices, sphereIndices);

    // Cube (using CGAL builder for halfedge structure, then flattening)
    SurfaceMesh sm;
    // CgalMeshBuilder::buildHollowCuboid(sm, 2, 2, 2, 1.0f);
    CgalMeshBuilder::buildThickDisc(sm, 1.5, 0.4, 32);
    
    // Optional: Add visual debugging arrows (halfedges)
    std::vector<Vertex> heLines;
    CgalMeshBuilder::buildHalfedgeArrows(sm, heLines, /*inset*/0.017f, /*head*/0.08f, true);

    // Triangulate the polygon mesh
    CgalMeshBuilder::triangulateAll(sm);

    // Convert CGAL mesh to flat buffers
    std::vector<Vertex>   cubeVertices;
    std::vector<uint32_t> cubeIndices;
    CgalMeshBuilder::toVertexIndexFlat(sm, cubeVertices, cubeIndices);


    // 2) Instances Generation (one list per mesh)
    std::vector<rtx::InstanceData> sphereInstances;
    std::vector<rtx::InstanceData> cubeInstances;

    // Base scales
    const float baseSphereScale = 0.30f;
    const float baseCubeScale   = 0.3f;
    const float specialScale    = 0.10f;

    // Setup base transform
    glm::vec3 pos = { 0.0f, 0.0f, 0.0f };
    glm::mat4 M   = glm::translate(glm::mat4(1.f), pos);
    
    // Add one cube instance
    cubeInstances.push_back({ glm::scale(M, glm::vec3(baseCubeScale)) });
    
    // Note: The original code had commented out sphereInstances.push_back(...)
    // If you want spheres, you would add them here.
    
    // Example logic from original code for modifying instances based on ID
    // Since we only have 1 cube and 0 spheres active in the original snippet, 
    // loops below might not do much, but preserving logic structure.
    
    const uint32_t numSpheres = static_cast<uint32_t>(sphereInstances.size());

    // a) Spheres: uniqueID == sphereIndex
    for (uint32_t si = 0; si < numSpheres; ++si) {
        if (si != 0 && (si % 27u) == 0u) {
            const float k = specialScale / baseSphereScale;
            // sphereInstances[si].transform *= glm::scale(glm::mat4(1.f), glm::vec3(k));
        }
    }

    // b) Cubes: uniqueID == numSpheres + cubeIndex
    for (uint32_t ci = 0; ci < cubeInstances.size(); ++ci) {
        const uint32_t uid = numSpheres + ci;
        if ((uid % 27u) == 0u) {
            const float k = specialScale / baseCubeScale;
            // cubeInstances[ci].transform *= glm::scale(glm::mat4(1.f), glm::vec3(k));
        }
    }

    // 3) Upload to RayTracingModule
    // meshId 0 = sphere, meshId 1 = cube
    rtxModule->LoadFromMultipleMeshes({
        { sphereVertices, sphereIndices, sphereInstances }, // meshId 0
        { cubeVertices,   cubeIndices,   cubeInstances   }  // meshId 1
    });

    // 4) Setup Rasterizer Debug Renderer if available
    if (meshRenderer) {
        // meshRenderer->SetMesh(cubeVertices, cubeIndices); 
        // meshRenderer->SetLines(dbgLines, {1,0,0});
        meshRenderer->SetColoredLines(heLines);
    }
}
