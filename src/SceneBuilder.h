#pragma once

#include <vector>
#include <memory> 
#include "HelpStructures.h" 

// Forward declarations
namespace rtx {
    class RayTracingModule;
}
class StandardMeshRenderer;

class SceneBuilder {
public:
    // Builds the default scene (spheres, cubes, etc.) and loads it into the RTX module
    static void BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer, const SolverParameters& params);
};
