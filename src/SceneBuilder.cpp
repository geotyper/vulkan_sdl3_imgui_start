#include "SceneBuilder.h"

#include "RayTracingModule.h"
#include "StandardMeshRenderer.h"
#include "GeomCreate.h"
#include "CgalMeshBuilder.h"
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <random>

// Box2D 3.0 math helpers
static glm::vec2 toGlm(b2Vec2 v) { return glm::vec2(v.x, v.y); }
static b2Vec2 toB2(glm::vec2 v) { return {v.x, v.y}; }

SceneBuilder::SceneBuilder() {
}

SceneBuilder::~SceneBuilder() {
    CleanupPhysics();
}

void SceneBuilder::CleanupPhysics() {
    if (b2World_IsValid(m_worldId)) {
        b2DestroyWorld(m_worldId);
        m_worldId = b2_nullWorldId;
    }
    m_discBodies.clear();
    m_bodyInfos.clear();
    m_boundaryBody = b2_nullBodyId;
    m_physicsInitialized = false;
}

void SceneBuilder::BuildScene(rtx::RayTracingModule* rtxModule, StandardMeshRenderer* meshRenderer, const SolverParameters& params) {
    if (!rtxModule) return;

    CleanupPhysics();

    // -------------------------------------------------------------------------
    // Box2D Simulation Setup
    // -------------------------------------------------------------------------

    // Parameters
    const float baseInradius = 4.0f;
    const float outerRadius = baseInradius / cosf(3.14159f / (float)params.boundarySides);

    const int   numDiscs    = params.numDiscs;
    const int   numLayers   = params.numLayers;
    float containerThickness = 0.1f;

    std::random_device rd;
    std::mt19937 rng(rd()); 

    std::uniform_real_distribution<float> distPos(-outerRadius/2.0f, outerRadius/2.0f);
    std::uniform_int_distribution<int> distColor(0, 6); // 7 colors

    struct SimResult {
        glm::mat4 transform;
        uint32_t colorID;
    };
    std::vector<SimResult> simulationResults; // ONLY used for static mode

    // We might have multiple worlds if layers are independent, 
    // BUT for animation it is better to have ONE world.
    // However, the original code had independent layers stack on Y.
    // Box2D is 2D. To animate multiple layers, we could:
    // 1. Create one world, use collision filtering so layers don't interact?
    // 2. Or just one world where they all interact in 2D but are rendered at different Z (which would look weird if they overlap).
    // The previous code created a NEW world for EACH layer, simulated, then destroyed it.
    // If we want to animate ALL layers, we need ONE world with filtering.
    // Or just run one simulation and duplicate the motion for all layers? 
    // Or hold vector<b2WorldId>.
    
    // For simplicity: Let's use ONE world.
    // Layers will be simulated effectively as "clones" if we use one world?
    // User requests "Figures will be in hexagon".
    // Let's create `numLayers` independent worlds? Box2D is cheap.
    // But `SceneBuilder` only stored one `m_worldId`.
    // Let's just simulate ONE layer in physics, and render it `numLayers` times at different heights.
    // This looks visually synchronized (kaleidoscope-like).
    // If we want randomness per layer, we need more descriptors.
    // Let's stick to **Simulating ONE world**, and rendering it multiple times with offsets.

    // --- 1. Setup World ---
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = {0.0f, 10.0f}; // Gravity Downwards
    m_worldId = b2CreateWorld(&worldDef);


    m_physicsInitialized = true;

    // --- 2. Create Boundary ---
    b2BodyDef groundBodyDef = b2DefaultBodyDef();
    if (params.animate) {
        groundBodyDef.type = b2_kinematicBody;
        groundBodyDef.gravityScale = 0.0f; // Ensure it doesn't fall
        groundBodyDef.angularVelocity = params.rotationSpeed; 
    } else {
        groundBodyDef.type = b2_staticBody;
    }
    m_boundaryBody = b2CreateBody(m_worldId, &groundBodyDef);

    std::vector<b2Vec2> boundaryPoints;
    int segments = params.boundarySides; 
    for(int i=0; i<segments; ++i) {
        float theta = 2.0f * 3.14159f * float(i) / float(segments);
        boundaryPoints.push_back({outerRadius * cosf(theta), outerRadius * sinf(theta)});
    }
    
    // --- Thick Wall Boundary (6 Polygons) ---
    // More robust than Chain for "Tumbler" mechanics.
    b2ShapeDef wallShapeDef = b2DefaultShapeDef();
    wallShapeDef.density = 1.0f; 
    wallShapeDef.friction = 0.5f; 
    wallShapeDef.restitution = 0.1f;
    wallShapeDef.filter.categoryBits = 0x1;
    wallShapeDef.filter.maskBits = 0xFFFFFFFF;

    float wallThickness = 1.0f;
    // use existing 'segments' int or cast
    float angleStep = 2.0f * 3.14159f / (float)segments;
    
    for(int i=0; i<segments; ++i) {
        float theta = angleStep * (i + 0.5f); // Midpoint angle
        
        // Correct distance to match Visual Hexagon Flat Side
        float flatDist = outerRadius * cosf(3.14159f / (float)segments);
        
        // Push physics box out by half thickness so inner face aligns with flatDist
        float dist = flatDist + wallThickness * 0.5f;
        
        b2Vec2 center = { dist * cosf(theta), dist * sinf(theta) };
        
        float angle = theta + 3.14159f / 2.0f;
        
        // Exact half-length of a side: outerRadius * sin(pi/n)
        float hx = outerRadius * sinf(3.14159f / (float)segments) + 0.2f; // Overlap for corner closure
        float hy = wallThickness * 0.5f;
        
        // Use angle directly (Box2D 3.0 API variant)
        b2Polygon wallPoly = b2MakeOffsetBox(hx, hy, center, angle);
        b2CreatePolygonShape(m_boundaryBody, &wallShapeDef, &wallPoly);
    }

    // --- 3. Create Independent Bodies for EACH Layer ---
    b2BodyDef bodyDef = b2DefaultBodyDef();
    bodyDef.type = b2_dynamicBody;
    bodyDef.isBullet = false; // User requested OFF
    if (params.animate) {
        bodyDef.linearDamping = 8.0f;
        bodyDef.angularDamping = 2.0f;
    } else {
        bodyDef.linearDamping = 5.0f;
    }

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.restitution = 0.2f;
    shapeDef.friction = 0.0f;

    b2Circle circleShape = {0};
    circleShape.radius = params.discRadius;

    m_discBodies.clear(); // Clear old list (although CleanupPhysics likely did)
    m_bodyInfos.clear();
    
    // --- 4. Prepare Meshes (Unique per instance) ---
    std::vector<rtx::MeshLoadData> allMeshes;

    std::uniform_int_distribution<int> distSides(5, 7);
    int meshCounter = 0;

    // Independent Bodies Loop
    for(int layer=0; layer < numLayers; ++layer) {
        
        // Define Collision Mask for this Layer
        // Categories: Boundary=0x1, Layer0=0x2, Layer1=0x4, etc.
        uint32_t layerCategory = (1u << (layer + 1));
        uint32_t boundaryCategory = 0x1;
        
        shapeDef.filter.categoryBits = layerCategory;
        shapeDef.filter.maskBits = layerCategory | boundaryCategory; // Self + Boundary

        for(int i=0; i<numDiscs; ++i) {
            // Random Independent Position
            bodyDef.position = { distPos(rng), distPos(rng) };
            b2BodyId bid = b2CreateBody(m_worldId, &bodyDef);
            // Shape creation moved inside loop based on type
            
            // Note: We don't store into m_discBodies flat vector anymore as primary source,
            // but we can if we want to clean them up easily. 
            // CleanupPhysics iterates ALL bodies in world? No, uses ID list? 
            // CleanupPhysics calls b2DestroyWorld which destroys all.
            // m_discBodies is just a helper. Let's populate it.
            m_discBodies.push_back(bid);

            uint32_t cId = distColor(rng);

            // --- Mesh Generation ---
            // Disc Mode: Re-use Mesh 0? 
            // Poly Mode: Unique per stone.
            
            uint32_t assignedMeshID = 0;
            
            if (params.shapeType == 0) {
                 // Disc Mode: Mesh 0 is the single shared cylinder
                 // We add mapping info
                 assignedMeshID = 0;
                 // We will generate Mesh 0 just ONCE at the end or begin.
                 // Body matches visual (Circle/Cylinder)
                 b2CreateCircleShape(bid, &shapeDef, &circleShape);
                 
            } else {            
                 // Poly Mode: Unique Mesh AND Unique Physics Shape
                 int sides = distSides(rng);
                 uint32_t layerSeed = (uint32_t)rng() + (uint32_t)(layer * 123456) + (uint32_t)(i * 789);
                 
                 // 1. Generate Polygon Points using same logic as buildThickPolygon (but explicitly here)
                 std::mt19937 polyRng(layerSeed);
                 std::uniform_real_distribution<float> distAngle(0.0f, 6.28318f); // Start angle
                 std::uniform_real_distribution<float> distVar(0.8f, 1.2f); // Radius variation

                 std::vector<b2Vec2> b2Points;
                 std::vector<glm::vec2> glmPoints;
                 float startAngle = distAngle(polyRng);
                 
                 for(int k=0; k<sides; ++k) {
                     float t = startAngle + 2.0f * 3.14159f * k / sides;
                     float r = params.discRadius * distVar(polyRng);
                     
                     // Box2D Points
                     b2Points.push_back({r * cosf(t), r * sinf(t)});
                     // GLM Points (same)
                     glmPoints.push_back({r * cosf(t), r * sinf(t)});
                 }
                 
                 
                 // 2. Create Physics Shape (Convex Hull)
                 b2Hull hull = b2ComputeHull(b2Points.data(), (int)b2Points.size());
                 
                 // Basic validation
                 if (hull.count > 0) {
                     b2Polygon polyShape = b2MakePolygon(&hull, 0.0f);
                     b2CreatePolygonShape(bid, &shapeDef, &polyShape);
                 } else {
                     // Fallback
                     b2Circle circle; circle.center = {0,0}; circle.radius = params.discRadius;
                     b2CreateCircleShape(bid, &shapeDef, &circle);
                 }

                 // 3. Generate Visual Mesh using SAME points
                 bool isTop = (layer == numLayers - 1);
                 bool makePyramid = isTop && params.makePyramid; 

                 SurfaceMesh polyMesh;
                 // Use the new helper function
                 CgalMeshBuilder::buildThickPolygonFromPoints(polyMesh, glmPoints, containerThickness * 1.5, makePyramid, 0.3);
                 
                 if (params.subdivisionIterations > 0) {
                     CgalMeshBuilder::applyCatmullClark(polyMesh, params.subdivisionIterations, true);
                 }

                 CgalMeshBuilder::triangulateAll(polyMesh);
                 
                 std::vector<Vertex> v; std::vector<uint32_t> ind;
                 CgalMeshBuilder::toVertexIndexFlat(polyMesh, v, ind);
                 
                 // Create Load Data for this specific mesh
                 std::vector<rtx::InstanceData> dummyInst; 
                 // Initial transform instance
                 b2Vec2 pos = bodyDef.position; 
                 float yOffset = layer * containerThickness;
                 glm::mat4 M = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, yOffset, pos.y));
                 
                 dummyInst.push_back({M, 0, cId});
                 allMeshes.push_back({v, ind, dummyInst});
                 
                 assignedMeshID = meshCounter;
                 meshCounter++; // Unique ID per poly
            }
            
            m_bodyInfos.push_back({bid, layer, cId, assignedMeshID, toGlm(bodyDef.position)});
        }
    }
    
    // --- Shared Disc Mesh Generation (if Mode 0) ---
    if (params.shapeType == 0) {
        SurfaceMesh discMesh;
        CgalMeshBuilder::buildThickDisc(discMesh, params.discRadius, containerThickness, 24);

        if (params.subdivisionIterations > 0) {
            CgalMeshBuilder::applyCatmullClark(discMesh, params.subdivisionIterations, true);
        }

        CgalMeshBuilder::triangulateAll(discMesh);
        std::vector<Vertex> v; std::vector<uint32_t> ind;
        CgalMeshBuilder::toVertexIndexFlat(discMesh, v, ind);
        
        // Find all instances using Mesh 0
        std::vector<rtx::InstanceData> instances;
        for(const auto& info : m_bodyInfos) {
             b2Vec2 pos = b2Body_GetPosition(info.bodyId);
             float yOffset = info.layer * containerThickness;
             glm::mat4 M = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, yOffset, pos.y));
             instances.push_back({M, 0, info.colorID});
        }
        allMeshes.insert(allMeshes.begin(), {v, ind, instances}); // Insert at 0
        
        // Shift all assignedMeshIDs by 1? No, we used 0. Correct.
        // Wait, if we mix modes, logic gets complex. 
        // But user selects "Shape" combo.
    }
    // --- 5. Generate Boundary Mesh ---
    // We visualize the kinematic boundary so user sees the "Tumbler"
    {
        SurfaceMesh boundaryMesh;
        // Height covers all layers + safety
        double height = numLayers * containerThickness * 3.0; // Taller to prevent spill
        double thickness = 0.5; // Thick wall
        int segments = params.boundarySides;
        
        CgalMeshBuilder::buildHollowHexagon(boundaryMesh, outerRadius, thickness, height, segments);
        CgalMeshBuilder::triangulateAll(boundaryMesh);

        std::vector<Vertex> v; std::vector<uint32_t> ind;
        CgalMeshBuilder::toVertexIndexFlat(boundaryMesh, v, ind);
        
        std::vector<rtx::InstanceData> dummyInst;
        // Initial transform
        glm::mat4 M = glm::mat4(1.0f); 
        // Color ID 7 (White/Black)
        dummyInst.push_back({M, 0, 7}); 
        
        allMeshes.push_back({v, ind, dummyInst});
        
        // Add to m_bodyInfos so it gets updated
        // Use layer = 0, colorID = 7
        // We use meshID = size-1
        m_bodyInfos.push_back({m_boundaryBody, 0, 7, (uint32_t)allMeshes.size()-1, glm::vec2(0,0)});
    }
    // --- Kaleidoscope Mirrors (Static) ---
    if (params.useKaleidoscope) {
        SurfaceMesh mirrorMesh;
        float r = 3.5f; 
        float h = params.mirrorHeight;
        float overlap = 0.5f; 

        for(int i=0; i<3; ++i) {
            float ang1 = 2.0f * M_PI * i / 3.0f + M_PI / 6.0f; 
            float ang2 = 2.0f * M_PI * ((i+1)%3) / 3.0f + M_PI / 6.0f;
            float x1 = r * cos(ang1); float z1 = r * sin(ang1);
            float x2 = r * cos(ang2); float z2 = r * sin(ang2);
            float dx = x2 - x1; float dz = z2 - z1;
            float len = sqrt(dx*dx + dz*dz); dx /= len; dz /= len;
            float ex1 = x1 - dx * overlap; float ez1 = z1 - dz * overlap;
            float ex2 = x2 + dx * overlap; float ez2 = z2 + dz * overlap;

            auto v0 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex1, 0.2f, ez1));
            auto v1 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex2, 0.2f, ez2));
            auto v2 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex2, 0.2f+h, ez2));
            auto v3 = mirrorMesh.add_vertex(CgalMeshBuilder::P3(ex1, 0.2f+h, ez1));
            mirrorMesh.add_face(v1, v0, v3, v2);
        }
        CgalMeshBuilder::triangulateAll(mirrorMesh);
        std::vector<Vertex> mv; std::vector<uint32_t> mi;
        CgalMeshBuilder::toVertexIndexFlat(mirrorMesh, mv, mi);
        
        std::vector<rtx::InstanceData> mInst;
        mInst.push_back({glm::mat4(1.0f), 0, 99}); 
        
        // The previous logic for LoadFromMultipleMeshes assigns meshIDs sequentially.
        // If we added disc mesh first (ID 0), this will be ID 1.
        allMeshes.push_back({mv, mi, mInst});
    }

    // Load geometry
    m_loadedMeshCount = (uint32_t)allMeshes.size();
    rtxModule->LoadFromMultipleMeshes(allMeshes);

    if (!params.animate) {
        // If not animating, we don't need the world anymore
        CleanupPhysics();
    }
}


void SceneBuilder::RestartSimulation() {
    if (!b2World_IsValid(m_worldId)) return;
    
    // Reset all bodies to initial positions
    for (const auto& info : m_bodyInfos) {
        if (b2Body_IsValid(info.bodyId)) {
            b2Body_SetTransform(info.bodyId, toB2(info.initialPos), b2Rot_identity);
            b2Body_SetLinearVelocity(info.bodyId, {0.0f, 0.0f});
            b2Body_SetAngularVelocity(info.bodyId, 0.0f);
            b2Body_SetAwake(info.bodyId, true);
        }
    }
    m_accumTime = 0.0f;
}

void SceneBuilder::UpdatePhysics(float dt, rtx::RayTracingModule* rtxModule, const SolverParameters& params) {
    if (!m_physicsInitialized || !rtxModule || !params.animate) return;
    if (!b2World_IsValid(m_worldId)) return;

    // Handle Restart Request
    if (params.requestRestart) {
        // We need to cast away constness or make RestartSimulation const but it modifies state.
        // Or SceneBuilder method handles it.
        // Actually, SceneBuilder is non-const pointer in GraphicsModule, so we can modify it.
        // But here 'params' is const ref. 
        // We called RestartSimulation() on 'this'. That's fine.
        const_cast<SceneBuilder*>(this)->RestartSimulation(); 
    }

    // Handle Pause or Manual Step
    if (!params.paused || params.triggerStep) {
        // 1. Step Physics
        if (params.triggerStep) {
            // Manual Single Step
            b2World_Step(m_worldId, 1.0f / 60.0f, 12);
        } else {
            // Continuous Run
            m_accumTime += dt;
        const float stepSize = 1.0f / 60.0f;
        while (m_accumTime >= stepSize) {
            b2World_Step(m_worldId, stepSize, 12); // Increased iterations for stability
            m_accumTime -= stepSize;
            
            // 2. Animate Boundary (Rotate Shape)
            // Kinematic boundary rotates automatically due to angularVelocity
            if (b2Body_IsValid(m_boundaryBody)) {
                b2Body_SetAngularVelocity(m_boundaryBody, params.rotationSpeed);
            }
        }
    }
    }

    // 3. Update Instances
    std::vector<rtx::InstanceData> newInstances;
    newInstances.reserve(m_bodyInfos.size() + 1);

    // Discs
    float containerThickness = 0.1f;
    for (const auto& info : m_bodyInfos) {
        if (!b2Body_IsValid(info.bodyId)) continue;
        
        // Safety Check
        if (info.meshID >= m_loadedMeshCount) continue;
        
        b2Vec2 pos = b2Body_GetPosition(info.bodyId);
        b2Rot rot = b2Body_GetRotation(info.bodyId);
        float angle = b2Rot_GetAngle(rot);
        float yOffset = info.layer * containerThickness;

        glm::mat4 M = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, yOffset, pos.y));
        M = glm::rotate(M, -angle, glm::vec3(0,1,0));
        
        newInstances.push_back({M, info.meshID, info.colorID});
    }

    // Mirrors
    if (params.useKaleidoscope) {
         // Mirror is the last mesh added
         // We can infer its ID: it's equal to the number of dynamic meshes.
         // Or just: (total meshes loaded - 1). 
         
         uint32_t mirrorMeshID = 0;
         if (params.shapeType == 0) mirrorMeshID = 1; // Disc (0) + Mirror (1)
         else mirrorMeshID = (uint32_t)m_bodyInfos.size(); // N Polys + Mirror
         
         if (mirrorMeshID < m_loadedMeshCount) {
              newInstances.push_back({glm::mat4(1.0f), mirrorMeshID, 99});
         }
    }

    rtxModule->UpdateInstances(newInstances);
}
