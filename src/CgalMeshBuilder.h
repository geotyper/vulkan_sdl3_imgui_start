#pragma once
#include <vector>
#include <unordered_set>
#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

using Kernel      = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3     = Kernel::Point_3;
using Vector_3    = Kernel::Vector_3;
using SurfaceMesh = CGAL::Surface_mesh<Point_3>;

// Your engine's Vertex (pos, normal, color)
#include "GeomCreate.h"   // must define: struct Vertex { glm::vec4 position, normal, color; }

struct MeshData { std::vector<float> vertices; std::vector<uint32_t> indices; };

struct ExtrudeParams {
    double inset_scale   = 0.5;   // 0..1 (0 = no inset; 0.5 = half-size ring)
    double distance      = 0.2;   // + outward along face normal, − inward
    bool   keep_base     = true;  // keep original base polygon face
    bool   remove_base   = false; // if true, remove the face before building the frame
    bool   add_outer_wall= false; // vertical wall along the outer boundary (useful on shells)
};

class CgalMeshBuilder {
public:
    // ---- Stage 1: create a polygonal mesh (n-gons allowed) ----
    static void buildCube(SurfaceMesh& sm, double size = 1.0);

    // ---- Stage 2: select faces ----
    static std::vector<SurfaceMesh::Face_index>
    selectFacesRandom(const SurfaceMesh& sm, double probability01, uint32_t seed = 42);

    // ---- Stage 3A: delete faces ----
    static void deleteFaces(SurfaceMesh& sm,
                            const std::vector<SurfaceMesh::Face_index>& faces,bool split_kissing_vertices /* = true */);


    // ---- Stage 3B: extrude faces (with optional inset) ----
    static void extrudeFaces(SurfaceMesh& sm,
                             const std::vector<SurfaceMesh::Face_index>& faces,
                             const ExtrudeParams& p);

    // ---- Stage 4: triangulation ----
    static void triangulateAll(SurfaceMesh& sm);
    static void triangulateFaces(SurfaceMesh& sm,
                                 const std::vector<SurfaceMesh::Face_index>& faces);

    // ---- Stage 5: export for RT (flat normals) ----
    static void toVertexIndexFlat(const SurfaceMesh& sm,
                                  std::vector<Vertex>& outV,
                                  std::vector<uint32_t>& outI,
                                  glm::vec4 color = glm::vec4(1,1,1,1));

    static void subdivideQuadFacesGrid(SurfaceMesh& sm, int nx, int ny);
    static  void buildCubeWithGrid(SurfaceMesh &sm, double size, int nx, int ny);
    static void applyCatmullClark(SurfaceMesh &sm, int iterations = 1, bool keep_borders = true);
    static void circularizeBorderLoops(SurfaceMesh &sm, double scale);

    static void filletBorderLoops(SurfaceMesh& sm,
                                 int rings,          // how many edge-rings to affect (3..8)
                                 double height,      // max offset at the rim
                                 bool outward=true,  // direction
                                 double sharpness=1.0);
    static void catmullClarkRefineNoSmooth(SurfaceMesh &sm);
    static void catmullClarkRefine_NoInterp(SurfaceMesh &sm, bool keep_borders);

    static void buildHalfedgeArrows(
        const SurfaceMesh& sm,
        std::vector<Vertex>& outLineVerts,
        float inset = 0.02f,
        float headRel = 0.08f);

    static void buildHollowCuboid(SurfaceMesh& sm, int N, int M, int L, double cellSize);

    static void buildBoxGrid(SurfaceMesh& sm,
                             int nx, int ny, int nz,
                             double cellSize);
    void splitJunctionVertices(SurfaceMesh &sm);
};
