#include "CgalMeshBuilder.h"

#include <array>
#include <random>
#include <algorithm>
#include <cmath>

#include <CGAL/centroid.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/boost/graph/generators.h>

#include <CGAL/Subdivision_method_3/subdivision_methods_3.h>
#include <unordered_set>
#include <CGAL/Polygon_mesh_processing/repair.h>

namespace S3 = CGAL::Subdivision_method_3;
namespace PMP = CGAL::Polygon_mesh_processing;
using SM = SurfaceMesh;

namespace {
inline void remove_isolated_vertices_safe(SurfaceMesh& sm) {
    // Если у тебя свежий CGAL, просто раскомментируй следующую строку:
    // PMP::remove_isolated_vertices(sm);
    // Универсальный fallback:
    std::vector<SurfaceMesh::Vertex_index> dead;
    for (auto v : sm.vertices())
        if (sm.halfedge(v) == SurfaceMesh::null_halfedge())
            dead.push_back(v);
    for (auto v : dead) sm.remove_vertex(v);
    sm.collect_garbage();
}

// collect face boundary vertices (CCW as stored)
static std::vector<SurfaceMesh::Vertex_index>
face_vertices_ring(const SurfaceMesh& sm, SurfaceMesh::Face_index f) {
    std::vector<SurfaceMesh::Vertex_index> ring;
    auto h = sm.halfedge(f);
    auto it = h;
    do {
        ring.push_back( target(it, sm) );
        it = next(it, sm);
    } while (it != h);
    return ring;
}

static Point_3 average_point(const SurfaceMesh& sm, const std::vector<SurfaceMesh::Vertex_index>& ring) {
    double x=0, y=0, z=0;
    for (auto v : ring) { auto p = sm.point(v); x+=p.x(); y+=p.y(); z+=p.z(); }
    const double inv = 1.0 / double(ring.size());
    return Point_3(x*inv, y*inv, z*inv);
}

// Newell's method for robust polygon normal calculation
static Vector_3 face_normal_unit(const SurfaceMesh& sm, const std::vector<SurfaceMesh::Vertex_index>& ring) {
    Vector_3 n(0, 0, 0);
    const std::size_t k = ring.size();
    if (k < 3) {
        return Vector_3(0, 0, 1); // Not a valid polygon
    }

    for (std::size_t i = 0; i < k; ++i) {
        const Point_3& current_p = sm.point(ring[i]);
        const Point_3& next_p = sm.point(ring[(i + 1) % k]);

        // This is the core formula of Newell's method
        n = n + Vector_3((current_p.y() - next_p.y()) * (current_p.z() + next_p.z()),
                         (current_p.z() - next_p.z()) * (current_p.x() + next_p.x()),
                         (current_p.x() - next_p.x()) * (current_p.y() + next_p.y()));
    }

    const double L = std::sqrt(n.squared_length());
    if (L > 1e-9) { // Use a small epsilon for floating point safety
        return n / L;
    } else {
        return Vector_3(0, 0, 1); // Default for degenerate polygons
    }
}

static Vector_3 normal_from_points(const std::vector<Point_3>& P) {
    if (P.size() < 3) return Vector_3(0,0,1);
    Vector_3 n(0,0,0);
    for (size_t i=0;i<P.size();++i) {
        const auto& a = P[i];
        const auto& b = P[(i+1)%P.size()];
        n = n + Vector_3((a.y()-b.y())*(a.z()+b.z()),
                         (a.z()-b.z())*(a.x()+b.x()),
                         (a.x()-b.x())*(a.y()+b.y()));
    }
    const double L = std::sqrt(n.squared_length());
    return (L>1e-12)? n/L : Vector_3(0,0,1);
}

inline Point_3 scale_about(const Point_3& p, const Point_3& c, double s) {
    return Point_3( c.x() + (p.x()-c.x())*s,
                   c.y() + (p.y()-c.y())*s,
                   c.z() + (p.z()-c.z())*s );
}

inline void add_polygon_face(SurfaceMesh& sm, const std::vector<SurfaceMesh::Vertex_index>& ring) {
    if (ring.size() < 3) return;
    CGAL::Euler::add_face(ring, sm);
}

inline void add_quad(SurfaceMesh& sm,
                     SurfaceMesh::Vertex_index a, SurfaceMesh::Vertex_index b,
                     SurfaceMesh::Vertex_index c, SurfaceMesh::Vertex_index d)
{
    std::array<SurfaceMesh::Vertex_index,4> quad{a,b,c,d};
    CGAL::Euler::add_face(quad, sm);
}

inline Point_3 lerpP(const Point_3& a, const Point_3& b, double t) {
    return Point_3(a.x() + (b.x()-a.x())*t,
                   a.y() + (b.y()-a.y())*t,
                   a.z() + (b.z()-a.z())*t);
}
inline Point_3 bilerpP(const Point_3& p00, const Point_3& p10,
                       const Point_3& p11, const Point_3& p01,
                       double u, double v) {
    const double w00=(1-u)*(1-v), w10=u*(1-v), w11=u*v, w01=(1-u)*v;
    return Point_3( p00.x()*w00 + p10.x()*w10 + p11.x()*w11 + p01.x()*w01,
                   p00.y()*w00 + p10.y()*w10 + p11.y()*w11 + p01.y()*w01,
                   p00.z()*w00 + p10.z()*w10 + p11.z()*w11 + p01.z()*w01 );
}

inline Point_3 bilerp(const Point_3& p00, const Point_3& p10,
                      const Point_3& p11, const Point_3& p01,
                      double u, double v)
{
    const double w00 = (1.0 - u) * (1.0 - v);
    const double w10 = u * (1.0 - v);
    const double w11 = u * v;
    const double w01 = (1.0 - u) * v;
    return Point_3(p00.x()*w00 + p10.x()*w10 + p11.x()*w11 + p01.x()*w01,
                   p00.y()*w00 + p10.y()*w10 + p11.y()*w11 + p01.y()*w01,
                   p00.z()*w00 + p10.z()*w10 + p11.z()*w11 + p01.z()*w01);
}


inline Point_3 midpoint(const Point_3& a, const Point_3& b) {
    return Point_3( (a.x()+b.x())*0.5, (a.y()+b.y())*0.5, (a.z()+b.z())*0.5 );
}
inline Point_3 centroid4(const Point_3& a, const Point_3& b,
                         const Point_3& c, const Point_3& d) {
    return Point_3( (a.x()+b.x()+c.x()+d.x())*0.25,
                   (a.y()+b.y()+c.y()+d.y())*0.25,
                   (a.z()+b.z()+c.z()+d.z())*0.25 );
}
inline void add_quad_ccw(SurfaceMesh& sm,
                         SurfaceMesh::Vertex_index a, SurfaceMesh::Vertex_index b,
                         SurfaceMesh::Vertex_index c, SurfaceMesh::Vertex_index d)
{
    std::array<SurfaceMesh::Vertex_index,4> q{a,b,c,d};
    CGAL::Euler::add_face(q, sm);
}

inline void add_quad_ccw(SurfaceMesh& sm,
                         SurfaceMesh::Vertex_index a, SurfaceMesh::Vertex_index b,
                         SurfaceMesh::Vertex_index c, SurfaceMesh::Vertex_index d,
                         const Vector_3& Nwant)
{
    const Point_3 pa = sm.point(a), pb = sm.point(b), pc = sm.point(c);
    const Vector_3 cr = CGAL::cross_product(pb - pa, pc - pa);
    const double   s  = cr.x()*Nwant.x() + cr.y()*Nwant.y() + cr.z()*Nwant.z();
    if (s >= 0.0) {
        std::array<SurfaceMesh::Vertex_index,4> q{a,b,c,d};
        CGAL::Euler::add_face(q, sm);
    } else {
        std::array<SurfaceMesh::Vertex_index,4> q{a,d,c,b};
        CGAL::Euler::add_face(q, sm);
    }
}

inline Point_3 move_along(const Point_3& p, const Vector_3& v) { return p + v; }

// --- add near the top (helpers) ---
static inline glm::vec3 to_glm_2(const Point_3& p) {
    return glm::vec3((float)p.x(), (float)p.y(), (float)p.z());
}

static Point_3 mesh_centroid(const SurfaceMesh& sm) {
    double x=0, y=0, z=0; std::size_t n=0;
    for (auto v : sm.vertices()) if (!sm.is_removed(v)) {
            auto p = sm.point(v); x+=p.x(); y+=p.y(); z+=p.z(); ++n;
        }
    if (n==0) return Point_3(0,0,0);
    const double inv = 1.0 / double(n);
    return Point_3(x*inv, y*inv, z*inv);
}

static inline glm::vec3 unit3_2(const glm::vec3& v) {
    float l2 = glm::dot(v,v); return (l2>0.f)? v/std::sqrt(l2) : glm::vec3(0,0,1);
}

// ===== UID helpers ==============================================================


inline SurfaceMesh::Halfedge_index halfedge_between(SurfaceMesh& sm,
                                                    SurfaceMesh::Vertex_index vi,
                                                    SurfaceMesh::Vertex_index vj)
{
    auto p = CGAL::halfedge(vi, vj, sm);      // std::pair<Halfedge_index,bool>
    return p.second ? p.first : SurfaceMesh::null_halfedge();
}

static std::pair<
    SurfaceMesh::Property_map<SurfaceMesh::Face_index, std::uint64_t>,
    std::uint64_t>
ensure_face_uid_map(SurfaceMesh& sm)
{
    using SM = SurfaceMesh;
    auto opt = sm.property_map<SM::Face_index, std::uint64_t>("f:uid");
    if (opt) {
        auto pm = *opt;
        std::uint64_t next = 1;
        for (auto f : sm.faces()) if (!sm.is_removed(f))
                next = std::max(next, pm[f]);
        return { pm, next + 1 };
    }
    auto created = sm.add_property_map<SM::Face_index, std::uint64_t>("f:uid", 0);
    auto pm = created.first;
    std::uint64_t next = 1;
    for (auto f : sm.faces()) if (!sm.is_removed(f)) pm[f] = next++;
    return { pm, next };
}

// ---- vertices: v:uid ----
static std::pair<
    SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, std::uint64_t>,
    std::uint64_t>
ensure_vertex_uid_map(SurfaceMesh& sm)
{
    using SM = SurfaceMesh;
    auto opt = sm.property_map<SM::Vertex_index, std::uint64_t>("v:uid");
    if (opt) {
        auto pm = *opt;
        std::uint64_t next = 1;
        for (auto v : sm.vertices()) if (!sm.is_removed(v))
                next = std::max(next, pm[v]);
        return { pm, next + 1 };
    }
    auto created = sm.add_property_map<SM::Vertex_index, std::uint64_t>("v:uid", 0);
    auto pm = created.first;
    std::uint64_t next = 1;
    for (auto v : sm.vertices()) if (!sm.is_removed(v)) pm[v] = next++;
    return { pm, next };
}

static inline std::uint64_t edge_key_u64(std::uint64_t a, std::uint64_t b) {
    if (a > b) std::swap(a,b);
    return (a << 32) ^ b;
}

// соберём ключи рёбер грани по v:uid
static std::vector<std::uint64_t>
face_edge_keys(SurfaceMesh& sm,
               SurfaceMesh::Face_index f,
               const SurfaceMesh::Property_map<SurfaceMesh::Vertex_index, std::uint64_t>& v_uid)
{
    std::vector<std::uint64_t> E;
    if (f == SurfaceMesh::null_face() || sm.is_removed(f)) return E;

    auto h0 = sm.halfedge(f);
    if (h0 == SurfaceMesh::null_halfedge()) return E;

    auto h = h0;
    do {
        auto vi = source(h, sm);
        auto vj = target(h, sm);
        E.push_back(edge_key_u64(v_uid[vi], v_uid[vj]));
        h = next(h, sm);
    } while (h != h0);

    return E;
}

static SurfaceMesh::Face_index
find_face_by_uid(const SurfaceMesh& sm,
                 const SurfaceMesh::Property_map<SurfaceMesh::Face_index, std::uint64_t>& f_uid,
                 std::uint64_t uid)
{
    for (auto f : sm.faces())
        if (!sm.is_removed(f) && f_uid[f] == uid)
            return f;
    return SurfaceMesh::null_face();
}

// single helper to avoid “ambiguous” overloads
static inline SM::Halfedge_index hedg(SM& sm, SM::Vertex_index a, SM::Vertex_index b)
{
    auto pr = CGAL::halfedge(a, b, sm);
    return pr.second ? pr.first : SM::null_halfedge();
}


} // namespace

void CgalMeshBuilder::buildCube(SurfaceMesh& sm, double size) {
    sm.clear();
    const double h = 0.5 * size;

    // 1. Создаем 8 вершин куба вручную
    SurfaceMesh::Vertex_index v0 = sm.add_vertex(Point_3(-h, -h, -h));
    SurfaceMesh::Vertex_index v1 = sm.add_vertex(Point_3( h, -h, -h));
    SurfaceMesh::Vertex_index v2 = sm.add_vertex(Point_3( h,  h, -h));
    SurfaceMesh::Vertex_index v3 = sm.add_vertex(Point_3(-h,  h, -h));
    SurfaceMesh::Vertex_index v4 = sm.add_vertex(Point_3(-h, -h,  h));
    SurfaceMesh::Vertex_index v5 = sm.add_vertex(Point_3( h, -h,  h));
    SurfaceMesh::Vertex_index v6 = sm.add_vertex(Point_3( h,  h,  h));
    SurfaceMesh::Vertex_index v7 = sm.add_vertex(Point_3(-h,  h,  h));

    // 2. Явно добавляем 6 четырехугольных граней
    // Важно соблюдать порядок против часовой стрелки (если смотреть снаружи)

    // Нижняя грань (-Y)
    sm.add_face(v0, v1, v5, v4);
    // Верхняя грань (+Y)
    sm.add_face(v3, v7, v6, v2);
    // Передняя грань (-Z)
    sm.add_face(v0, v3, v2, v1);
    // Задняя грань (+Z)
    sm.add_face(v4, v5, v6, v7);
    // Левая грань (-X)
    sm.add_face(v0, v4, v7, v3);
    // Правая грань (+X)
    sm.add_face(v1, v2, v6, v5);
}


// Add this new function to the end of your CgalMeshBuilder.cpp file
// Add this new function to the end of your CgalMeshBuilder.cpp file

void CgalMeshBuilder::buildHollowCuboid(SurfaceMesh& sm, int N, int M, int L, double cellSize)
{
    sm.clear();
    if (N <= 0 || M <= 0 || L <= 0 || cellSize <= 0.0) {
        return; // Return an empty mesh for invalid dimensions
    }

    // Calculate the total dimensions of the grid to center it at the origin
    const double totalWidth  = N * cellSize;
    const double totalHeight = M * cellSize;
    const double totalDepth  = L * cellSize;
    const Point_3 start_corner(-totalWidth / 2.0, -totalHeight / 2.0, -totalDepth / 2.0);

    // 1. Create a 3D grid of all vertices first, just like before.
    std::vector<std::vector<std::vector<SurfaceMesh::Vertex_index>>> vertex_grid(
        N + 1,
        std::vector<std::vector<SurfaceMesh::Vertex_index>>(
            M + 1,
            std::vector<SurfaceMesh::Vertex_index>(L + 1)
            )
        );

    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j <= M; ++j) {
            for (int k = 0; k <= L; ++k) {
                Point_3 p = start_corner + Vector_3(i * cellSize, j * cellSize, k * cellSize);
                vertex_grid[i][j][k] = sm.add_vertex(p);
            }
        }
    }

    // 2. Iterate through each cell and create ONLY the exterior faces.
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            for (int k = 0; k < L; ++k) {
                // Get the 8 corner vertices for the current cell (i, j, k)
                SurfaceMesh::Vertex_index v000 = vertex_grid[i][j][k];
                SurfaceMesh::Vertex_index v100 = vertex_grid[i + 1][j][k];
                SurfaceMesh::Vertex_index v110 = vertex_grid[i + 1][j + 1][k];
                SurfaceMesh::Vertex_index v010 = vertex_grid[i][j + 1][k];
                SurfaceMesh::Vertex_index v001 = vertex_grid[i][j][k + 1];
                SurfaceMesh::Vertex_index v101 = vertex_grid[i + 1][j][k + 1];
                SurfaceMesh::Vertex_index v111 = vertex_grid[i + 1][j + 1][k + 1];
                SurfaceMesh::Vertex_index v011 = vertex_grid[i][j + 1][k + 1];

                // Add faces only if they are on the outer shell of the N x M x L grid
                if (j == 0)     sm.add_face(v000, v100, v101, v001); // Bottom face of the entire grid
                if (j == M - 1) sm.add_face(v010, v011, v111, v110); // Top face
                if (k == 0)     sm.add_face(v000, v010, v110, v100); // Front face
                if (k == L - 1) sm.add_face(v001, v101, v111, v011); // Back face
                if (i == 0)     sm.add_face(v000, v001, v011, v010); // Left face
                if (i == N - 1) sm.add_face(v100, v110, v111, v101); // Right face
            }
        }
    }

    // Clean up any vertices that were created but are not part of any face
    // (this will remove all the interior vertices).
    remove_isolated_vertices_safe(sm);
}

void CgalMeshBuilder::buildPlaneXY(SurfaceMesh& sm, int N, int M, double cellSize)
{
    sm.clear();
    if (N <= 0 || M <= 0 || cellSize <= 0.0) return;

    const double totalW = N * cellSize;
    const double totalH = M * cellSize;

    // левый-нижний угол, чтобы центрировать
    const Point_3 start_corner(-totalW * 0.5, -totalH * 0.5, 0.0);

    // (N+1) x (M+1) вершины
    std::vector<std::vector<SurfaceMesh::Vertex_index>> V(N + 1,
                                                          std::vector<SurfaceMesh::Vertex_index>(M + 1));

    for (int i = 0; i <= N; ++i) {
        for (int j = 0; j <= M; ++j) {
            const Point_3 p = start_corner + Vector_3(i * cellSize, j * cellSize, 0.0);
            V[i][j] = sm.add_vertex(p);
        }
    }

    // N x M квадов; CCW при взгляде из +Z
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < M; ++j) {
            const auto v00 = V[i][j];
            const auto v10 = V[i+1][j];
            const auto v11 = V[i+1][j+1];
            const auto v01 = V[i][j+1];
            sm.add_face(v00, v10, v11, v01);
        }
    }

    // на всякий случай (не обязательно — все вершины используются)
    // remove_isolated_vertices_safe(sm);
}


// Строит плоскость в точке center с нормалью normal (необязательно нормированная).
// Размеры клетки cellSize, решётка N×M, CCW так, чтобы нормаль была ≈ normal.
void CgalMeshBuilder::buildPlaneOriented(
    SurfaceMesh& sm, int N, int M, double cellSize,
    const Point_3& center, Vector_3 normal)
{
    sm.clear();
    if (N <= 0 || M <= 0 || cellSize <= 0.0) return;

    // Нормируем нормаль
    const double len = std::sqrt(normal.squared_length());
    if (len < 1e-12) normal = Vector_3(0,0,1);
    else normal = normal / len;

    // Построим ортонормированный базис (u,v,normal)
    Vector_3 helper = (std::fabs(normal.x()) < 0.9) ? Vector_3(1,0,0) : Vector_3(0,1,0);
    Vector_3 u = CGAL::cross_product(helper, normal);
    double lu = std::sqrt(u.squared_length());
    if (lu < 1e-12) { helper = Vector_3(0,0,1); u = CGAL::cross_product(helper, normal); lu = std::sqrt(u.squared_length()); }
    u = u / lu;
    Vector_3 v = CGAL::cross_product(normal, u); // уже нормальный и ортогональный

    const double W = N * cellSize, H = M * cellSize;
    const Vector_3 originOffset = (-0.5*W)*u + (-0.5*H)*v;

    // Вершины
    std::vector<std::vector<SurfaceMesh::Vertex_index>> V(N+1, std::vector<SurfaceMesh::Vertex_index>(M+1));
    for (int i=0;i<=N;++i)
        for (int j=0;j<=M;++j) {
            Vector_3 offset = originOffset + (i*cellSize)*u + (j*cellSize)*v;
            V[i][j] = sm.add_vertex(center + offset);
        }

    // Квады CCW так, чтобы нормаль получилась ≈ +normal
    for (int i=0;i<N;++i)
        for (int j=0;j<M;++j) {
            auto v00=V[i][j], v10=V[i+1][j], v11=V[i+1][j+1], v01=V[i][j+1];
            sm.add_face(v00, v10, v11, v01);
        }
}


namespace {

// Разбивает граничные вершины, через которые проходит >1 граничная «дорожка».
// То есть, если вокруг вершины есть несколько отдельных подряд идущих блоков
// пограничных полу-ребер, каждую такую группу (кроме первой) отделяем
// отдельной вершиной с теми же координатами.
static void split_kissing_border_vertices(SurfaceMesh& sm)
{
    std::vector<SurfaceMesh::Vertex_index> candidates;
    candidates.reserve(sm.number_of_vertices());
    for (auto v : sm.vertices())
        if (!sm.is_removed(v) && CGAL::is_border(v, sm))
            candidates.push_back(v);

    for (auto v : candidates)
    {
        // все полуребра, приходящие в v (target = v) по кругу
        std::vector<SurfaceMesh::Halfedge_index> ring;
        for (auto h : CGAL::halfedges_around_target(v, sm))
            ring.push_back(h);
        if (ring.empty()) continue;

        auto isB = [&](int i)->bool { return CGAL::is_border(ring[i], sm); };

        // Найдём группы подряд идущих граничных полурёбер
        std::vector<std::pair<int,int>> groups; // [begin,end] по ring
        const int n = (int)ring.size();
        int i = 0;
        while (i < n) {
            // пропускаем неграничные
            while (i < n && !isB(i)) ++i;
            if (i == n) break;
            int b = i;
            while (i < n && isB(i)) ++i;
            int e = i - 1;
            groups.emplace_back(b, e);
        }

        if (groups.size() <= 1) continue; // ничего делить

        // Отделяем каждую группу, начиная со второй.
        // split_vertex(h1,h2,sm) отделяет сектор [h1..h2] в новую вершину.
        for (size_t g = 1; g < groups.size(); ++g) {
            auto h_begin = ring[groups[g].first];
            auto h_end   = ring[groups[g].second];
            // в некоторых версиях возвращает пару halfedge'ов — игнорируем
            CGAL::Euler::split_vertex(h_begin, h_end, sm);

            // Дублировать позицию не нужно: split_vertex сам создаёт новую вершину
            // с той же позицией, что у исходной. Если у вашей версии нет —
            // можно явно: sm.point(target(h_begin, sm)) = sm.point(v);
        }
    }

    sm.collect_garbage();
}

static std::vector<std::vector<SurfaceMesh::Face_index>>
partition_non_adjacent(SurfaceMesh& sm,
                       const std::vector<SurfaceMesh::Face_index>& in)
{
    // index map for quick lookups
    std::unordered_map<int,int> idx; // key = f.idx()
    idx.reserve(in.size()*2);
    for (int i=0;i<(int)in.size();++i) idx[in[i].idx()] = i;

    // adjacency by shared edge
    std::vector<std::vector<int>> adj(in.size());
    for (auto e : sm.edges()) if (!sm.is_removed(e)) {
            auto h  = halfedge(e, sm);
            auto f0 = face(h, sm);
            auto f1 = face(opposite(h, sm), sm);
            auto it0 = idx.find(f0.idx());
            auto it1 = idx.find(f1.idx());
            if (it0 != idx.end() && it1 != idx.end()) {
                int i = it0->second, j = it1->second;
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }

    // greedy coloring
    std::vector<int> color(in.size(), -1);
    int maxColor = -1;
    for (int i=0;i<(int)in.size();++i) {
        std::unordered_set<int> used;
        for (int j : adj[i]) if (color[j] >= 0) used.insert(color[j]);
        int c = 0; while (used.count(c)) ++c;
        color[i] = c;
        maxColor = std::max(maxColor, c);
    }

    std::vector<std::vector<SurfaceMesh::Face_index>> batches(maxColor+1);
    for (int i=0;i<(int)in.size();++i) batches[color[i]].push_back(in[i]);
    return batches;
}

} // namespace


// ---------------- Stage 2 ----------------
std::vector<SurfaceMesh::Face_index>
CgalMeshBuilder::selectFacesRandom(const SurfaceMesh& sm, double p01, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::vector<SurfaceMesh::Face_index> out;
    for (auto f : sm.faces()) if (!sm.is_removed(f)) {
            if (U(rng) < p01) out.push_back(f);
        }
    return out;
}

void CgalMeshBuilder::deleteFaces(SurfaceMesh& sm,
                                  const std::vector<SurfaceMesh::Face_index>& faces,
                                  bool split_kissing_vertices /* = true */)
{
    for (auto f : faces) {
        if (f == SurfaceMesh::null_face() || sm.is_removed(f)) continue;
        CGAL::Euler::remove_face(sm.halfedge(f), sm);

    }
    sm.collect_garbage();

    if (split_kissing_vertices) {
        PMP::duplicate_non_manifold_vertices(sm); // splits “kissing” corners
        sm.collect_garbage();
    }
}

void CgalMeshBuilder::cleanup_after_deletions(SurfaceMesh& sm) {
    namespace PMP = CGAL::Polygon_mesh_processing;
    // 1) убрать вырожденные лица (повторы вершин/нулевая площадь)
  //  PMP::remove_degenerate_faces(sm);
    // 2) раздублировать поцелуйные вершины (у тебя уже делается, но пусть будет тут)
    PMP::duplicate_non_manifold_vertices(sm);
    split_kissing_border_vertices(sm);
    // 3) выбросить изолированные вершины
    remove_isolated_vertices_safe(sm);
    // 4) компактнуть индексы
    sm.collect_garbage();
}

static inline void refresh_face_uid_map(
    SurfaceMesh& sm,
    SurfaceMesh::Property_map<SurfaceMesh::Face_index, std::uint64_t>& f_uid,
    std::uint64_t& nextUID)
{
    // Returns existing map if present, otherwise creates it with default 0
    f_uid = sm.add_property_map<SurfaceMesh::Face_index, std::uint64_t>("f:uid", 0).first;

    std::uint64_t max_uid = 0;
    for (auto f : sm.faces()) {
        if (sm.is_removed(f)) continue;
        auto& id = f_uid[f];
        if (id == 0) id = ++max_uid;       // assign missing ids
        else         max_uid = std::max(max_uid, id);
    }
    nextUID = max_uid + 1;
}

// маленький помощник: положить грань в нужные корзины и проставить UID
static inline void push_created_face(SurfaceMesh::Face_index f,
                                     bool is_cap,
                                     SurfaceMesh::Property_map<SurfaceMesh::Face_index, std::uint64_t>& f_uid,
                                     std::uint64_t& nextUID,
                                     std::uint64_t parent_uid,
                                     ExtrudeLists* out)
{
    if (f == SurfaceMesh::null_face()) return;
    // Боковины получают новый uid, крышка наследует uid исходной грани
    f_uid[f] = is_cap ? parent_uid : nextUID++;
    if (out) {
        out->all.push_back(f);
        if (is_cap) out->caps.push_back(f);
    }
}
// -----------------------------------------------------------------------------
// 3) Экструзия ОДНОЙ грани — НИЧЕГО не пишем в f_uid внутри! Только собираем out
// -----------------------------------------------------------------------------
SurfaceMesh::Face_index CgalMeshBuilder::extrudeFace_one(
    SurfaceMesh& sm,
    SurfaceMesh::Face_index f,
    double distance,
    double scale,
    /* НЕ НУЖНО юзать f_uid здесь */
    SurfaceMesh::Property_map<SurfaceMesh::Face_index, std::uint64_t>& /*f_uid*/,
    std::uint64_t& /*nextUID*/,
    ExtrudeLists* out)
{
    using SM = SurfaceMesh;
    if (f == SM::null_face() || sm.is_removed(f)) return SM::null_face();

    // 1) base[i] = source(h_i) — важно для согласованной ориентации
    std::vector<SM::Halfedge_index> ringH;
    std::vector<SM::Vertex_index>   base;
    {
        auto h0 = sm.halfedge(f);
        if (h0 == SM::null_halfedge()) return SM::null_face();
        auto h = h0;
        do {
            if (sm.is_removed(h)) return SM::null_face();
            ringH.push_back(h);
            base.push_back(source(h, sm));
            h = next(h, sm);
        } while (h != h0);
    }
    const std::size_t k = base.size();
    if (k < 3) return SM::null_face();

    // 2) верхнее кольцо
    Point_3  c = average_point(sm, base);
    Vector_3 n = face_normal_unit(sm, base);
    const Point_3 Cmesh = mesh_centroid(sm);
    if (CGAL::scalar_product(n, c - Cmesh) < 0.0) n = -n;

    std::vector<SM::Vertex_index> top(k);
    for (std::size_t i = 0; i < k; ++i) {
        const Point_3 Pi = sm.point(base[i]);
        const Vector_3 dc(Pi.x()-c.x(), Pi.y()-c.y(), Pi.z()-c.z());
        const Point_3  Ptop( c.x() + n.x()*distance + scale*dc.x(),
                           c.y() + n.y()*distance + scale*dc.y(),
                           c.z() + n.z()*distance + scale*dc.z() );
        top[i] = sm.add_vertex(Ptop);
    }

    // 3) удаляем исходную грань → её vi->vj становятся граничными
    CGAL::Euler::remove_face(sm.halfedge(f), sm);

    // 4) боковины: { base[i], base[j], top[j], top[i] }
    for (std::size_t i = 0, j = 1; i < k; ++i, j = (i + 1) % k) {
        auto fwall = CGAL::Euler::add_face(
            std::array<SM::Vertex_index,4>{ base[i], base[j], top[j], top[i] }, sm);
        if (out && fwall != SM::null_face()) out->all.push_back(fwall);
        if (fwall == SM::null_face()) {
            auto f1 = CGAL::Euler::add_face(std::array<SM::Vertex_index,3>{ base[i], base[j], top[j] }, sm);
            auto f2 = CGAL::Euler::add_face(std::array<SM::Vertex_index,3>{ base[i], top[j],  top[i] }, sm);
            if (out && f1 != SM::null_face()) out->all.push_back(f1);
            if (out && f2 != SM::null_face()) out->all.push_back(f2);
        }
    }

    // 5) крышка
    SM::Face_index ftop = SM::null_face();
    {
        Vector_3 nt = face_normal_unit(sm, top);
        const bool sameDir = (CGAL::scalar_product(nt, n) > 0.0);
        if (sameDir)  ftop = CGAL::Euler::add_face(top, sm);
        else {
            std::vector<SM::Vertex_index> r(top.rbegin(), top.rend());
            ftop = CGAL::Euler::add_face(r, sm);
        }
        if (out && ftop != SM::null_face()) {
            out->all.push_back(ftop);
            out->caps.push_back(ftop);
        }
        if (ftop == SM::null_face()) {
            for (std::size_t i = 1; i + 1 < k; ++i) {
                auto ft = CGAL::Euler::add_face(
                    std::array<SM::Vertex_index,3>{ top[0], top[i], top[i+1] }, sm);
                if (out && ft != SM::null_face()) {
                    out->all.push_back(ft);
                    out->caps.push_back(ft);
                    if (i == 1) ftop = ft;
                }
            }
        }
    }
    return ftop;
}

static inline void reserve_for_extrusion(
    SurfaceMesh& sm,
    const std::vector<SurfaceMesh::Face_index>& todo)
{
    std::size_t addK = 0;
    for (auto f : todo) {
        if (f == SurfaceMesh::null_face() || sm.is_removed(f)) continue;
        auto h0 = sm.halfedge(f); if (h0 == SurfaceMesh::null_halfedge()) continue;
        std::size_t k = 0; auto h = h0; do { ++k; h = next(h, sm); } while (h != h0);
        addK += k;
    }
    // очень грубо: на k-угольник добавляем k вершин и ~ (k боковин + 1 крышку)
    sm.reserve(sm.number_of_vertices()  + addK,
               sm.number_of_halfedges() + addK * 10, // безопасный запас
               sm.number_of_faces()     + addK + todo.size());
}

SurfaceMesh::Face_index CgalMeshBuilder::extrudeFace_one_internalOnly(
    SurfaceMesh& sm, SurfaceMesh::Face_index f, double distance, double scale, ExtrudeLists* out)
{
    using SM = SurfaceMesh;
    if (f == SM::null_face() || sm.is_removed(f)) return SM::null_face();

    // 1) кольцо грани как хранится
    std::vector<SM::Vertex_index> base;
    std::vector<Point_3>          baseP;
    {
        auto h0 = sm.halfedge(f); if (h0 == SM::null_halfedge()) return SM::null_face();
        auto h  = h0;
        std::unordered_set<SM::Vertex_index> seen;
        do {
            if (sm.is_removed(h)) return SM::null_face();
            auto v = source(h, sm);
            if (v == SM::null_vertex() || sm.is_removed(v)) return SM::null_face();
            if (!seen.insert(v).second) return SM::null_face();
            base.push_back(v);
            baseP.push_back(sm.point(v));
            h = next(h, sm);
        } while (h != h0);
        if (base.size() < 3) return SM::null_face();
    }
    const size_t k = base.size();

    // 2) верхнее кольцо (локально)
    auto avg = [](const std::vector<Point_3>& R)->Point_3{
        double x=0,y=0,z=0; for (auto& p:R){ x+=p.x(); y+=p.y(); z+=p.z(); }
        double inv = 1.0 / double(R.size()); return Point_3(x*inv,y*inv,z*inv);
    };
    auto newell = [](const std::vector<Point_3>& P)->Vector_3{
        if (P.size()<3) return Vector_3(0,0,1);
        Vector_3 n(0,0,0);
        for (size_t i=0;i<P.size();++i){
            const auto& a=P[i]; const auto& b=P[(i+1)%P.size()];
            n = n + Vector_3((a.y()-b.y())*(a.z()+b.z()),
                             (a.z()-b.z())*(a.x()+b.x()),
                             (a.x()-b.x())*(a.y()+b.y()));
        }
        const double L = std::sqrt(n.squared_length());
        return (L>1e-12)? n/L : Vector_3(0,0,1);
    };

    const Point_3 c = avg(baseP);
    const Vector_3 n = newell(baseP);

    std::vector<SM::Vertex_index> top(k);
    std::vector<Point_3>          topP(k);
    for (size_t i=0;i<k;++i){
        const Vector_3 dc(baseP[i].x()-c.x(), baseP[i].y()-c.y(), baseP[i].z()-c.z());
        const Point_3  Ptop(c.x()+n.x()*distance + scale*dc.x(),
                           c.y()+n.y()*distance + scale*dc.y(),
                           c.z()+n.z()*distance + scale*dc.z());
        top[i]  = sm.add_vertex(Ptop);
        topP[i] = Ptop;
    }

    auto area2 = [](const Point_3& P, const Point_3& R, const Point_3& S){
        const Vector_3 u(R.x()-P.x(),R.y()-P.y(),R.z()-P.z());
        const Vector_3 v(S.x()-P.x(),S.y()-P.y(),S.z()-P.z());
        return CGAL::cross_product(u,v).squared_length();
    };
    auto alive = [&](SM::Vertex_index v){ return v!=SM::null_vertex() && !sm.is_removed(v); };
    auto try_add_quad_keep_lower = [&](SM::Vertex_index q0, SM::Vertex_index q1,
                                       SM::Vertex_index q2, SM::Vertex_index q3)->SM::Face_index {
        using A4 = std::array<SM::Vertex_index,4>;
        // сохраняем направление нижнего ребра q0->q1 во всех вариантах
        if (auto f1 = CGAL::Euler::add_face(A4{q0,q1,q2,q3}, sm); f1 != SM::null_face()) return f1;
        if (auto f2 = CGAL::Euler::add_face(A4{q1,q2,q3,q0}, sm); f2 != SM::null_face()) return f2;
        if (auto f3 = CGAL::Euler::add_face(A4{q2,q3,q0,q1}, sm); f3 != SM::null_face()) return f3;
        return CGAL::Euler::add_face(A4{q3,q0,q1,q2}, sm);
    };

    // 3) удаляем исходную грань — теперь вдоль каждого ребра остался border halfedge (a->b)
    CGAL::Euler::remove_face(sm.halfedge(f), sm);

    // 4) стены: низ всегда (a->b), верх всегда (top[b] -> top[a]) — единая ориентация для всех
    for (size_t i=0, j=1; i<k; ++i, j=(i+1)%k){
        SM::Vertex_index a = base[i], b = base[j];
        if (!alive(a) || !alive(b)) continue;

        // отсечка вырожденности
        const Point_3 &A = baseP[i], &B = baseP[j], &C = topP[j], &D = topP[i];
        if (a==b || a==top[j] || b==top[i]) continue;
        if (area2(A,B,C) <= 1e-24 && area2(A,C,D) <= 1e-24) continue;

        // ВАЖНО: низ (a->b), верх (top[b]->top[a])
        if (SM::Face_index fw = try_add_quad_keep_lower(a, b, top[j], top[i]); fw != SM::null_face())
            if (out) out->all.push_back(fw);
    }

    // 5) крышка: cap идёт top[0]→top[1]→… (против верха стен)
    std::vector<SM::Vertex_index> cap(top.begin(), top.end());
    SM::Face_index fcap = SM::null_face();
    for (size_t t=0; t<k; ++t){ // попробуем разные стартовые halfedge
        fcap = CGAL::Euler::add_face(cap, sm);
        if (fcap != SM::null_face()) break;
        std::rotate(cap.begin(), cap.begin()+1, cap.end());
    }
    if (out && fcap != SM::null_face()) { out->all.push_back(fcap); out->caps.push_back(fcap); }
    return fcap;
}



ExtrudeLists CgalMeshBuilder::extrudeFaces_collectBoth(
    SurfaceMesh& sm,
    const std::vector<SurfaceMesh::Face_index>& faces,
    double distance,
    double scale)
{
    sm.collect_garbage(); // один раз до модификаций

    std::vector<SurfaceMesh::Face_index> todo;
    todo.reserve(faces.size());
    for (auto f : faces)
        if (f != SurfaceMesh::null_face() && !sm.is_removed(f))
            todo.push_back(f);

    ExtrudeLists out;
    out.all.reserve(todo.size()*5);

    for (auto f : todo)
        extrudeFace_one_internalOnly(sm, f, distance, scale, &out);

    return out;
}


ExtrudeLists
CgalMeshBuilder::extrudeRegion(
    SurfaceMesh& sm,
    const std::vector<SurfaceMesh::Face_index>& region_faces,
    double distance,
    double scale)
{
    using SM = SurfaceMesh;
    using V_idx = SM::Vertex_index;
    using F_idx = SM::Face_index;
    using H_idx = SM::Halfedge_index;

    ExtrudeLists results;
    if (region_faces.empty()) {
        return results;
    }

    // --- ШАГ 0: Подготовка данных ---
    auto [f_uid_map, nextUID] = ensure_face_uid_map(sm);

    using V_hash = CGAL::Handle_hash_function;
    using F_hash = CGAL::Handle_hash_function;

    std::unordered_set<F_idx, F_hash> face_set(region_faces.begin(), region_faces.end());

    // --- ШАГ 1: Найти периметр региона и все его вершины ---
    // --- ИЗМЕНЕНИЕ: Храним пары вершин, а не полурёбра ---
    std::vector<std::pair<V_idx, V_idx>> boundary_edges;
    std::unordered_set<V_idx, V_hash> region_vertices_set;

    for (auto f : region_faces) {
        if (sm.is_removed(f)) continue;
        for (auto h : sm.halfedges_around_face(sm.halfedge(f))) {
            region_vertices_set.insert(sm.target(h));

            H_idx h_opp = sm.opposite(h);
            if (h_opp == SM::null_halfedge() || face_set.find(sm.face(h_opp)) == face_set.end()) {
                // Сразу извлекаем и сохраняем стабильные Vertex_index
                boundary_edges.push_back({sm.source(h), sm.target(h)});
            }
        }
    }

    // --- ШАГ 2: Создать "Крышку" (верхние вершины и грани) ---
    // ... (эта часть кода не меняется, она была корректной) ...
    Vector_3 region_normal(0, 0, 0);
    Point_3 region_center(0, 0, 0);
    double total_area = 0;
    for (auto f : region_faces) {
        if (!sm.is_removed(f)) {
            double area = PMP::face_area(f, sm);
            if (area > 1e-12) {
                total_area += area;
                region_normal = region_normal + PMP::compute_face_normal(f, sm) * area;
            }
        }
    }
    if (total_area > 1e-12) region_normal = region_normal / total_area;

    if (region_normal.squared_length() < 1e-12) region_normal = Vector_3(0,0,1);
    else region_normal = region_normal / std::sqrt(region_normal.squared_length());

    for (auto v : region_vertices_set) region_center = Point_3(region_center.x() + sm.point(v).x(), region_center.y() + sm.point(v).y(), region_center.z() + sm.point(v).z());
    if(!region_vertices_set.empty()) region_center = Point_3(region_center.x() / region_vertices_set.size(), region_center.y() / region_vertices_set.size(), region_center.z() / region_vertices_set.size());

    if (CGAL::scalar_product(region_normal, region_center - mesh_centroid(sm)) < 0.0) {
        region_normal = -region_normal;
    }

    std::unordered_map<V_idx, V_idx, V_hash> base_to_top_vertex;
    for (auto v_base : region_vertices_set) {
        const Point_3& p_base = sm.point(v_base);
        const Vector_3 dc = p_base - region_center;
        const Point_3 p_top = region_center + (region_normal * distance) + (dc * scale);
        base_to_top_vertex[v_base] = sm.add_vertex(p_top);
    }

    for (auto f_base : region_faces) {
        if (sm.is_removed(f_base)) continue;

        std::vector<V_idx> base_face_vertices = face_vertices_ring(sm, f_base);
        std::vector<V_idx> top_face_vertices;
        top_face_vertices.reserve(base_face_vertices.size());
        for (auto v_base : base_face_vertices) {
            top_face_vertices.push_back(base_to_top_vertex.at(v_base));
        }

        auto f_top = CGAL::Euler::add_face(top_face_vertices, sm);
        push_created_face(f_top, true, f_uid_map, nextUID, f_uid_map[f_base], &results);
    }

    // --- ШАГ 3: Удалить оригинальные грани ---
    for (auto f_base : region_faces) {
        if (!sm.is_removed(f_base)) {
            CGAL::Euler::remove_face(sm.halfedge(f_base), sm);
        }
    }

    // --- ШАГ 4: Построить боковые стенки по периметру ---
    // --- ИЗМЕНЕНИЕ: Используем сохранённые пары вершин ---
    for (const auto& edge : boundary_edges) {
        auto v_start_base = edge.first;
        auto v_end_base = edge.second;
        auto v_start_top = base_to_top_vertex.at(v_start_base);
        auto v_end_top = base_to_top_vertex.at(v_end_base);

        auto f_wall = CGAL::Euler::add_face(
            std::array<V_idx, 4>{v_start_base, v_end_base, v_end_top, v_start_top}, sm);
        push_created_face(f_wall, false, f_uid_map, nextUID, 0, &results);
    }

    sm.collect_garbage();
    return results;
}

void CgalMeshBuilder::applyCatmullClark(SurfaceMesh& sm, int iterations , bool keep_borders)
{
    if (iterations <= 0) return;

    // Optional: preserve borders/creases
    auto constrained = sm.add_property_map<SurfaceMesh::Edge_index, bool>("e:constrained", false).first;
    if (keep_borders) {
        for (auto e : sm.edges())
            if (is_border(e, sm)) constrained[e] = true;
    }

    // Run Catmull–Clark
    S3::CatmullClark_subdivision(
        sm,
        CGAL::parameters::number_of_iterations(iterations)
            .vertex_point_map(get(CGAL::vertex_point, sm))
            .edge_is_constrained_map(constrained)
        );
}

void CgalMeshBuilder::subdivideQuadFacesGrid(SurfaceMesh& sm, int nx, int ny)
{
    if (nx <= 1 && ny <= 1) return;

    std::vector<SurfaceMesh::Face_index> faces;
    faces.reserve(sm.number_of_faces());
    for (const auto& f : sm.faces()) {
        faces.push_back(f);
    }

    for (const auto& f : faces) {
        if (sm.is_removed(f)) continue;

        std::vector<SurfaceMesh::Vertex_index> ring;
        for(const auto& v_idx : sm.vertices_around_face(sm.halfedge(f))) {
            ring.push_back(v_idx);
        }
        if (ring.size() != 4) continue;

        const Point_3 p00 = sm.point(ring[0]);
        const Point_3 p10 = sm.point(ring[1]);
        const Point_3 p11 = sm.point(ring[2]);
        const Point_3 p01 = sm.point(ring[3]);

        std::vector<std::vector<SurfaceMesh::Vertex_index>> G(ny + 1);
        for (int j = 0; j <= ny; ++j) {
            G[j].resize(nx + 1);
            const double v = (ny == 0) ? 0.0 : static_cast<double>(j) / ny;
            for (int i = 0; i <= nx; ++i) {
                const double u = (nx == 0) ? 0.0 : static_cast<double>(i) / nx;

                if (j == 0  && i == 0)  { G[j][i] = ring[0]; continue; }
                if (j == 0  && i == nx) { G[j][i] = ring[1]; continue; }
                if (j == ny && i == nx) { G[j][i] = ring[2]; continue; }
                if (j == ny && i == 0)  { G[j][i] = ring[3]; continue; }

                const Point_3 P = bilerp(p00, p10, p11, p01, u, v);
                G[j][i] = sm.add_vertex(P);
            }
        }

        sm.remove_face(f);

        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                sm.add_face(G[j][i], G[j][i + 1], G[j + 1][i + 1], G[j + 1][i]);
            }
        }
    }

    // ИСПРАВЛЕНИЕ: "Свариваем" дублирующиеся вершины, созданные на ребрах.
    // Эта строка объединит вершины, которые находятся в одной и той же позиции,
    // восстанавливая корректную топологию "водонепроницаемой" сетки.
    //CGAL::Polygon_mesh_processing::weld_vertices(sm);

    sm.collect_garbage();
}

void CgalMeshBuilder::buildCubeWithGrid(SurfaceMesh& sm, double size, int nx, int ny)
{
    sm.clear();
    nx = std::max(1, nx);
    ny = std::max(1, ny);

    const double h = 0.5 * size;

    // 8 cube corners
    const std::array<Point_3,8> P = {
        Point_3(-h,-h,-h), Point_3( h,-h,-h),
        Point_3( h, h,-h), Point_3(-h, h,-h),
        Point_3(-h,-h, h), Point_3( h,-h, h),
        Point_3( h,  h, h), Point_3(-h,  h, h)
    };

    // 6 faces as CCW quads (viewed from outside). We will tessellate each.
    const std::array<std::array<int,4>,6> F = {{
        {{0,1,2,3}}, // -Z
        {{4,5,6,7}}, // +Z
        {{0,4,5,1}}, // -Y
        {{3,2,6,7}}, // +Y
        {{0,3,7,4}}, // -X
        {{1,5,6,2}}  // +X
    }};

    for (const auto& f : F) {
        const Point_3 p00 = P[f[0]];
        const Point_3 p10 = P[f[1]];
        const Point_3 p11 = P[f[2]];
        const Point_3 p01 = P[f[3]];

        // grid of (ny+1)x(nx+1) vertices (unique per face; no cross-face sharing → robust)
        std::vector<std::vector<SurfaceMesh::Vertex_index>> G(ny+1);
        for (int j = 0; j <= ny; ++j) {
            G[j].resize(nx+1);
            const double v = double(j) / ny;
            for (int i = 0; i <= nx; ++i) {
                const double u = double(i) / nx;
                G[j][i] = sm.add_vertex( bilerpP(p00,p10,p11,p01, u,v) );
            }
        }

        // emit nx*ny quads; order keeps CCW w.r.t. outward normal
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                CGAL::Euler::add_face(
                    std::array<SurfaceMesh::Vertex_index,4>{
                        G[j][i], G[j][i+1], G[j+1][i+1], G[j+1][i]
                    },
                    sm);
            }
        }
    }
    sm.collect_garbage();
}
// ---------------- Stage 4 ----------------
void CgalMeshBuilder::triangulateAll(SurfaceMesh& sm) {
    PMP::triangulate_faces(sm);
}
void CgalMeshBuilder::triangulateFaces(SurfaceMesh& sm,
                                       const std::vector<SurfaceMesh::Face_index>& faces) {
    // triangulate only a subset
    PMP::triangulate_faces(faces, sm);
}

// ---------------- Stage 5 ----------------
static inline glm::vec3 to_glm(const Point_3& p) {
    return glm::vec3( (float)p.x(), (float)p.y(), (float)p.z() );
}
static inline glm::vec3 unit3(const glm::vec3& v) {
    float l2 = glm::dot(v,v); return (l2>0.f)? v/std::sqrt(l2) : glm::vec3(0,0,1);
}

void CgalMeshBuilder::toVertexIndexFlat(const SurfaceMesh& sm,
                                        std::vector<Vertex>& outV,
                                        std::vector<uint32_t>& outI,
                                        glm::vec4 color)
{
    outV.clear(); outI.clear();
    const glm::vec3 C = to_glm(mesh_centroid(sm));

    for (auto f : sm.faces()) {
        if (sm.is_removed(f)) continue;

        std::vector<SurfaceMesh::Vertex_index> ring;
        auto h = sm.halfedge(f), it = h;
        do { ring.push_back(target(it, sm)); it = next(it, sm); } while (it != h);
        if (ring.size() < 3)
            continue;

        // face normal (Newell) + outward flip
        glm::vec3 n(0);
        for (size_t i=0; i<ring.size(); ++i) {
            const glm::vec3 pi = to_glm(sm.point(ring[i]));
            const glm::vec3 pj = to_glm(sm.point(ring[(i+1)%ring.size()]));
            n.x += (pi.y - pj.y) * (pi.z + pj.z);
            n.y += (pi.z - pj.z) * (pi.x + pj.x);
            n.z += (pi.x - pj.x) * (pi.y + pj.y);
        }
        n = unit3(n);
        glm::vec3 fc(0); for (auto v : ring) fc += to_glm(sm.point(v));
        fc *= 1.f / float(ring.size());
        if (glm::dot(n, fc - C) < 0.f) n = -n;

        // emit triangle fan — duplicate verts so each tri gets face normal
        for (size_t i = 1; i + 1 < ring.size(); ++i) {
            const glm::vec3 a = to_glm(sm.point(ring[0]));
            const glm::vec3 b = to_glm(sm.point(ring[i]));
            const glm::vec3 c = to_glm(sm.point(ring[i+1]));
            const uint32_t base = (uint32_t)outV.size();
            outV.push_back({ glm::vec4(a,1), glm::vec4(n,0), color });
            outV.push_back({ glm::vec4(b,1), glm::vec4(n,0), color });
            outV.push_back({ glm::vec4(c,1), glm::vec4(n,0), color });
            outI.push_back(base+0); outI.push_back(base+1); outI.push_back(base+2);
        }
    }
}



// --- vertex normals (area-weighted) ---
static std::unordered_map<SurfaceMesh::Vertex_index, Vector_3>
compute_vertex_normals(const SurfaceMesh& sm)
{
    std::unordered_map<SurfaceMesh::Vertex_index, Vector_3> N;
    N.reserve(sm.number_of_vertices());
    for (auto v : sm.vertices()) if (!sm.is_removed(v)) N[v] = Vector_3(0,0,0);

    for (auto f : sm.faces()) if (!sm.is_removed(f)) {
            // face normal via Newell around centroid (length ~ area)
            std::vector<SurfaceMesh::Vertex_index> ring;
            auto h = sm.halfedge(f), it = h;
            do { ring.push_back(target(it, sm)); it = next(it, sm); } while (it != h);
            if (ring.size() < 3) continue;

            // centroid
            Point_3 c(0,0,0);
            for (auto v : ring) { auto p = sm.point(v); c = Point_3(c.x()+p.x(), c.y()+p.y(), c.z()+p.z()); }
            c = Point_3(c.x()/ring.size(), c.y()/ring.size(), c.z()/ring.size());

            Vector_3 nf(0,0,0);
            for (size_t i=0;i<ring.size();++i) {
                const auto pi = sm.point(ring[i]);
                const auto pj = sm.point(ring[(i+1)%ring.size()]);
                nf = nf + CGAL::cross_product(pj - c, pi - c);
            }
            for (auto v : ring) N[v] = N[v] + nf; // area-weighted accumulate
        }

    // normalize
    for (auto& kv : N) {
        double L = std::sqrt(kv.second.squared_length());
        kv.second = (L>0.0) ? kv.second/L : Vector_3(0,0,1);
    }
    return N;
}

// --- collect all border halfedge cycles (you already have a variant) ---
static std::vector<std::vector<SurfaceMesh::Vertex_index>>
collect_border_vertex_loops(const SurfaceMesh& sm)
{
    std::vector<std::vector<SurfaceMesh::Vertex_index>> loops;
    std::unordered_set<SurfaceMesh::Halfedge_index> seen;
    for (auto h : sm.halfedges()) {
        if (!is_border(h, sm) || seen.count(h)) continue;
        std::vector<SurfaceMesh::Vertex_index> cyc;
        auto start = h, cur = h;
        do { cyc.push_back(target(cur, sm)); seen.insert(cur); cur = next(cur, sm); } while (cur != start);
        loops.push_back(std::move(cyc));
    }
    return loops;
}

// --- K-ring BFS distances from all border vertices at once ---
static std::unordered_map<SurfaceMesh::Vertex_index,int>
border_k_rings(const SurfaceMesh& sm, int rings)
{
    std::unordered_map<SurfaceMesh::Vertex_index,int> dist;
    std::queue<SurfaceMesh::Vertex_index> q;

    for (auto h : sm.halfedges()) if (is_border(h, sm)) {
            auto v = target(h, sm);
            if (!dist.count(v)) { dist[v]=0; q.push(v); }
        }
    while (!q.empty()) {
        auto v = q.front(); q.pop();
        int d = dist[v];
        if (d >= rings) continue;
        // neighbors via one-ring
        for (auto h : CGAL::halfedges_around_target(v, sm)) {
            auto u = source(h, sm);
            if (!dist.count(u)) { dist[u] = d+1; q.push(u); }
        }
    }
    return dist;
}

// --- smooth falloff (0..1 -> 0..1), cosine-ease then exponent for "sharpness" ---
static inline double falloff(double t, double sharpness) {
    t = std::clamp(t, 0.0, 1.0);                  // t=1 at rim, 0 at outer ring
    double w = 0.5*(1.0 - std::cos(M_PI * t));    // cosine smooth
    if (sharpness != 1.0) w = std::pow(w, sharpness);
    return w;
}

// --- the fillet itself ---
void CgalMeshBuilder::filletBorderLoops(SurfaceMesh& sm, int rings, double height, bool outward, double sharpness)
{
    if (rings <= 0 || height == 0.0) return;

    // vertex normals (once)
    auto N = compute_vertex_normals(sm);

    // k-ring distances from all border vertices
    auto D = border_k_rings(sm, rings);

    // maximum distance present (<= rings)
    int dmax = 0; for (auto& kv : D) dmax = std::max(dmax, kv.second);
    if (dmax == 0) return;

    // displace vertices within the band [0..dmax]
    for (auto& kv : D) {
        auto v = kv.first; int d = kv.second;          // 0 at rim
        double t = 1.0 - double(d)/double(dmax);       // 1 at rim → 0 at outer
        double w = falloff(t, sharpness);
        Vector_3 dir = outward ? N[v] : -N[v];
        sm.point(v) = sm.point(v) + dir * (height * w);
    }
}


// ---------------- Circularize border loops ----------------
// helpers
static inline Vector_3 unit3(const Vector_3& v){
    double L = std::sqrt(v.squared_length());
    return (L>0.0)? v/L : Vector_3(1,0,0);
}
static inline double dot3(const Vector_3& a, const Vector_3& b){ return a.x()*b.x()+a.y()*b.y()+a.z()*b.z(); }

static std::vector<std::vector<SurfaceMesh::Halfedge_index>>
collect_border_halfedge_cycles(const SurfaceMesh& sm)
{
    std::vector<std::vector<SurfaceMesh::Halfedge_index>> loops;
    std::unordered_set<SurfaceMesh::Halfedge_index> seen;
    for (auto h : sm.halfedges()) {
        if (!is_border(h, sm) || seen.count(h)) continue;
        std::vector<SurfaceMesh::Halfedge_index> cyc;
        auto start = h, cur = h;
        do {
            cyc.push_back(cur);
            seen.insert(cur);
            cur = next(cur, sm);   // along the border cycle
        } while (cur != start);
        loops.push_back(std::move(cyc));
    }
    return loops;
}

void CgalMeshBuilder::circularizeBorderLoops(SurfaceMesh& sm, double scale)
{
    // 1) find all border cycles (each hole rim is one cycle)
    auto loops = collect_border_halfedge_cycles(sm);
    if (loops.empty()) return;

    for (const auto& cyc : loops) {
        // 2) gather vertices in order
        std::vector<SurfaceMesh::Vertex_index> verts;
        verts.reserve(cyc.size());
        for (auto h : cyc) verts.push_back(target(h, sm));

        // 3) plane (normal via Newell around centroid)
        Point_3 C(0,0,0);
        for (auto v : verts) { const auto& p = sm.point(v); C = Point_3(C.x()+p.x(), C.y()+p.y(), C.z()+p.z()); }
        C = Point_3(C.x()/verts.size(), C.y()/verts.size(), C.z()/verts.size());

        Vector_3 N(0,0,0);
        for (size_t i=0;i<verts.size();++i) {
            const auto& pi = sm.point(verts[i]);
            const auto& pj = sm.point(verts[(i+1)%verts.size()]);
            N = N + CGAL::cross_product(pj - C, pi - C);
        }
        N = unit3(N);

        // 4) build orthonormal basis (u,v) in plane
        // pick any edge direction, project to plane
        Vector_3 t = sm.point(verts[1]) - sm.point(verts[0]);
        t = unit3( t - N*dot3(t,N) );              // tangent in plane
        Vector_3 u = t;
        Vector_3 v = unit3( CGAL::cross_product(N, u) );

        // 5) project vertices to 2D, compute center & avg radius
        double cx=0, cy=0;
        std::vector<std::pair<double,double>> pv; pv.reserve(verts.size());
        for (auto vtx : verts) {
            Vector_3 d = sm.point(vtx) - C;
            double x = dot3(d, u);
            double y = dot3(d, v);
            pv.emplace_back(x,y);
            cx += x; cy += y;
        }
        cx /= pv.size(); cy /= pv.size();

        double r = 0.0;
        for (auto [x,y] : pv) { double dx=x-cx, dy=y-cy; r += std::sqrt(dx*dx + dy*dy); }
        r = (r / pv.size()) * ((scale>0.0)? scale : 1.0);

        // 6) place vertices uniformly around the circle (preserves order)
        const double TWO_PI = 6.283185307179586;
        for (size_t i=0;i<verts.size(); ++i) {
            double theta = TWO_PI * (double(i) / double(verts.size()));
            double x = r*std::cos(theta), y = r*std::sin(theta);
            Point_3 P = Point_3( C.x() + u.x()*x + v.x()*y,
                                C.y() + u.y()*x + v.y()*y,
                                C.z() + u.z()*x + v.z()*y );
            sm.point(verts[i]) = P;
        }
    }
}


void CgalMeshBuilder::catmullClarkRefineNoSmooth(SurfaceMesh& sm)
{
    // 0) precondition: we only refine quads; others are left as-is
    // Take snapshots because we’ll mutate topology.
    std::vector<SurfaceMesh::Face_index> faces;
    faces.reserve(sm.number_of_faces());
    for (auto f : sm.faces()) if (!sm.is_removed(f)) faces.push_back(f);

    // 1) For every ORIGINAL edge, create/reuse its midpoint vertex.
    //     Store in an edge property so both adjacent faces share it.
    auto edge_mid =
        sm.add_property_map<SurfaceMesh::Edge_index, SurfaceMesh::Vertex_index>("e:mid", SurfaceMesh::null_vertex()).first;

    for (auto e : sm.edges()) if (!sm.is_removed(e)) {
            auto h  = halfedge(e, sm);
            auto v0 = source(h, sm);
            auto v1 = target(h, sm);
            const Point_3 p0 = sm.point(v0);
            const Point_3 p1 = sm.point(v1);
            edge_mid[e] = sm.add_vertex( midpoint(p0, p1) );
        }

    // 2) For every ORIGINAL quad face: compute its face-point, remove it,
    //    then add the 4 child quads using the stored midpoints.
    for (auto f : faces) {
        if (f == SurfaceMesh::null_face() || sm.is_removed(f)) continue;

        // Gather ring (expect 4 verts); skip non-quads.
        std::vector<SurfaceMesh::Vertex_index> ring;
        std::vector<SurfaceMesh::Edge_index>   edges;
        ring.reserve(4); edges.reserve(4);

        auto h0 = sm.halfedge(f), h = h0;
        do {
            ring.push_back( target(h, sm) );
            edges.push_back( edge(h, sm) );
            h = next(h, sm);
        } while (h != h0);

        if (ring.size() != 4) continue; // leave untouched

        const Point_3 p00 = sm.point(ring[0]);
        const Point_3 p10 = sm.point(ring[1]);
        const Point_3 p11 = sm.point(ring[2]);
        const Point_3 p01 = sm.point(ring[3]);

        // face point (centroid)
        SurfaceMesh::Vertex_index vf = sm.add_vertex( centroid4(p00,p10,p11,p01) );

        // midpoints (shared via edge property)
        SurfaceMesh::Vertex_index m01 = edge_mid[edges[0]]; // between ring[0]-ring[1]
        SurfaceMesh::Vertex_index m12 = edge_mid[edges[1]];
        SurfaceMesh::Vertex_index m23 = edge_mid[edges[2]];
        SurfaceMesh::Vertex_index m30 = edge_mid[edges[3]];

        // Remove the original face with Euler op
        CGAL::Euler::remove_face(sm.halfedge(f), sm);

        // Reconstruct 4 quads around the face point (keep CCW orientation)
        // (v0, m01, vf, m30), (v1, m12, vf, m01), (v2, m23, vf, m12), (v3, m30, vf, m23)
        add_quad_ccw(sm, ring[0], m01, vf, m30);
        add_quad_ccw(sm, ring[1], m12, vf, m01);
        add_quad_ccw(sm, ring[2], m23, vf, m12);
        add_quad_ccw(sm, ring[3], m30, vf, m23);
    }

    // 3) Clean dangling vertices created by removals (none of the old ones should be isolated,
    //    but be safe if some faces were not quads).
    {
        std::vector<SurfaceMesh::Vertex_index> dead;
        for (auto v : sm.vertices())
            if (sm.halfedge(v) == SurfaceMesh::null_halfedge())
                dead.push_back(v);
        for (auto v : dead) sm.remove_vertex(v);
    }
    sm.collect_garbage();
}



static inline uint64_t edge_key(uint32_t a, uint32_t b) {
    if (a>b) std::swap(a,b);
    return (uint64_t(a) << 32) | uint64_t(b);
}

void CgalMeshBuilder::catmullClarkRefine_NoInterp(SurfaceMesh& sm, bool keep_borders)
{
    // --- 1) Tag original vertices and store their positions/ids
    auto v_old   = sm.add_property_map<SurfaceMesh::Vertex_index, bool>("v:is_old", false).first;
    auto v_id    = sm.add_property_map<SurfaceMesh::Vertex_index, uint32_t>("v:orig_id", 0).first;
    auto v_pos   = sm.add_property_map<SurfaceMesh::Vertex_index, Point_3>("v:old_pos", Point_3(0,0,0)).first;

    uint32_t next_id = 0;
    for (auto v : sm.vertices()) if (!sm.is_removed(v)) {
            v_old[v] = true;
            v_id[v]  = next_id++;
            v_pos[v] = sm.point(v);
        }

    // Store the set of original edges (by original ids)
    std::unordered_set<uint64_t> orig_edges;
    orig_edges.reserve(sm.number_of_edges()*2+1);
    for (auto e : sm.edges()) if (!sm.is_removed(e)) {
            auto h  = halfedge(e, sm);
            auto s  = source(h, sm);
            auto t  = target(h, sm);
            orig_edges.insert(edge_key(v_id[s], v_id[t]));
        }

    // Optional: constrain borders if you want them to remain straight lines
    auto constrained = sm.add_property_map<SurfaceMesh::Edge_index, bool>("e:constrained", false).first;
    if (keep_borders) {
        for (auto e : sm.edges()) if (is_border(e, sm)) constrained[e] = true;
    }

    // --- 2) One Catmull–Clark iteration (does topology correctly)
    S3::CatmullClark_subdivision(
        sm,
        CGAL::parameters::number_of_iterations(1)
            .vertex_point_map(get(CGAL::vertex_point, sm))
            .edge_is_constrained_map(constrained)
        );

    // --- 3) Reposition: old verts → original; new verts → mid/centroid of original neighbors
    // We can recognize "new" vertices as those without v_old=true
    for (auto v : sm.vertices()) if (!sm.is_removed(v)) {
            if (v_old[v]) {
                sm.point(v) = v_pos[v];              // restore
                continue;
            }

            // gather adjacent ORIGINAL vertices around v
            std::vector<SurfaceMesh::Vertex_index> adj_old;
            for (auto h : CGAL::halfedges_around_target(v, sm)) {
                auto u = source(h, sm);
                if (v_old[u]) adj_old.push_back(u);
            }

            // classify: edge-vertex if exactly two original neighbors that were an original edge
            if (adj_old.size() == 2) {
                uint32_t a = v_id[adj_old[0]], b = v_id[adj_old[1]];
                if (orig_edges.count(edge_key(a,b))) {
                    const Point_3 &pa = v_pos[adj_old[0]], &pb = v_pos[adj_old[1]];
                    sm.point(v) = Point_3( (pa.x()+pb.x())*0.5,
                                          (pa.y()+pb.y())*0.5,
                                          (pa.z()+pb.z())*0.5 );
                    continue;
                }
            }

            // otherwise treat as face-vertex: centroid of all original neighbors (usually 4)
            double x=0,y=0,z=0; int n=0;
            for (auto u : adj_old) { const auto& p = v_pos[u]; x+=p.x(); y+=p.y(); z+=p.z(); ++n; }
            if (n>0) sm.point(v) = Point_3(x/n, y/n, z/n);
            // if n==0 (shouldn't happen), leave the CC position
        }

    // Note: property maps remain; OK to keep or remove them later if you want.
}

static inline glm::vec3 to_glm3(const Point_3& p) {
    return glm::vec3((float)p.x(), (float)p.y(), (float)p.z());
}
static inline glm::vec3 to_glm3(const Vector_3& v) {
    return glm::vec3((float)v.x(), (float)v.y(), (float)v.z());
}
static inline glm::vec3 safe_norm(const glm::vec3& v) {
    float l2 = glm::dot(v,v); return (l2>0.f)? v/std::sqrt(l2) : glm::vec3(1,0,0);
}
static glm::vec3 hsv2rgb(float h, float s, float v) {
    h = h - std::floor(h);
    float c = v*s, x = c*(1.f - std::fabs(std::fmod(h*6.f,2.f)-1.f)), m = v-c;
    float r=0,g=0,b=0;
    if      (0.f<=h && h<1.f/6.f) { r=c; g=x; }
    else if (h<2.f/6.f) { r=x; g=c; }
    else if (h<3.f/6.f) { g=c; b=x; }
    else if (h<4.f/6.f) { g=x; b=c; }
    else if (h<5.f/6.f) { r=x; b=c; }
    else                { r=c; b=x; }
    return glm::vec3(r+m,g+m,b+m);
}

// цвет по id грани – равномерный по кругу оттенков
static glm::vec3 face_color(uint32_t fid) {
    float h = std::fmod(fid * 0.61803398875f, 1.0f); // золотое сечение
    return hsv2rgb(h, 0.65f, 0.95f);
}

// ---------- helpers: hash -> HSV -> RGB ----------
static inline uint32_t hash_u32(uint64_t x) {
    // простой стабильный хэш PCG / wang
    x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
    x ^= x >> 33;
    return static_cast<uint32_t>(x);
}


// Белый для границы, иначе — «небелый» детерминированный цвет
static inline glm::vec3 pick_color(bool is_border, uint64_t seed) {
    if (is_border) return glm::vec3(0.98f); // чуть-чуть не 1.0 чтобы точно было видно
    uint32_t h = hash_u32(seed);
    float hue = (h & 0xFFFF) / 65535.0f;
    // фиксируем насыщенность/яркость так, чтобы точно не получить белый
    return hsv2rgb(hue, /*S*/0.65f, /*V*/0.95f);
}

static inline void draw_halfedge_arrow(
    const SurfaceMesh& sm,
    SurfaceMesh::Halfedge_index h,
    const glm::vec3& N,        // нормаль для смещения/стрелки
    const glm::vec3& col,
    float inset, float headRel,
    std::vector<Vertex>& out)
{
    const auto va = source(h, sm);
    const auto vb = target(h, sm);
    glm::vec3 A = to_glm3(sm.point(va));
    glm::vec3 B = to_glm3(sm.point(vb));

    glm::vec3 M   = 0.5f * (A + B);
    glm::vec3 dir = glm::normalize(B - A);
    glm::vec3 side= glm::normalize(glm::cross(N, dir));

    float L = glm::length(B - A);
    float insetAbs = inset * L;

    // лёгкое утапливание «внутрь»
    glm::vec3 A2 = glm::mix(A, B, 0.15f) + glm::normalize(glm::cross(dir, side)) * insetAbs;
    glm::vec3 B2 = glm::mix(A, B, 0.85f) + glm::normalize(glm::cross(dir, side)) * insetAbs;

    auto push = [&](const glm::vec3& P, const glm::vec3& Q){
        out.push_back({ glm::vec4(P,1), glm::vec4(N,0), glm::vec4(col,1) });
        out.push_back({ glm::vec4(Q,1), glm::vec4(N,0), glm::vec4(col,1) });
    };

    // ствол
    push(A2, B2);

    // наконечник
    float hl = std::min(L*0.25f, L*headRel);
    glm::vec3 base = B2 - dir * hl;
    glm::vec3 Lft  = base + side * (0.6f*hl);
    glm::vec3 Rgt  = base - side * (0.6f*hl);
    push(B2, Lft);
    push(B2, Rgt);
}

void CgalMeshBuilder::buildHalfedgeArrows(
    const SurfaceMesh& sm,
    std::vector<Vertex>& outLineVerts,
    float inset,
    float headRel,
    bool draw_border /* = true */)
{
    outLineVerts.clear();

    // --- 1) по всем граням (как у вас) ---
    uint32_t fid = 0;
    auto fuid_opt = sm.property_map<SurfaceMesh::Face_index, std::uint64_t>("f:uid");
    for (auto f : sm.faces()) {
        if (sm.is_removed(f)) { ++fid; continue; }

        // нормаль и цвет грани
        std::vector<SurfaceMesh::Vertex_index> ring;
        auto h0 = sm.halfedge(f), h = h0;
        do { ring.push_back(target(h, sm)); h = next(h, sm); } while (h != h0);
        if (ring.size() < 3) { ++fid; continue; }

        // Newell normal
        Point_3 C(0,0,0); for (auto v : ring){ const auto& p=sm.point(v);
            C = Point_3(C.x()+p.x(), C.y()+p.y(), C.z()+p.z()); }
        C = Point_3(C.x()/ring.size(), C.y()/ring.size(), C.z()/ring.size());

        Vector_3 nCG(0,0,0);
        for (size_t i=0;i<ring.size();++i){
            const auto& pi = sm.point(ring[i]);
            const auto& pj = sm.point(ring[(i+1)%ring.size()]);
            nCG = nCG + CGAL::cross_product(pj - C, pi - C);
        }
        glm::vec3 N   = safe_norm(to_glm3(nCG));
        //glm::vec3 col = face_color(fid++);
        uint64_t seed = fuid_opt ? (*fuid_opt)[f] : fid;

        // стрелки вдоль halfedge грани
        h = h0;
        do {
            glm::vec3 col = pick_color(/*is_border=*/false, seed);
            draw_halfedge_arrow(sm, h, N, col, inset, headRel, outLineVerts);
            h = next(h, sm);
        } while (h != h0);
    }

    if (!draw_border) return;

    // --- 2) дополнительно: по граничным рёбрам отверстий ---
    // рисуем ОДНУ стрелку на каждое граничное ребро, ориентированную вдоль border halfedge
    for (auto e : sm.edges()) {
        auto h  = halfedge(e, sm);
        auto ho = opposite(h, sm);

        SurfaceMesh::Halfedge_index hb = SurfaceMesh::null_halfedge();
        SurfaceMesh::Halfedge_index hi = SurfaceMesh::null_halfedge(); // внутренний (у которого есть face)

        if (CGAL::is_border(h, sm) && !CGAL::is_border(ho, sm)) { hb = h;  hi = ho; }
        else if (CGAL::is_border(ho, sm) && !CGAL::is_border(h, sm)) { hb = ho; hi = h; }
        else continue; // не граница (две стороны имеют грани)

        // нормаль берём из соседней (не border) грани, чтобы смещать «внутрь» полости
        glm::vec3 N = glm::vec3(0,0,1);
        {
            // вычислим нормаль как у face(hi)
            std::vector<SurfaceMesh::Vertex_index> ring;
            auto hf = sm.halfedge(face(hi, sm));
            auto t  = hf;
            Point_3 C(0,0,0);
            size_t  cnt=0;
            do { auto v = target(t, sm); ring.push_back(v);
                const auto& p=sm.point(v);
                C = Point_3(C.x()+p.x(), C.y()+p.y(), C.z()+p.z());
                ++cnt; t = next(t, sm);
            } while (t != hf);
            C = Point_3(C.x()/cnt, C.y()/cnt, C.z()/cnt);

            Vector_3 nCG(0,0,0);
            for (size_t i=0;i<ring.size();++i){
                const auto& pi = sm.point(ring[i]);
                const auto& pj = sm.point(ring[(i+1)%ring.size()]);
                nCG = nCG + CGAL::cross_product(pj - C, pi - C);
            }
            N = safe_norm(to_glm3(nCG));
        }

        glm::vec3 col = pick_color(/*is_border=*/true, /*seed*/0);
        draw_halfedge_arrow(sm, hb, N, col, inset, headRel, outLineVerts);
    }
}



// CgalMeshBuilder.cpp  (add implementation)
#include <CGAL/Polygon_mesh_processing/orientation.h>
namespace PMP = CGAL::Polygon_mesh_processing;

void CgalMeshBuilder::buildBoxGrid(SurfaceMesh& sm,
                                   int nx, int ny, int nz,
                                   double cellSize)
{
    sm.clear();
    nx = std::max(1, nx);
    ny = std::max(1, ny);
    nz = std::max(1, nz);

    // Box centered at origin
    const double sx = nx * cellSize;
    const double sy = ny * cellSize;
    const double sz = nz * cellSize;
    const double x0 = -0.5 * sx, y0 = -0.5 * sy, z0 = -0.5 * sz;

    // Allocate (nx+1)*(ny+1)*(nz+1) vertices on the grid
    const int VX = nx + 1, VY = ny + 1, VZ = nz + 1;
    auto idx = [=](int i, int j, int k) { return (k*VY + j)*VX + i; };

    std::vector<SurfaceMesh::Vertex_index> V(VX*VY*VZ);

    for (int k = 0; k < VZ; ++k) {
        const double z = z0 + k * cellSize;
        for (int j = 0; j < VY; ++j) {
            const double y = y0 + j * cellSize;
            for (int i = 0; i < VX; ++i) {
                const double x = x0 + i * cellSize;
                V[idx(i,j,k)] = sm.add_vertex(Point_3(x,y,z));
            }
        }
    }

    auto Vid = [&](int i,int j,int k)->SurfaceMesh::Vertex_index& { return V[idx(i,j,k)]; };

    // Emit boundary quads with outward CCW orientation
    // Z- plane (k = 0)
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            add_quad_ccw(sm,
                         Vid(i,  j,  0), Vid(i+1,j,  0),
                         Vid(i+1,j+1,0), Vid(i,  j+1,0));

    // Z+ plane (k = nz)
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i)
            add_quad_ccw(sm,
                         Vid(i,  j,  nz), Vid(i+1,j,  nz),
                         Vid(i+1,j+1,nz), Vid(i,  j+1,nz));

    // Y- plane (j = 0)
    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i)
            add_quad_ccw(sm,
                         Vid(i,  0,  k),  Vid(i,  0,  k+1),
                         Vid(i+1,0,  k+1),Vid(i+1,0,  k));

    // Y+ plane (j = ny)
    for (int k = 0; k < nz; ++k)
        for (int i = 0; i < nx; ++i)
            add_quad_ccw(sm,
                         Vid(i,  ny, k),  Vid(i+1,ny,k),
                         Vid(i+1,ny,k+1), Vid(i,  ny,k+1));

    // X- plane (i = 0)
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            add_quad_ccw(sm,
                         Vid(0,  j,  k),  Vid(0,  j+1,k),
                         Vid(0,  j+1,k+1),Vid(0,  j,  k+1));

    // X+ plane (i = nx)
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            add_quad_ccw(sm,
                         Vid(nx, j,  k),  Vid(nx, j,  k+1),
                         Vid(nx, j+1,k+1),Vid(nx, j+1,k));

    sm.collect_garbage();

    // Safety: ensure outward orientation (usually already correct)
    if (!PMP::is_outward_oriented(sm))
        PMP::orient_to_bound_a_volume(sm);
}
