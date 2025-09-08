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


#include <glm/glm.hpp>
#include <cmath>
#include <unordered_map>

using V  = SM::Vertex_index;
using H  = SM::Halfedge_index;
using F  = SM::Face_index;
using E = SM::Edge_index;
using P =SM::Point;

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


static inline glm::vec3 unit3_2(const glm::vec3& v) {
    float l2 = glm::dot(v,v); return (l2>0.f)? v/std::sqrt(l2) : glm::vec3(0,0,1);
}

// ===== UID helpers ==============================================================


static inline bool valid_v(const SM& sm, V v){
    return v != SM::null_vertex()
    && (std::size_t)v.idx() < sm.number_of_vertices()
        && !sm.is_removed(v);
}
static inline bool valid_f(const SM& sm, F f){
    return f != SM::null_face()
    && (std::size_t)f.idx() < sm.number_of_faces()
        && !sm.is_removed(f);
}
static inline bool valid_h(const SM& sm, H h){
    return h != SM::null_halfedge()
    && (std::size_t)h.idx() < sm.number_of_halfedges()
        && !sm.is_removed(h);
}
static inline bool valid_e(const SM& sm, SM::Edge_index e){
    return e != SM::null_edge()
    && (std::size_t)e.idx() < sm.number_of_edges()
        && !sm.is_removed(e);
}


// Если у вершины нет halfedge — проставляем любой граничный вокруг неё.
// Безопасно вызывать после add_edge(...) / сшивок.
static inline void fix_vertex_halfedge_safe(SM& sm, V v) {
    if (!valid_v(sm, v)) return;
    if (sm.halfedge(v) != SM::null_halfedge()) return;

    // Попытаться выбрать border-halfedge (CGAL сам найдёт, если есть)
    if (sm.set_vertex_halfedge_to_border_halfedge(v))
        return;

    // Фоллбэк: найдём любой инцидентный полурёбер и поставим его
    // (range вокруг target/source может быть пустым, если старт не задан,
    // поэтому попробуем напрямую через смежные вершины не надо — см. перегрузку ниже).
    for (H h : CGAL::halfedges_around_target(v, sm)) { sm.set_halfedge(v, h); return; }
    for (H h : CGAL::halfedges_around_source(v, sm)) { sm.set_halfedge(v, sm.opposite(h)); return; }

}


static inline void fix_vertex_halfedge_safe(SM& sm, V v, H prefer) {
    if (!valid_v(sm, v)) return;
    if (sm.halfedge(v) != SM::null_halfedge()) return;

    if (prefer != SM::null_halfedge()
        && (sm.source(prefer)==v || sm.target(prefer)==v))
    { sm.set_halfedge(v, prefer); return; }

    if (sm.set_vertex_halfedge_to_border_halfedge(v)) return;

    for (H h : CGAL::halfedges_around_target(v, sm)) { sm.set_halfedge(v, h); return; }
    for (H h : CGAL::halfedges_around_source(v, sm)) { sm.set_halfedge(v, h); return; }
}


struct FacePlan {
    std::uint64_t fuid = 0;
    std::vector<std::uint64_t> ring_vuid;  // CCW по исходной грани
    std::vector<char>          wasBorder;  // по каждому ребру (i->i+1) на момент слепка
};

// ---------- UID инфраструктура ----------
static inline void ensure_uid_maps_and_assign_all(SM& sm,
                                                  SM::Property_map<V, std::uint64_t>& vuid,
                                                  SM::Property_map<F, std::uint64_t>& fuid,
                                                  std::uint64_t& next_vuid,
                                                  std::uint64_t& next_fuid)
{
    // создать property map'ы, если их нет
    vuid = sm.add_property_map<V, std::uint64_t>("v:uid64", 0).first;
    fuid = sm.add_property_map<F, std::uint64_t>("f:uid64", 0).first;

    next_vuid = 1;
    next_fuid = 1;

    // пронумеруем существующие вершины/грани и найдём next_*
    for (auto v : sm.vertices()) {
        auto& u = vuid[v];
        if (u == 0) u = next_vuid++;
        else next_vuid = std::max(next_vuid, u + 1);
    }
    for (auto f : sm.faces()) {
        auto& u = fuid[f];
        if (u == 0) u = next_fuid++;
        else next_fuid = std::max(next_fuid, u + 1);
    }
}

static inline void assign_uid_vertex_new(V v,
                                         SM::Property_map<V, std::uint64_t>& vuid,
                                         std::uint64_t& next_vuid)
{
    vuid[v] = (next_vuid ? next_vuid++ : 1); // на случай если забыли инициализировать
}

static inline void assign_uid_face_new(F f,
                                       SM::Property_map<F, std::uint64_t>& fuid,
                                       std::uint64_t& next_fuid)
{
    fuid[f] = (next_fuid ? next_fuid++ : 1);
}

// быстрые словари UID→дескриптор
static inline void rebuild_vertex_uid_map(const SM& sm,
                                          const SM::Property_map<V, std::uint64_t>& vuid,
                                          std::unordered_map<std::uint64_t, V>& out)
{
    out.clear(); out.reserve(sm.number_of_vertices()*2);
    for (auto v : sm.vertices()) out[vuid[v]] = v;
}
static inline void rebuild_face_uid_map(const SM& sm,
                                        const SM::Property_map<F, std::uint64_t>& fuid,
                                        std::unordered_map<std::uint64_t, F>& out)
{
    out.clear(); out.reserve(sm.number_of_faces()*2);
    for (auto f : sm.faces()) out[fuid[f]] = f;
}

// ---------- геометрия ----------
static inline Point_3 centroid_points(const std::vector<Point_3>& R){
    double x=0,y=0,z=0; for (auto& p: R){ x+=p.x(); y+=p.y(); z+=p.z(); }
    const double inv = (R.empty()? 0.0 : 1.0/double(R.size()));
    return Point_3(x*inv,y*inv,z*inv);
}
static inline Vector_3 newell_normal(const std::vector<Point_3>& P){
    Vector_3 n(0,0,0); const size_t m=P.size();
    for (size_t i=0;i<m;++i){
        const auto& a=P[i]; const auto& b=P[(i+1)%m];
        n = n + Vector_3( (a.y()-b.y())*(a.z()+b.z()),
                         (a.z()-b.z())*(a.x()+b.x()),
                         (a.x()-b.x())*(a.y()+b.y()) );
    }
    const double L = std::sqrt(n.squared_length());
    return (L>1e-12)? n/L : Vector_3(0,0,1);
}
static inline Point_3 mesh_centroid(const SM& sm){
    double x=0,y=0,z=0; std::size_t n=0;
    for (auto v : sm.vertices()){ const auto& p=sm.point(v); x+=p.x(); y+=p.y(); z+=p.z(); ++n; }
    if (!n) return Point_3(0,0,0);
    const double inv = 1.0/double(n); return Point_3(x*inv,y*inv,z*inv);
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
    using SM = SurfaceMesh;
    using V  = SM::Vertex_index;
    using H  = SM::Halfedge_index;

    auto next_around_target = [&](H h)->H {
        if (h == SM::null_halfedge()) return h;
        H n = sm.next(h);                  // v -> w
        if (n == SM::null_halfedge()) return SM::null_halfedge();
        return sm.opposite(n);             // w -> v  (снова target == v)
    };

    // кандидаты — только бордер-вершины
    std::vector<V> cand;
    cand.reserve(sm.number_of_vertices());
    for (V v : sm.vertices())
        if (!sm.is_removed(v) && CGAL::is_border(v, sm))
            cand.push_back(v);

    for (V v : cand)
    {
        // будем резать, пока вокруг v больше одной бордер-группы
        while (true)
        {
            // 1) кольцо входящих к v
            std::vector<H> ring;
            ring.reserve(16);
            for (H h : CGAL::halfedges_around_target(v, sm))
                ring.push_back(h);
            const int n = (int)ring.size();
            if (n <= 1) break;

            auto isB = [&](int i){ return CGAL::is_border(ring[i], sm); };

            // 2) собрать подряд идущие бордер-участки (учитывая wrap-around)
            std::vector<std::pair<int,int>> groups; // [b,e], обе границы включительно
            groups.reserve(4);

            // найдём старт с НЕбордерного, чтобы не рвать группу на границе
            int start = 0;
            for (; start<n && isB(start); ++start) {}
            if (start == n) {
                // весь веер — бордер; это одна группа, «поцелуев» нет
                break;
            }

            int i = start;
            do {
                // пропускаем небoрдеры
                while (!isB(i)) { i = (i+1)%n; if (i==start) break; }
                if (!isB(i)) break;             // вернулись к старту

                int b = i;                       // начало бордер-полосы
                do { i = (i+1)%n; } while (isB(i));
                int e = (i + n - 1) % n;         // конец бордер-полосы
                groups.emplace_back(b, e);
            } while (i != start);

            if (groups.size() <= 1) break;       // уже нет «поцелуев»

            // 3) режем между двумя соседними группами
            const int b2 = groups[1].first;      // первый halfedge второй группы  (border)
            const int e1 = groups[0].second;     // последний halfedge первой группы (border)

            H h1 = ring[b2];                     // граница со стороны 2-й группы
            H h2 = ring[(e1 + 1) % n];           // первый после 1-й группы (не border)

            // страховки пред-условий CGAL::Euler::split_vertex
            if (h1 == h2) {                      // на всякий
                h2 = next_around_target(h1);
                if (h1 == h2) break;
            }
            if (sm.target(h1) != v || sm.target(h2) != v)
                break;

            // 4) сплитим: сектор между h1 и h2 (по кругу target(v)) уедет в новую вершину
            const auto Pv = sm.point(v);         // запомним позицию исходной вершины
            CGAL::Euler::split_vertex(h1, h2, sm);

            // В Surface_mesh split_vertex НЕ копирует point-property.
            // После операции h1 и h2 теперь имеют разный target: один старый v, другой — новая вершина.
            V va = sm.target(h1);
            V vb = sm.target(h2);
            sm.point(va) = Pv;                   // обе получают ту же позицию, что и исходная
            sm.point(vb) = Pv;

            // продолжаем цикл: переменная v оставляем прежней — это «старая» вершина;
            // если остались ещё лишние бордер-группы вокруг неё, следующая итерация их найдёт.
        }
    }

    sm.collect_garbage();
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


// ---------------- Stage 2 ----------------
std::vector<SurfaceMesh::Face_index>
CgalMeshBuilder::selectFaceByIndex(const SurfaceMesh& sm, uint32_t indexFace) {
    uint32_t counter =0u;
    std::vector<SurfaceMesh::Face_index> out;
    for (auto f : sm.faces()) if (!sm.is_removed(f)) {
            if (counter ==indexFace)
            {
                out.push_back(f);
                break;
            }
            counter++;
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
    //    PMP::duplicate_non_manifold_vertices(sm); // splits “kissing” corners
       sm.collect_garbage();
    }
}

#include <CGAL/Polygon_mesh_processing/stitch_borders.h>

void CgalMeshBuilder::cleanup_after_deletions(SurfaceMesh& sm) {
    namespace PMP = CGAL::Polygon_mesh_processing;

    sm.collect_garbage();
    // 1) У всех вершин гарантированно выставить какой-то halfedge
    for (auto v : sm.vertices())
        fix_vertex_halfedge_safe(sm, v);

    std::size_t nb = PMP::stitch_borders(sm);
    (void)nb; // чтобы компилятор не ворчал
    sm.collect_garbage();

    // 2) Раздублировать неманифолдные граничные вершины (kissing)
    PMP::duplicate_non_manifold_vertices(sm); // CGAL >= 5.6
    split_kissing_border_vertices(sm);        // твой fallback

    // 3) Подчистить мусор
   // PMP::remove_degenerate_faces(sm);
   // remove_isolated_vertices_safe(sm);


   // PMP::remove_degenerate_faces(sm);
    // 4) Компактим индексы (ИНВАЛИДИРУЕТ все старые дескрипторы!)
    sm.collect_garbage();
}

// Найти halfedge u->v без использования sm.halfedge(u) / sm.halfedge(u,v).
static inline H find_halfedge_uv_scan(SM& sm, V u, V v) {
    if (!valid_v(sm,u) || !valid_v(sm,v) || u==v) return SM::null_halfedge();
    for (H h : sm.halfedges()) {
        if (sm.is_removed(h)) continue;
        if (source(h, sm) == u && target(h, sm) == v) return h;
    }
    return SM::null_halfedge();
}

// Безопасная версия "дать бордерный полурёбер u->v, иначе создать ребро".
static inline H get_or_make_border_h_safe(SM& sm, V u, V v) {
    if (!valid_v(sm,u) || !valid_v(sm,v) || u==v) return SM::null_halfedge();

    // 1) Пытаемся найти существующее ребро сканированием
    if (H h = find_halfedge_uv_scan(sm, u, v); h != SM::null_halfedge()) {
        if (sm.face(h) == SM::null_face()) return h;                 // свободна сторона u->v
        H ho = sm.opposite(h);
        if (ho != SM::null_halfedge() && sm.face(ho) == SM::null_face()) return ho; // v->u свободна
        return SM::null_halfedge();                                   // обе стороны заняты
    }
    if (H h = find_halfedge_uv_scan(sm, v, u); h != SM::null_halfedge()) {
        if (sm.face(h) == SM::null_face()) return h;                 // уже есть v->u, но свободна
        H ho = sm.opposite(h);
        if (ho != SM::null_halfedge() && sm.face(ho) == SM::null_face()) return ho; // обратная свободна
        return SM::null_halfedge();
    }

    // 2) Ребра нет — создаём (вернётся u->v)
    H hnew = sm.add_edge(u, v);
    // Сразу проставим опорные halfedge'ы вершинам, чтобы не было "пустых" указателей
    fix_vertex_halfedge_safe(sm, u, hnew);
    fix_vertex_halfedge_safe(sm, v, sm.opposite(hnew));
    return hnew;
}

static inline F attach_face_from_ring4(SM& sm, H h0, H h1, H h2, H h3){
    H ring[4] = {h0,h1,h2,h3};
    for (int i=0;i<4;++i){
        if (ring[i]==SM::null_halfedge())
            return SM::null_face();
        if (sm.face(ring[i]) != SM::null_face())
            return SM::null_face();
        if (sm.target(ring[i]) != sm.source(ring[(i+1)&3]))
            return SM::null_face();
    }

    F nf = sm.add_face();
    if (nf == SM::null_face())
        return nf;

    // 1) цикл по НОВОЙ грани
    for (int i=0;i<4;++i){
        sm.set_face(ring[i], nf);
        sm.set_next(ring[i], ring[(i+1)&3]);
    }
    sm.set_halfedge(nf, ring[0]);

    // 2) ОБЯЗАТЕЛЬНО: проложить бордер-цикл на обратной стороне
    //    порядок противоположный: opp(h_{i+1}) -> opp(h_i)
    for (int i=0;i<4;++i){
        H o_cur  = sm.opposite(ring[i]);           // ... -> target(ring[i])
        H o_next = sm.opposite(ring[(i+1)&3]);     // ... -> source(ring[(i+1)&3]) == target(ring[i])
        if (sm.face(o_cur)  == SM::null_face() &&
            sm.face(o_next) == SM::null_face())
        {
            sm.set_next(o_next, o_cur);            // border "идёт" в обратном направлении
        }
    }

    // 3) Подстраховка: у вершин должен стоять какой-то halfedge
    //fix_vertex_halfedge_safe(sm, sm.source(ring[0]));
    //fix_vertex_halfedge_safe(sm, sm.target(ring[0]));
    //fix_vertex_halfedge_safe(sm, sm.source(ring[2]));
    //fix_vertex_halfedge_safe(sm, sm.target(ring[2]));
    return nf;
}



// ========== ПОДГОТОВКА СЕРИИ: слепок по UID ==========
static inline void build_face_plans_snapshot(
    SM& sm,
    const std::vector<F>& faces_in,
    const SM::Property_map<V,std::uint64_t>& vuid,
    const SM::Property_map<F,std::uint64_t>& fuid,
    std::vector<FacePlan>& outPlans)
{
    outPlans.clear(); outPlans.reserve(faces_in.size());

    for (F f : faces_in){
        if (f == SM::null_face() || sm.is_removed(f)) continue;

        // кольцо CCW
        std::vector<std::uint64_t> ring_v;
        std::vector<char> wasBorder;

        H h0 = sm.halfedge(f);
        if (h0 == SM::null_halfedge()) continue;
        H h = h0;

        std::unordered_set<V> seen;
        do{
            V v = source(h, sm);
            if (v==SM::null_vertex() || sm.is_removed(v) || sm.is_removed(h)) { ring_v.clear(); break; }
            if (!seen.insert(v).second) { ring_v.clear(); break; } // не простой цикл

            ring_v.push_back( vuid[v] );
            bool is_border_initial = ( face(opposite(h, sm), sm) == SM::null_face() );
            wasBorder.push_back( is_border_initial ? 1 : 0 );

            h = next(h, sm);
        } while(h != h0);

        if (ring_v.size() < 3) continue;

        FacePlan fp;
        fp.fuid      = fuid[f];
        fp.ring_vuid = std::move(ring_v);
        fp.wasBorder = std::move(wasBorder);
        outPlans.push_back(std::move(fp));
    }
}

// вернуть ИМЕННО border halfedge u->v (или null, если эта сторона занята)
// создаст ребро, если его ещё нет; при необходимости перевернёт add_edge результат
// строго u->v, только border; безопасен, когда у вершины нет halfedge
// Вернуть ИМЕННО border halfedge u->v.
// НИКОГДА не зовёт CGAL::halfedge(u,v,sm), если у u или v нет halfedge.
// Ищем/создаём border halfedge u->v, не вызывая sm.halfedge(u).
static inline H get_or_make_border_h_uv(SM& sm, V u, V v) {
    if (!valid_v(sm,u) || !valid_v(sm,v) || u == v) return SM::null_halfedge();

    // 1) Пытаемся СРАЗУ создать ребро (без каких-либо проверок существования).
    H h = sm.add_edge(u, v);                    // создаёт 2 border halfedge'а
    if (h != SM::null_halfedge()) {
        if (sm.source(h) != u) h = sm.opposite(h); // нормализуем в u->v
        fix_vertex_halfedge_safe(sm, u);           // мягко проставим vertex->halfedge
        fix_vertex_halfedge_safe(sm, v);
        return h;                                  // гарантированно border u->v
    }

    // 2) Если add_edge вернул null, ребро уже существует.
    //    Теперь можно аккуратно попробовать получить половинки *с направлением*.
    //    Здесь уже шансы, что у вершин есть halfedge, намного выше.
    H hu = sm.halfedge(u, v);                     // НЕ используем свободную CGAL::halfedge!
    if (hu != SM::null_halfedge() && sm.face(hu) == SM::null_face()) return hu;

    H hv = sm.halfedge(v, u);
    if (hv != SM::null_halfedge()) {
        H huv = sm.opposite(hv);                  // это u->v
        if (sm.face(huv) == SM::null_face()) return huv;
    }

    return SM::null_halfedge();                   // обе стороны заняты
}

static inline F attach_face_from_ringN(SM& sm, const std::vector<H>& ring){
    const size_t n = ring.size();
    if (n < 3) return SM::null_face();
    for (size_t i=0;i<n;++i){
        H h = ring[i];
        if (h==SM::null_halfedge()) return SM::null_face();
        if (sm.face(h) != SM::null_face()) return SM::null_face();
        if (sm.target(h) != sm.source(ring[(i+1)%n])) return SM::null_face();
    }

    F nf = sm.add_face(); if (nf == SM::null_face()) return nf;

    // грань
    for (size_t i=0;i<n;++i){
        sm.set_face(ring[i], nf);
        sm.set_next(ring[i], ring[(i+1)%n]);
    }
    sm.set_halfedge(nf, ring[0]);

    // бордер-кольцо на обратной стороне
    for (size_t i=0;i<n;++i){
        H o_cur  = sm.opposite(ring[i]);
        H o_next = sm.opposite(ring[(i+1)%n]);
        if (sm.face(o_cur)  == SM::null_face() &&
            sm.face(o_next) == SM::null_face())
        {
            sm.set_next(o_next, o_cur);
        }
    }

    // лёгкая починка указателей вершин
    for (size_t i=0;i<n;++i){
        fix_vertex_halfedge_safe(sm, sm.source(ring[i]));
        fix_vertex_halfedge_safe(sm, sm.target(ring[i]));
    }
    return nf;
}


// найти любой верхний бордерный halfedge среди рёбер top[i]—top[j]
static inline H find_top_border_start(SM& sm, const std::vector<V>& top){
    const size_t k = top.size();
    for (size_t i=0;i<k;++i){
        size_t j=(i+1)%k;
        auto pr = CGAL::halfedge(top[i], top[j], sm);
        if (pr.second && sm.face(pr.first) == SM::null_face()) return pr.first;
        pr = CGAL::halfedge(top[j], top[i], sm);
        if (pr.second && sm.face(pr.first) == SM::null_face()) return pr.first;
    }
    return SM::null_halfedge();
}



static inline F extrude_face_from_plan_uid(
    SM& sm,
    const FacePlan& plan,
    double distExtrude, double amountExtrude,
    const Point_3& meshC,
    SM::Property_map<V,std::uint64_t>& vuid,
    SM::Property_map<F,std::uint64_t>& /*fuid*/,
    std::uint64_t& next_vuid, std::uint64_t& /*next_fuid*/,
    std::unordered_map<std::uint64_t,V>& vByUid,
    std::unordered_map<std::uint64_t,F>& fByUid,
    ExtrudeLists* out)
{
    // --- 0) валидная грань ---
    auto itF = fByUid.find(plan.fuid);
    if (itF == fByUid.end()) return SM::null_face();
    F f = itF->second;
    if (!valid_f(sm, f))     return SM::null_face();

    // --- 1) кольцо грани: base[], baseP[], ringH[] (source(h)=base[i], target(h)=base[i+1]) ---
    std::vector<V> base; base.reserve(8);
    std::vector<Point_3> baseP; baseP.reserve(8);
    std::vector<H> ringH; ringH.reserve(8);

    H h0 = sm.halfedge(f); if (!valid_h(sm,h0)) return SM::null_face();
    H h  = h0;
    std::unordered_set<V> seen;
    do{
        if (!valid_h(sm,h)) return SM::null_face();
        V s = source(h, sm);
        if (!valid_v(sm,s) || !seen.insert(s).second) return SM::null_face();
        base.push_back(s);
        baseP.push_back(sm.point(s));
        ringH.push_back(h);
        h = next(h, sm);
    } while(h != h0);

    const size_t k = base.size();
    if (k < 3 || plan.ring_vuid.size() != k) return SM::null_face();

    // --- 2) верхнее кольцо ---
    const Point_3  c = centroid_points(baseP);
    Vector_3       n = newell_normal(baseP);
    if (CGAL::scalar_product(n, Vector_3(c.x()-meshC.x(), c.y()-meshC.y(), c.z()-meshC.z())) < 0) n = -n;

    const Point_3 cc( c.x()+n.x()*distExtrude,
                     c.y()+n.y()*distExtrude,
                     c.z()+n.z()*distExtrude );

    std::vector<V> top(k);
    for (size_t i=0;i<k;++i){
        const Vector_3 d( baseP[i].x()-c.x(), baseP[i].y()-c.y(), baseP[i].z()-c.z() );
        const Point_3  pt( cc.x()+amountExtrude*d.x(),
                         cc.y()+amountExtrude*d.y(),
                         cc.z()+amountExtrude*d.z() );
        V tv = sm.add_vertex(pt);
        assign_uid_vertex_new(tv, vuid, next_vuid);
        vByUid[vuid[tv]] = tv;
        top[i] = tv;
    }

    // --- 3) стены по исходным бордерам (низ = b->a = opposite(ringH[i])) ---
    for (size_t i=0;i<k;++i){
        if (!plan.wasBorder[i]) continue;
        const size_t j = (i+1)%k;

        H h_ba  = sm.opposite(ringH[i]);                 // низ b->a (бордер уже был)
        if (!valid_h(sm,h_ba) || sm.face(h_ba) != SM::null_face()) continue;

        H h_a_ta  = get_or_make_border_h_safe(sm, base[i], top[i]);
        H h_ta_tb = get_or_make_border_h_safe(sm, top[i], top[j]);
        H h_tb_b  = get_or_make_border_h_safe(sm, top[j], base[j]);
        if (!valid_h(sm,h_a_ta) || !valid_h(sm,h_ta_tb) || !valid_h(sm,h_tb_b)) continue;

        F fw = attach_face_from_ring4(sm, h_ba, h_a_ta, h_ta_tb, h_tb_b);
        if (out && fw != SM::null_face()) out->all.push_back(fw);
    }

    // --- 4) ПЕРЕД удалением: вернуть vertex->halfedge на контур удаляемой грани ---
    //for (size_t i=0;i<k;++i) sm.set_halfedge(base[i], ringH[i]);

    // ----- 5) удаляем базовую грань (ringH[i] станут border на стороне a->b) -----
    CGAL::Euler::remove_face(ringH[0], sm);
    fByUid.erase(plan.fuid);

    // (не обязательно, но полезно стабилизировать указатели у вершин)
    for (V v : base) fix_vertex_halfedge_safe(sm, v);

    // ----- 6) стены по остальным (бывшим внутренним) рёбрам -----
    for (size_t i = 0; i < k; ++i) {
        if (plan.wasBorder[i])
            continue;
        const size_t j = (i + 1) % k;

        // НИЗ БЕРЁМ РОВНО ТОТ ЖЕ halfedge, что был у удалённой грани
        H h_ab   = ringH[i];  // низ a->b, уже border после remove_face
        if (!valid_h(sm,h_ab) || sm.face(h_ab) != SM::null_face()) continue;

        H h_b_tb = get_or_make_border_h_uv(sm, base[j], top[j]);  // b -> tb
        H h_tb_ta= get_or_make_border_h_uv(sm, top[j],  top[i]);  // tb -> ta
        H h_ta_a = get_or_make_border_h_uv(sm, top[i],  base[i]); // ta -> a

        //auto why = [&](const char* tag, H h){
        //    std::cerr << tag << " idx=" << h.idx()
        //    << " border=" << (valid_h(sm,h) && sm.face(h)==SM::null_face())
        //    << " src=" << sm.source(h).idx()
        //    << " dst=" << sm.target(h).idx() << "\n";
        //};
        //why("h_ab", h_ab);
        //why("h_b_tb", h_b_tb);
        //why("h_tb_ta", h_tb_ta);
        //why("h_ta_a", h_ta_a);

        F fw = attach_face_from_ring4(sm, h_ab, h_b_tb, h_tb_ta, h_ta_a);
        if (fw != SM::null_face()) {
            // мягко «чинить» по одному halfedge на вершину — так следующие итерации стабильнее
            fix_vertex_halfedge_safe(sm, base[i]);
            fix_vertex_halfedge_safe(sm, base[j]);
            fix_vertex_halfedge_safe(sm, top[i]);
            fix_vertex_halfedge_safe(sm, top[j]);
            if (out) out->all.push_back(fw);
        }
    }

    // --- 7) КРЫШКА ИЗ top[] БЕЗ ПОИСКОВ ---
    F fcap = SM::null_face();
    if(false)
    {
        // подстрахуем: у верхних вершин может не стоять vertex->halfedge
        for (V v : top) fix_vertex_halfedge_safe(sm, v);

        std::vector<H> ring; ring.reserve(top.size());
        bool ok = true;
        for (size_t i=0, k = top.size(); i<k; ++i){
            V a = top[i], b = top[(i+1)%k];
            H h = get_or_make_border_h_uv(sm, a, b);        // вернуть именно border a->b,
            if (h == SM::null_halfedge()) { ok = false; break; } // если сторона занята — крышку не шьём
            ring.push_back(h);
        }

        if (ok) {
            fcap = attach_face_from_ringN(sm, ring);
            if (fcap != SM::null_face() && out){
                out->all.push_back(fcap);
                out->caps.push_back(fcap);
            }
            // необязательно, но полезно стабилизировать указатель у вершин
            for (V v : top) fix_vertex_halfedge_safe(sm, v, ring[0]);
        }
    }
    return fcap;
}

// add (or fetch) oriented halfedge u->v; returns a BORDER halfedge (face==null) on success
static inline H add_edge_oriented_new_old(SM& sm, V u, V v){
    if (u==v) return SM::null_halfedge();
    H h = sm.add_edge(u,v);
    if (h==SM::null_halfedge()) return SM::null_halfedge();
    if (sm.source(h)!=u) h = sm.opposite(h);
    return h;
}

static inline H add_edge_oriented_new(SM& sm, V u, V v) {
    H h = sm.add_edge(u, v);
    if (h == SM::null_halfedge()) return SM::null_halfedge();
    return (sm.source(h) == u) ? h : sm.opposite(h);
}

// Полностью заново строит vertex->halfedge.
// 1) сначала всем ставим null, чтобы не осталось «старых» значений;
// 2) подбираем лучший входящий (приоритет border), иначе входящий от opposite исходящего;
// 3) записываем обратно.
static inline void rebuild_vertex_halfedge_table_strict(SM& sm)
{
    const std::size_t capV = sm.number_of_vertices();

    // 0) обнуляем property (важно для случаев, когда best останется null)
    for (V v : sm.vertices())
        sm.set_halfedge(v, SM::null_halfedge());

    std::vector<H> best(capV, SM::null_halfedge());

    // 1) лучший входящий по target(h) == v (border в приоритете)
    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        V t = sm.target(h);
        if (!valid_v(sm,t)) continue;

        std::size_t ti = (std::size_t)t.idx();
        H cur = best[ti];

        const bool cur_is_border = (cur != SM::null_halfedge() && sm.face(cur) == SM::null_face());
        const bool h_is_border   = (sm.face(h) == SM::null_face());

        if (cur == SM::null_halfedge() || (h_is_border && !cur_is_border))
            best[ti] = h;
    }

    // 2) если входящего не нашлось — возьмём opposite от исходящего
    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        V s = sm.source(h);
        if (!valid_v(sm,s)) continue;

        std::size_t si = (std::size_t)s.idx();
        if (best[si] != SM::null_halfedge()) continue;

        H hin = sm.opposite(h);
        if (valid_h(sm,hin) && sm.target(hin) == s)
            best[si] = hin;
    }

    // 3) записываем
    for (V v : sm.vertices())
        sm.set_halfedge(v, best[(std::size_t)v.idx()]);
}

// Нормализация: у каждой вершины либо null, либо входящий halfedge.
// Если стоит исходящий — переворачиваем на opposite.
// Если стоит вообще не инцидентный — подбираем любой входящий/исходящий, иначе оставляем null.
static inline void normalize_vertex_halfedges(SM& sm)
{
    for (V v : sm.vertices()) {
        if (!valid_v(sm,v)) continue;

        H hv = sm.halfedge(v);
        if (hv == SM::null_halfedge()) continue;

        // если уже входящий — ок
        if (valid_h(sm,hv) && sm.target(hv) == v) continue;

        // если исходящий — переведём на opposite, если тот действительно входит
        if (valid_h(sm,hv) && sm.source(hv) == v) {
            H hin = sm.opposite(hv);
            if (valid_h(sm,hin) && sm.target(hin) == v) {
                sm.set_halfedge(v, hin);
                continue;
            }
        }

        // hv вообще не инцидентен v — найдём любой входящий
        H any_in = SM::null_halfedge();
        for (H h : sm.halfedges()) {
            if (!valid_h(sm,h)) continue;
            if (sm.target(h) == v) { any_in = h; break; }
        }
        if (any_in != SM::null_halfedge()) { sm.set_halfedge(v, any_in); continue; }

        // попробуем opposite от любого исходящего
        H any_out = SM::null_halfedge();
        for (H h : sm.halfedges()) {
            if (!valid_h(sm,h)) continue;
            if (sm.source(h) == v) { any_out = h; break; }
        }
        if (any_out != SM::null_halfedge()) {
            H hin = sm.opposite(any_out);
            if (valid_h(sm,hin) && sm.target(hin) == v) sm.set_halfedge(v, hin);
            else sm.set_halfedge(v, SM::null_halfedge());
        } else {
            // изолированная вершина — оставляем null
            sm.set_halfedge(v, SM::null_halfedge());
        }
    }
}

static inline void collect_face_halfedges(const SM& sm, F f, std::vector<H>& owned){
    owned.clear();
    owned.reserve(16);
    for (H h : sm.halfedges()){
        if (sm.is_removed(h)) continue;
        if (sm.face(h) == f) owned.push_back(h);
    }
}

// Отсортировать полурёбра грани в кольцо, используя соответствие source/target.
// Возвращает true, если получилось замкнутое кольцо (ring.size()==owned.size()).
static inline bool order_face_ring_by_vertices(const SM& sm,
                                               const std::vector<H>& owned,
                                               std::vector<H>& ring)
{
    ring.clear();
    if (owned.size() < 3) return false;

    // map: source(h) -> h
    std::unordered_map<std::size_t, H> by_src;
    by_src.reserve(owned.size()*2);
    for (H h : owned) by_src[(std::size_t)sm.source(h).idx()] = h;

    H start = owned.front();
    ring.push_back(start);
    for (;;) {
        V t = sm.target(ring.back());
        auto it = by_src.find((std::size_t)t.idx());
        if (it == by_src.end()) break;
        H nxt = it->second;
        if (nxt == start) {                // замкнулись
            if (ring.size()+1 == owned.size()) { ring.push_back(nxt); ring.pop_back(); return true; }
            return ring.size() == owned.size(); // закрылись преждевременно
        }
        // защита от повторов
        if (std::find(ring.begin(), ring.end(), nxt) != ring.end()) break;
        ring.push_back(nxt);
        if (ring.size() > owned.size()+1) break; // предохранитель
    }
    return ring.size() == owned.size();
}

// Жёстко "распороть" грань: всем её halfedge поставить face=null и прошить border-цикл на opp
static inline bool drop_face_to_border_all(SM& sm, F f)
{
    if (f == SM::null_face() || sm.is_removed(f)) return false;

    std::vector<H> owned;  collect_face_halfedges(sm, f, owned);
    if (owned.size() < 3) { sm.remove_face(f); return true; }

    std::vector<H> ring;   bool ok = order_face_ring_by_vertices(sm, owned, ring);
    if (!ok) {
        // даже если порядок не восстановили — всё равно делаем их border
        for (H h : owned) sm.set_face(h, SM::null_face());
        sm.set_halfedge(f, SM::null_halfedge());
        sm.remove_face(f);
        return true;
    }

    // 1) все стороны грани становятся border a->b
    for (H h : ring) sm.set_face(h, SM::null_face());

    // 2) прошиваем бордер-цикл на противоположной стороне:
    //    opp(h_{i+1}) -> opp(h_i)
    const size_t n = ring.size();
    for (size_t i=0; i<n; ++i){
        H o_cur  = sm.opposite(ring[i]);
        H o_next = sm.opposite(ring[(i+1)%n]);
        if (o_cur != SM::null_halfedge()  && sm.face(o_cur)  == SM::null_face() &&
            o_next!= SM::null_halfedge() && sm.face(o_next) == SM::null_face())
        {
            sm.set_next(o_next, o_cur);
        }
    }

    // 3) удалить саму грань
    sm.set_halfedge(f, SM::null_halfedge());
    sm.remove_face(f);
    return true;
}


// previous-of by scan (safe even if prev() isn’t available)
static inline H prev_of(const SM& sm, H target) {
    if (!valid_h(sm, target)) return SM::null_halfedge();
    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        if (sm.next(h) == target) return h;
    }
    return SM::null_halfedge();
}

// collect all incident halfedges safely (no circulators)
struct Incidents { std::vector<H> in, out; };
static inline Incidents incident_halfedges_linear(const SM& sm, V v) {
    Incidents I;
    if (!valid_v(sm,v)) return I;
    I.in.reserve(16); I.out.reserve(16);
    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        if (sm.target(h) == v) I.in.push_back(h);
        if (sm.source(h) == v) I.out.push_back(h);
    }
    return I;
}

// one-line halfedge descriptor
static inline void dump_halfedge(const SM& sm, H h, const char* tag = "") {
    if (!valid_h(sm,h)) {
        std::cerr << tag << " h=" << (h==SM::null_halfedge() ? -1 : (int)h.idx())
        << " INVALID\n";
        return;
    }
    H ho   = sm.opposite(h);
    H hn   = sm.next(h);
    H hp   = prev_of(sm, h);
    V s    = sm.source(h);
    V t    = sm.target(h);
    F f    = sm.face(h);
    bool br= (f == SM::null_face());
    std::cerr << tag
              << " h=" << h.idx()
              << " s=" << (valid_v(sm,s)?(int)s.idx():-1)
              << " t=" << (valid_v(sm,t)?(int)t.idx():-1)
              << " f=" << (valid_f(sm,f)?(int)f.idx():-1)
              << " next=" << (valid_h(sm,hn)?(int)hn.idx():-1)
              << " prev=" << (valid_h(sm,hp)?(int)hp.idx():-1)
              << " opp="  << (valid_h(sm,ho)?(int)ho.idx():-1)
              << " border=" << (br?"1":"0")
              << "\n";
}


// detailed vertex dump + reason why hv is “bad”
static inline void dump_vertex(const SM& sm, V v) {
    std::cerr << "---- Vertex v=" << (valid_v(sm,v)?(int)v.idx():-1) << " ----\n";
    if (!valid_v(sm,v)) { std::cerr << "INVALID VERTEX\n"; return; }

    H hv = sm.halfedge(v);
    std::cerr << " hv=" << (hv==SM::null_halfedge() ? -1 : (int)hv.idx()) << "\n";

    auto I = incident_halfedges_linear(sm, v);
    std::cerr << " degree_in="  << I.in.size()
              << " degree_out=" << I.out.size()
              << " degree="     << (I.in.size()+I.out.size()) << "\n";

    // diagnose hv
    if (hv == SM::null_halfedge()) {
        if (I.in.empty() && I.out.empty())
            std::cerr << " note: isolated vertex: hv=null is OK (should be removed before GC)\n";
        else
            std::cerr << " PROBLEM: hv=null but vertex has incident halfedges\n";
    } else if (!valid_h(sm,hv)) {
        std::cerr << " PROBLEM: hv invalid/removed\n";
    } else {
        if (sm.target(hv) != v)
            std::cerr << " PROBLEM: hv does not target v (bad orientation)\n";
        if (sm.source(hv) == sm.target(hv))
            std::cerr << " PROBLEM: hv is a loop (source==target)\n";
        // face/next sanity
        F f = sm.face(hv);
        if (f != SM::null_face()) {
            H h = hv; bool closed=false;
            for (int cap=0; cap< (int)sm.number_of_halfedges()+10; ++cap) {
                h = sm.next(h);
                if (h == hv) { closed=true; break; }
                if (!valid_h(sm,h) || sm.face(h)!=f) break;
            }
            if (!closed) std::cerr << " PROBLEM: face cycle from hv not closed via next()\n";
        }
    }

    // list in-coming halfedges (good candidates for hv)
    for (H h : I.in)  dump_halfedge(sm, h, "  in :");
    for (H h : I.out) dump_halfedge(sm, h, "  out:");
}

// quick scan for duplicate parallel edges u->v
static inline void find_parallel_halfedges(const SM& sm) {
    std::unordered_map<uint64_t, std::vector<H>> bucket;
    bucket.reserve(sm.number_of_halfedges());
    auto key = [](int u,int v)->uint64_t { return (uint64_t(uint32_t(u))<<32) | uint32_t(v); };

    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        V u = sm.source(h), v = sm.target(h);
        bucket[key((int)u.idx(), (int)v.idx())].push_back(h);
    }
    for (auto& kv : bucket) {
        const auto& vec = kv.second;
        if (vec.size() > 1) {
            V u = sm.source(vec[0]), v = sm.target(vec[0]);
            std::cerr << "WARN: multiple halfedges " << vec.size()
                      << " from " << (int)u.idx() << " to " << (int)v.idx() << " :";
            for (H h : vec) std::cerr << " " << (int)h.idx();
            std::cerr << "\n";
        }
    }
}

// global sanity sweep; returns #problems
static inline int sweep_and_report(const SM& sm) {
    int problems = 0;

    // halfedge opposites consistency
    for (H h : sm.halfedges()) {
        if (!valid_h(sm,h)) continue;
        H ho = sm.opposite(h);
        if (!valid_h(sm,ho) || sm.opposite(ho) != h) {
            ++problems;
            std::cerr << "PROBLEM: opposite mismatch on h="<<h.idx()<<"\n";
            dump_halfedge(sm, h,  "  h ");
            dump_halfedge(sm, ho, "  ho");
        }
        if (sm.source(h) == sm.target(h)) {
            ++problems;
            std::cerr << "PROBLEM: loop halfedge h="<<h.idx()<<"\n";
        }
    }

    // face cycles (via next)
    for (F f : sm.faces()) {
        if (!valid_f(sm,f)) continue;
        H h0 = sm.halfedge(f);
        if (!valid_h(sm,h0)) { ++problems; std::cerr << "PROBLEM: face "<<f.idx()<<" has null/invalid halfedge\n"; continue; }
        H h = h0; bool closed=false;
        for (int cap=0; cap<(int)sm.number_of_halfedges()+10; ++cap) {
            if (!valid_h(sm,h) || sm.face(h)!=f) { ++problems; std::cerr << "PROBLEM: face "<<f.idx()<<" broken ring\n"; break; }
            h = sm.next(h);
            if (h == h0) { closed=true; break; }
        }
        if (!closed) { ++problems; std::cerr << "PROBLEM: face "<<f.idx()<<" ring not closed\n"; }
    }

    // vertex hv sanity
    for (V v : sm.vertices()) {
        if (!valid_v(sm,v)) continue;
        H hv = sm.halfedge(v);
        auto I = incident_halfedges_linear(sm, v);
        if ((hv == SM::null_halfedge()) && (!I.in.empty() || !I.out.empty())) {
            ++problems;
            std::cerr << "PROBLEM: v="<<v.idx()<<" hv=null but degree>0\n";
        } else if (valid_h(sm,hv) && sm.target(hv)!=v) {
            ++problems;
            std::cerr << "PROBLEM: v="<<v.idx()<<" hv targets "<<(int)sm.target(hv).idx()<<" (expected "<<(int)v.idx()<<")\n";
        }
    }
    return problems;
}

// convenience: dump everything about a set of vertices + their given hv ids
static inline void debug_vertices(const SM& sm,
                                  const std::vector<int>& v_ids,
                                  const std::vector<int>& h_ids = {})
{
    std::cerr << "===== DEBUG SNAPSHOT =====\n";
    std::cerr << "V="<<sm.number_of_vertices()
              << " E="<<sm.number_of_edges()
              << " H="<<sm.number_of_halfedges()
              << " F="<<sm.number_of_faces()<<"\n";

    int issues = sweep_and_report(sm);
    if (issues==0) std::cerr << "Global sweep: no structural problems found.\n";

    find_parallel_halfedges(sm);

    // per vertex
    for (int vid : v_ids) {
        V v(vid);
        dump_vertex(sm, v);
    }

    // any explicit halfedges the log complains about
    for (int hid : h_ids) {
        H h(hid);
        dump_halfedge(sm, h, "explicit:");
    }
    std::cerr << "===== END DEBUG =====\n";
}


// Удобный перегруз: передали любой полурёбер на грани
static inline void drop_face_to_border(SM& sm, H any_on_face) {
    if (any_on_face == SM::null_halfedge()) return;
    drop_face_to_border_all(sm, sm.face(any_on_face));
}


// выбрать лучший входящий hv для v (бордер > любой входящий > opposite от исходящего > null)
static inline SM::Halfedge_index pick_best_incoming_hv(const SM& sm, V v) {
    // входящее граничное
    for (H h : sm.halfedges()) if (valid_h(sm,h) && sm.target(h)==v && sm.face(h)==SM::null_face()) return h;
    // любое входящее
    for (H h : sm.halfedges()) if (valid_h(sm,h) && sm.target(h)==v) return h;
    // opposite от исходящего
    for (H h : sm.halfedges()) if (valid_h(sm,h) && sm.source(h)==v) {
            H hin = sm.opposite(h);
            if (valid_h(sm,hin) && sm.target(hin)==v) return hin;
        }
    return SM::null_halfedge();
}

static inline void repair_vertex_halfedges_after_ops(SM& sm) {
    for (V v : sm.vertices()) {
        if (!valid_v(sm,v)) continue;
        H hv = sm.halfedge(v);
        if (!valid_h(sm,hv) || sm.target(hv)!=v) {
            H pick = pick_best_incoming_hv(sm, v);
            sm.set_halfedge(v, pick);
        }
    }
}

static inline void repair_face_halfedges_after_ops(SM& sm) {
    for (F f : sm.faces()) {
        if (!valid_f(sm,f)) continue;
        H h0 = sm.halfedge(f);
        if (valid_h(sm,h0) && sm.face(h0)==f) continue;
        // найдём любой h с face(h)==f
        H pick = SM::null_halfedge();
        for (H h : sm.halfedges()) {
            if (valid_h(sm,h) && sm.face(h)==f) { pick = h; break; }
        }
        sm.set_halfedge(f, pick);
    }
}

// === helpers (минимум того, что нужно тут) ===
inline bool is_border_h(const SM& sm, H h){ return h!=SM::null_halfedge() && sm.face(h)==SM::null_face(); }
inline bool is_strict_boundary_edge(const SM& sm, H h){
    if (h==SM::null_halfedge()) return false;
    H ho = sm.opposite(h);
    return is_border_h(sm,h) && ho!=SM::null_halfedge() && sm.face(ho)!=SM::null_face();
}
static inline H border_prev(const SM& sm, H hb){
    if (!is_border_h(sm,hb)) return SM::null_halfedge();
    H h = hb;
    for (size_t cap=0; cap<sm.number_of_halfedges()+10; ++cap){
        H n = sm.next(h);
        if (n==hb) return h;
        if (n==SM::null_halfedge() || !is_border_h(sm,n)) break;
        h = n;
    }
    return SM::null_halfedge();
}

static inline bool can_attach_quad(const SM& sm, H h0,H h1,H h2,H h3){
    H r[4]={h0,h1,h2,h3};
    for (int i=0;i<4;++i){
        if (r[i]==SM::null_halfedge()) return false;
        if (sm.face(r[i])!=SM::null_face()) return false;
        if (sm.target(r[i])!=sm.source(r[(i+1)&3])) return false;
    }
    return true;
}


struct RingCorner {
    V a;        // вершина a
    H h_ab;     // halfedge a->b по исходной грани
    bool was_border_ba; // до удаления грани противоположная половинка b->a была border?
};

static inline bool attach_single_wall_keep_base_face(
    SM& sm, H h_ba, V ta, V tb, H* out_top_border=nullptr)
{
    if (!is_strict_boundary_edge(sm, h_ba)) return false;

    V b = sm.source(h_ba);
    V a = sm.target(h_ba);

    // соседние по старому бордер-кольцу
    H nB = sm.next(h_ba);
    H pB = border_prev(sm, h_ba);
    if (!is_border_h(sm,nB) || !is_border_h(sm,pB)) return false;

    // три новые ребра для стены (пока border-половинки)
    H h_a_ta = add_edge_oriented_new(sm, a,  ta); // a->ta
    H h_ta_tb= add_edge_oriented_new(sm, ta, tb); // ta->tb (верх)
    H h_tb_b = add_edge_oriented_new(sm, tb, b);  // tb->b
    if (!is_border_h(sm,h_a_ta) || !is_border_h(sm,h_ta_tb) || !is_border_h(sm,h_tb_b)) return false;

    // валидное кольцо будущей грани
    if (!can_attach_quad(sm, h_ba, h_a_ta, h_ta_tb, h_tb_b)) return false;

    // прошиваем face-сторону
    F nf = sm.add_face(); if (nf==SM::null_face()) return false;
    H ring[4] = {h_ba, h_a_ta, h_ta_tb, h_tb_b};
    for (int i=0;i<4;++i){ sm.set_face(ring[i], nf); sm.set_next(ring[i], ring[(i+1)&3]); }
    sm.set_halfedge(nf, h_ba);

    // подшиваем ГЛОБАЛЬНОЕ бордер-кольцо вместо h_ba
    H o_a  = sm.opposite(h_a_ta);  // ta->a
    H o_top= sm.opposite(h_ta_tb); // tb->ta   (нужен для крышки)
    H o_b  = sm.opposite(h_tb_b);  // b->tb
    if (!is_border_h(sm,o_a) || !is_border_h(sm,o_top) || !is_border_h(sm,o_b)) return false;

    sm.set_next(pB,   o_b);
    sm.set_next(o_b,  o_top);
    sm.set_next(o_top,o_a);
    sm.set_next(o_a,  nB);

    // входящие опорные hv для затронутых вершин
    sm.set_halfedge(a,   h_ba);
    sm.set_halfedge(ta,  h_a_ta);
    sm.set_halfedge(tb,  h_ta_tb);
    sm.set_halfedge(b,   h_tb_b);

    if (out_top_border) *out_top_border = o_top; // tb->ta — пригодится на крышку
    return true;
}

// ---- helper 1: соберём prev для всех halfedge, у которых есть грань ----
static inline void build_prev_on_faces(SM& sm,
                                       std::vector<SM::Halfedge_index>& prev_of)
{
    prev_of.assign(sm.number_of_halfedges(), SM::null_halfedge());
    for (SM::Face_index f : sm.faces()) {
        SM::Halfedge_index h0 = sm.halfedge(f);
        if (h0 == SM::null_halfedge()) continue;
        SM::Halfedge_index h = h0, p = SM::null_halfedge();
        // обойдём кольцо и проставим prev: p -> h
        do {
            SM::Halfedge_index n = sm.next(h);
            // когда дошли до следующего, значит текущий h — prev для n
            if (n != SM::null_halfedge()) prev_of[n.idx()] = h;
            h = n;
        } while (h != h0 && h != SM::null_halfedge());
    }
}

// ---- helper 2: корректно восстановим next на ВСЕХ границах ----
static inline void rebuild_all_border_next_using_prev(SM& sm)
{
    std::vector<SM::Halfedge_index> prev_of;
    build_prev_on_faces(sm, prev_of);

    for (SM::Halfedge_index h : sm.halfedges()) {
        if (sm.is_removed(h) || sm.face(h) != SM::null_face()) continue; // берём только border
        SM::Halfedge_index ho = sm.opposite(h);
        if (ho == SM::null_halfedge()) continue;               // на Surface_mesh не бывает, но на всякий
        SM::Halfedge_index p  = prev_of[ho.idx()];             // prev вокруг внутренней грани
        if (p == SM::null_halfedge()) continue;                // если вдруг грань не замкнута
        SM::Halfedge_index nb = sm.opposite(p);                // формула: next(h) = opposite(prev(opposite(h)))
        if (nb != SM::null_halfedge()) sm.set_next(h, nb);
    }
}


// следующий входящий вокруг target(v): h (u->v) -> opposite(next(h)) (w->v)
static inline H next_around_target(SM& sm, H h){
    if (h==SM::null_halfedge()) return h;
    H n = sm.next(h);
    if (n==SM::null_halfedge()) return SM::null_halfedge();
    return sm.opposite(n);
}

// медленный, но надёжный поиск: прямой перебор halfedges
static inline H find_halfedge_sm_only(SM& sm, V a, V b) {
    if (a == b) return SM::null_halfedge();
    for (H h : sm.halfedges()) {
        if (sm.is_removed(h)) continue;
        if (sm.source(h) == a && sm.target(h) == b) return h;
    }
    return SM::null_halfedge();
}

// === было ===
// static inline H find_halfedge_sm_only(SM& sm, V a, V b) { ...O(E)... }
static inline void set_vertex_incoming_if_any(SM& sm, V v, H prefer=SM::null_halfedge()){
    if (v==SM::null_vertex()) return;
    if (prefer!=SM::null_halfedge() && sm.target(prefer)==v){ sm.set_halfedge(v,prefer); return; }
    H hv = sm.halfedge(v);
    if (hv!=SM::null_halfedge() && sm.target(hv)==v) return;
    for (H h : sm.halfedges()) if (!sm.is_removed(h) && sm.target(h)==v){ sm.set_halfedge(v,h); return; }
}

// === стало: O(valence(a)) ===
static inline H find_halfedge_local(SM& sm, V a, V b) {
    if (a == b) return SM::null_halfedge();

    // гарантируем, что у 'a' есть входящий halfedge для локального обхода
    H hin = sm.halfedge(a);
    if (hin == SM::null_halfedge()) {
        set_vertex_incoming_if_any(sm, a);   // твой хелпер
        hin = sm.halfedge(a);
        if (hin == SM::null_halfedge()) {
            // вершина действительно «висячая» — редкий случай: fallback
            for (H h : sm.halfedges()) {
                if (sm.is_removed(h)) continue;
                if (sm.source(h)==a && sm.target(h)==b) return h;
            }
            return SM::null_halfedge();
        }
    }

    // hin: входящий в a (u->a). Обходим ВСЕ входящие вокруг target=a.
    H h = hin;
    do {
        // противоположный — исходящий из a (a->w)
        H ho = sm.opposite(h);
        if (ho != SM::null_halfedge() && sm.target(ho) == b)
            return ho;

        h = next_around_target(sm, h);   // (u->a) -> (w->a)
    } while (h != hin && h != SM::null_halfedge());

    return SM::null_halfedge();
}

// Обновлённый ensure_oriented_halfedge: без глобального O(E)
static inline H ensure_oriented_halfedge(SM& sm, V a, V b) {
    if (a == b) return SM::null_halfedge();

    if (H h = find_halfedge_local(sm, a, b); h != SM::null_halfedge())
        return h;

    if (H hb = find_halfedge_local(sm, b, a); hb != SM::null_halfedge())
        return sm.opposite(hb);

    // Рёбра нет — создаём 1 пару halfedge'ов
    H h = sm.add_edge(a, b);                  // возвращает половинку с target==b
    if (h == SM::null_halfedge()) return SM::null_halfedge();

    // Однократно проставим "входящий" у вершин, чтобы в будущем локальный обход был возможен
    if (sm.halfedge(b) == SM::null_halfedge()) sm.set_halfedge(b, h);
    H ho = sm.opposite(h);
    if (sm.halfedge(a) == SM::null_halfedge()) sm.set_halfedge(a, ho);

    return h;                                  // ориентирован как a->b
}


// прошить крышку одним N-угольником из упорядоченного бордер-кольца (ta->tb)
static inline F attach_polygon_cap_from_ring_manual(SM& sm, const std::vector<H>& ring){
    using H = H; using F = F;
    const size_t m = ring.size(); if (m<3) return SM::null_face();
    for (size_t i=0;i<m;++i){
        H a=ring[i], b=ring[(i+1)%m];
        if (a==SM::null_halfedge() || b==SM::null_halfedge()) return SM::null_face();
        if (sm.face(a)!=SM::null_face()) return SM::null_face();
        if (sm.target(a)!=sm.source(b))  return SM::null_face();
    }
    F f = sm.add_face(); if (f==SM::null_face()) return f;
    for (size_t i=0;i<m;++i){ sm.set_face(ring[i],f); sm.set_next(ring[i], ring[(i+1)%m]); }
    sm.set_halfedge(f, ring[0]);
    return f;
}



static void quick_self_test() {
    using P = CGAL::Point_3<CGAL::Epick>;
    SM sm;
    V a = sm.add_vertex(P(0,0,0));
    V b = sm.add_vertex(P(1,0,0));

    auto probe = [&](const char* tag){
        std::cout << tag
                  << " | halfedges=" << sm.number_of_halfedges()
                  << "\n";
    };

    probe("start");

    // 1) создаст первую пару a-b
    {
        std::size_t hb = sm.number_of_halfedges();
        H h = ensure_oriented_halfedge(sm, a, b);
        std::size_t ha = sm.number_of_halfedges();
        std::cout << "ensure(a,b): created_pairs=" << ((ha-hb)/2)
                  << "  h=" << h.idx()
                  << " src=" << sm.source(h).idx()
                  << " tgt=" << sm.target(h).idx() << "\n";
        probe("after ensure #1");
    }

    // 2) повтор — дубль не создаётся
    {
        std::size_t hb = sm.number_of_halfedges();
        H h = ensure_oriented_halfedge(sm, a, b);
        std::size_t ha = sm.number_of_halfedges();
        std::cout << "ensure(a,b) again: created_pairs=" << ((ha-hb)/2)
                  << "  h=" << h.idx()
                  << " src=" << sm.source(h).idx()
                  << " tgt=" << sm.target(h).idx() << "\n";
        probe("after ensure #2");
    }

    // 3) прямой add_edge(a,b) — СОЗДАСТ ПАРАЛЛЕЛЬНОЕ ребро
    {
        std::size_t hb = sm.number_of_halfedges();
        H h2 = sm.add_edge(a, b); // ⚠️ дубль
        std::size_t ha = sm.number_of_halfedges();
        std::cout << "sm.add_edge(a,b): created_pairs=" << ((ha-hb)/2)
                  << "  h2=" << h2.idx()
                  << " src=" << sm.source(h2).idx()
                  << " tgt=" << sm.target(h2).idx() << "\n";
        probe("after duplicate");
    }
}

template<class SM>
using H_of = typename SM::Halfedge_index;
template<class SM>
using V_of = typename SM::Vertex_index;
template<class SM>
using F_of = typename SM::Face_index;


bool CgalMeshBuilder::checkMesh(
    SM& sm)
{
    //CHECKS
    // 1) каждая face имеет корректное кольцо
    for (F f : sm.faces()) {
        H h0 = sm.halfedge(f);
        if (h0 == SM::null_halfedge()) { std::cerr<<"face "<<f.idx()<<" has null halfedge\n"; }
        else {
            H h=h0; bool closed=false;
            for (int cap=0; cap < (int)sm.number_of_halfedges()+10; ++cap) {
                if (sm.face(h)!=f) { std::cerr<<"face "<<f.idx()<<" broken ring\n"; break; }
                h = sm.next(h);
                if (h==h0) { closed=true; break; }
            }
            if (!closed) std::cerr<<"face "<<f.idx()<<" ring not closed\n";
        }
    }

    // 2) для border половинок next определён и образует циклы
    for (H h : sm.halfedges()) {
        if (sm.face(h)==SM::null_face()) {
            H n = sm.next(h);
            if (n==SM::null_halfedge()) std::cerr<<"border h="<<h.idx()<<" has null next\n";
        }
    }

    // Проверка, что все border-циклы замкнуты и согласованы
    for (H h : sm.halfedges()) if (sm.face(h)==SM::null_face()) {
        H n = sm.next(h);
        if (n==SM::null_halfedge() || sm.source(n)!=sm.target(h)) {
            std::cerr<<"broken border at h="<<h.idx()<<"\n";
            break;
        }
    }

    // 3) опорный hv у вершин — входящий
    for (V v : sm.vertices()) {
        H hv = sm.halfedge(v);
        if (hv!=SM::null_halfedge() && sm.target(hv)!=v)
            std::cerr<<"v "<<v.idx()<<": hv is not incoming ("<<hv.idx()<<")\n";
    }


    // === Диагностика "counting halfedges via vertices" ===
    auto diag_count_via_vertices = [&](SM& sm){
        using H = SM::Halfedge_index;
        using V = SM::Vertex_index;

        // 1) общее число "живых" полурёбер
        std::size_t total = 0;
        for (H h : sm.halfedges()) if (!sm.is_removed(h)) ++total;

        // 2) обходим через вершины: h -> next(opposite(h)) вокруг target(h)
        std::vector<char> visited(sm.number_of_halfedges(), 0);
        std::size_t via = 0;

        for (V v : sm.vertices()){
            if (sm.is_removed(v)) continue;

            // найдём входящий к v стартовый полурёбер
            H start = sm.halfedge(v);
            int GUARD = (int)sm.number_of_halfedges() + 10;

            // если опорный не входящий — повернёмся вокруг вершины до входящего
            if (start != SM::null_halfedge() && sm.target(start) != v){
                H t = start; bool got = false;
                for (int g=0; g<GUARD; ++g){
                    H o = sm.opposite(t);
                    if (o == SM::null_halfedge()){
                        std::cerr<<"v "<<v.idx()<<": opposite(null) at h="<<t.idx()<<"\n";
                        break;
                    }
                    t = sm.next(o);
                    if (t == SM::null_halfedge()){
                        std::cerr<<"v "<<v.idx()<<": next(null) at o="<<o.idx()<<"\n";
                        break;
                    }
                    if (sm.target(t) == v){ start = t; got = true; break; }
                    if (t == sm.halfedge(v)) break;
                }
                if (!got) start = SM::null_halfedge();
            }

            if (start == SM::null_halfedge()){
                // попробуем найти любой входящий к v перебором (на случай пустого/неправильного опорного)
                for (H h : sm.halfedges()){
                    if (!sm.is_removed(h) && sm.target(h) == v){ start = h; break; }
                }
                if (start == SM::null_halfedge()){
                    std::cerr<<"v "<<v.idx()<<": no incoming halfedge found\n";
                    continue;
                }
            }

            // обход вокруг target(v)
            H h = start;
            for (int g=0; g<GUARD; ++g){
                if (!visited[h.idx()]){ visited[h.idx()] = 1; ++via; }
                H o = sm.opposite(h);
                if (o == SM::null_halfedge()){
                    std::cerr<<"around v "<<v.idx()<<": opposite(null) at h="<<h.idx()<<"\n";
                    break;
                }
                H n = sm.next(o);
                if (n == SM::null_halfedge()){
                    std::cerr<<"around v "<<v.idx()<<": next(null) at o="<<o.idx()<<"\n";
                    break;
                }
                if (sm.target(n) != v){
                    std::cerr<<"around v "<<v.idx()<<": next(opposite(h)) doesn't target v. h="
                              <<h.idx()<<" n="<<n.idx()<<" target(n)="<<sm.target(n).idx()<<"\n";
                    break;
                }
                h = n;
                if (h == start) break; // замкнули цикл вокруг v
            }
        }

        if (via != total){
            std::cerr<<"counting halfedges via vertices failed: "<<via<<" vs "<<total<<"\n";
            // покажем первый "потерянный" h
            for (H h : sm.halfedges()){
                if (sm.is_removed(h)) continue;
                if (!visited[h.idx()]){
                    std::cerr<<"  unreachable halfedge h="<<h.idx()
                    <<" "<<sm.source(h).idx()<<"->"<<sm.target(h).idx()
                    <<" face="<<(sm.face(h)==SM::null_face() ? -1 : (int)sm.face(h).idx())
                    <<" opp="<<(sm.opposite(h)==SM::null_halfedge() ? -1 : (int)sm.opposite(h).idx())
                    <<" next="<<(sm.next(h)==SM::null_halfedge() ? -1 : (int)sm.next(h).idx())
                    <<"\n";
                    break;
                }
            }
        } else {
            std::cerr<<"counting halfedges via vertices: OK "<<via<<" == "<<total<<"\n";
        }
    };

    // вызови здесь:
    diag_count_via_vertices(sm);

    bool ok = CGAL::is_valid_polygon_mesh(sm, true); // true => verbose
    if(!ok)
    {
        std::cerr<<"mesh not valid \n";
    }

    return ok;
}


static inline F extrude_face_from_plan_uid5(
    SM& sm,
    const FacePlan& plan,
    double distExtrude, double amountExtrude,
    const Point_3& meshC,
    SM::Property_map<V,std::uint64_t>& vuid,
    SM::Property_map<F,std::uint64_t>& /*fuid*/,
    std::uint64_t& next_vuid, std::uint64_t& /*next_fuid*/,
    std::unordered_map<std::uint64_t,V>& vByUid,
    std::unordered_map<std::uint64_t,F>& fByUid,
    ExtrudeLists* out)
{


    //quick_self_test();

    using H = typename SM::Halfedge_index;
    using V = typename SM::Vertex_index;
    using F = typename SM::Face_index;

    // --- 0) найти исходную грань f по uid ---
    auto itF = fByUid.find(plan.fuid);
    if (itF == fByUid.end()) return SM::null_face();
    F f = itF->second;
    if (f == SM::null_face() || sm.is_removed(f)) return SM::null_face();

    // --- 1) кольцо исходной грани (CCW), R[i].h_ab : a_i -> b_i ---
    struct RingCorner { V a; H h_ab; bool was_border_ba; };
    std::vector<RingCorner> R; R.reserve(32);
    std::vector<Point_3>    baseP; baseP.reserve(32);

    H h0 = sm.halfedge(f);
    if (h0 == SM::null_halfedge())
        return SM::null_face();

    {
        H h = h0; std::unordered_set<V> seen;
        do{
            if (h == SM::null_halfedge())
                return SM::null_face();
            V a = sm.source(h);
            if (a == SM::null_vertex() || !seen.insert(a).second)
                return SM::null_face();
            H ho = sm.opposite(h);
            bool wasB = (ho != SM::null_halfedge() && sm.face(ho) == SM::null_face());
            R.push_back({a, h, wasB});
            baseP.push_back(sm.point(a));
            h = sm.next(h);
        } while(h != h0);
    }

    const size_t k = R.size();
    if (k < 3) return SM::null_face();
    auto nexti = [&](size_t i){ return (i+1)%k; };

    // --- 2) верхний контур (геометрия как в вашем TS) ---
    const Point_3 c  = centroid_points(baseP);
    Vector_3      n  = newell_normal(baseP);
    if (CGAL::scalar_product(n, Vector_3(c.x()-meshC.x(), c.y()-meshC.y(), c.z()-meshC.z())) < 0) n = -n;
    const Point_3 cc(c.x()+n.x()*distExtrude, c.y()+n.y()*distExtrude, c.z()+n.z()*distExtrude);

    std::vector<V> top(k, SM::null_vertex());
    for (size_t i=0;i<k;++i){
        const Vector_3 d(baseP[i].x()-c.x(), baseP[i].y()-c.y(), baseP[i].z()-c.z());
        const double len = std::sqrt(d.x()*d.x()+d.y()*d.y()+d.z()*d.z());
        Vector_3 dn = (len>0) ? Vector_3(d.x()/len, d.y()/len, d.z()/len) : Vector_3(0,0,0);
        const Point_3 pt(cc.x()+amountExtrude*len*dn.x(),
                         cc.y()+amountExtrude*len*dn.y(),
                         cc.z()+amountExtrude*len*dn.z());
        V tv = sm.add_vertex(pt);
        vuid[tv] = next_vuid++;    // ваш assign_uid_vertex_new(...)
        vByUid[vuid[tv]] = tv;
        top[i] = tv;
    }

    // --- 3) превратить f в "дырку": все h_ab -> border-цикл по порядку ---
    for (size_t i=0;i<k;++i) sm.set_face(R[i].h_ab, SM::null_face());
    for (size_t i=0;i<k;++i) sm.set_next(R[i].h_ab, R[(i+1)%k].h_ab);

    // локальные соседи в «дырке» (чтоб не звать чужие бордер-обходы)
    std::vector<H> hole_prev(k), hole_next(k);
    for (size_t i=0;i<k;++i){ hole_prev[i]=R[(i+k-1)%k].h_ab; hole_next[i]=R[(i+1)%k].h_ab; }

    // 4) подготовить стойки и «верх» без дублей
    std::vector<H> e_up(k), e_top(k);
    for (size_t i=0;i<k;++i){
        size_t j=(i+1)%k;
        e_up[i]  = ensure_oriented_halfedge(sm, R[i].a, top[i]); // a->ta
        e_top[i] = ensure_oriented_halfedge(sm, top[i], top[j]); // ta->tb
        if (e_up[i]==SM::null_halfedge() || e_top[i]==SM::null_halfedge())
            return SM::null_face();
    }

    // 5) стены + локальный сплайс бордера вместо h_ab
    for (size_t i=0;i<k;++i){
        size_t j=(i+1)%k;
        H h_ab    = R[i].h_ab;                 // низ a->b (уже border)
        V a       = R[i].a;
        V b       = sm.target(h_ab);
        V ta      = top[i];
        V tb      = top[j];

        H h_b_tb  = e_up[j];                 // b(=a_j)   -> tb(=ta_j)
        H h_tb_ta = sm.opposite(e_top[i]);   // tb        -> ta
        H h_ta_a  = sm.opposite(e_up[i]);    // ta        -> a

        // кольцо квадра
        if (sm.face(h_ab)!=SM::null_face() || sm.face(h_b_tb)!=SM::null_face()
            || sm.face(h_tb_ta)!=SM::null_face() || sm.face(h_ta_a)!=SM::null_face())
            continue;
        if (sm.target(h_ab)!=sm.source(h_b_tb) ||
            sm.target(h_b_tb)!=sm.source(h_tb_ta) ||
            sm.target(h_tb_ta)!=sm.source(h_ta_a) ||
            sm.target(h_ta_a)!=sm.source(h_ab))
                continue;

        F fw = sm.add_face(); if (fw==SM::null_face())
            continue;
        H ring[4] = { h_ab, h_b_tb, h_tb_ta, h_ta_a };
        for (int t=0;t<4;++t){
            sm.set_face(ring[t], fw); sm.set_next(ring[t], ring[(t+1)&3]);
        }
        sm.set_halfedge(fw, h_ab);

        sm.set_halfedge(sm.target(h_ab),   h_ab);    // target(h_ab) = b
        sm.set_halfedge(sm.target(h_b_tb), h_b_tb);  // target = tb
        sm.set_halfedge(sm.target(h_tb_ta),h_tb_ta); // target = ta
        sm.set_halfedge(sm.target(h_ta_a), h_ta_a);  // target = a


    }

    // 6) крышка одним n-угольником по ta->tb (e_top)
    std::vector<H> cap_ring; cap_ring.reserve(k);
    for (size_t i=0;i<k;++i){
        H h = e_top[i]; // ta->tb
        if (h==SM::null_halfedge() || sm.face(h)!=SM::null_face()) { cap_ring.clear(); break; }
        if (!cap_ring.empty() && sm.target(cap_ring.back())!=sm.source(h)) { cap_ring.clear(); break; }
        cap_ring.push_back(h);
    }
    F fcap = SM::null_face();
    if (!cap_ring.empty()){
        fcap = attach_polygon_cap_from_ring_manual(sm, cap_ring);
        // входящий для вершин крышки — удобно выставить
        for (size_t i=0;i<k;++i){
            V ta = top[i];
            H incoming = sm.opposite(e_top[(i+k-1)%k]); // (tb->ta)
            set_vertex_incoming_if_any(sm, ta, incoming);
        }
    }

    // 7) удалить исходную грань и её uid — только теперь!
    fByUid.erase(plan.fuid);
    sm.set_halfedge(f, SM::null_halfedge());
    sm.remove_face(f);

    return fcap;
}


std::vector<F> CgalMeshBuilder::extrudeFaces_collectBoth(
    SM& sm,
    const std::vector<F>& faces,
    double distance,
    double scale)
{
    std::vector<F> todo;
    for (auto f: faces) if (f!=SM::null_face() && !sm.is_removed(f)) todo.push_back(f);

    // UID и планы
    SM::Property_map<V,std::uint64_t> vuid;
    SM::Property_map<F,std::uint64_t> fuid;
    std::uint64_t next_vuid=0, next_fuid=0;
    ensure_uid_maps_and_assign_all(sm, vuid, fuid, next_vuid, next_fuid);

    std::vector<FacePlan> plans;
    build_face_plans_snapshot(sm, todo, vuid, fuid, plans);

    std::unordered_map<std::uint64_t,V> vByUid; rebuild_vertex_uid_map(sm, vuid, vByUid);
    std::unordered_map<std::uint64_t,F> fByUid; rebuild_face_uid_map(sm, fuid, fByUid);

    const Point_3 MC = mesh_centroid(sm);

    std::vector<F> caps; caps.reserve(plans.size());
    ExtrudeLists dump; // если нужно копить стены; иначе можно убрать

    for (auto p: plans) {
        F cap = extrude_face_from_plan_uid5(
            sm, p, distance, scale, MC,
            vuid, fuid, next_vuid, next_fuid,
            vByUid, fByUid, &dump);

        if (cap != SM::null_face()) caps.push_back(cap);

        // обновляем словари под новые вершины/удалённые грани
        rebuild_vertex_uid_map(sm, vuid, vByUid);
        rebuild_face_uid_map(sm, fuid, fByUid);
    }
    // без collect_garbage, чтобы дескрипторы остались валидными
    return caps;
}


static inline double vlen(const Vector_3& v){
    return std::sqrt(CGAL::to_double(v.squared_length()));
}
static inline Vector_3 vnorm(const Vector_3& v){
    double L = vlen(v); if (L==0) return Vector_3(0,0,0);
    return Vector_3(v.x()/L, v.y()/L, v.z()/L);
}
static inline Vector_3 vscale(const Vector_3& v, double s){
    return Vector_3(v.x()*s, v.y()*s, v.z()*s);
}
static inline Vector_3 rotate_around_axis(const Vector_3& v,
                                          const Vector_3& axis_unit,
                                          double ang)
{
    if (ang==0) return v;
    const double c = std::cos(ang), s = std::sin(ang);
    const Vector_3 a = axis_unit;                    // |a|=1
    const double vpa = CGAL::to_double(CGAL::scalar_product(v, a));
    const Vector_3 v_parallel = vscale(a, vpa);
    const Vector_3 v_perp     = v - v_parallel;
    const Vector_3 v_perp_rot = vscale(v_perp, c) + vscale(CGAL::cross_product(a, v_perp), s);
    return v_parallel + v_perp_rot;
}


// Выбираем ось, наименее коллинеарную нормали, и из неё строим t0, t1.
// Дополнительно фиксируем знак t0 по глобальной оси X (или Y как запасной план).
inline void canonical_tangent_basis(const Vector_3& n_unit, Vector_3& t0, Vector_3& t1){
    Vector_3 ref = (std::abs(n_unit.z()) < 0.9) ? Vector_3(0,0,1) : Vector_3(0,1,0);
    t0 = vnorm(CGAL::cross_product(ref, n_unit));
    if (t0 == CGAL::NULL_VECTOR) ref = Vector_3(1,0,0), t0 = vnorm(CGAL::cross_product(ref, n_unit));
    t1 = vnorm(CGAL::cross_product(n_unit, t0));

    // Стабилизируем направление (чтобы t0 не «флипался» от грани к грани)
    double ex = CGAL::to_double(CGAL::scalar_product(t0, Vector_3(1,0,0)));
    double ey = CGAL::to_double(CGAL::scalar_product(t0, Vector_3(0,1,0)));
    if (std::abs(ex) > 1e-9) { if (ex < 0) t0 = -t0, t1 = -t1; }
    else if (ey < 0)          { t0 = -t0, t1 = -t1; }
}


static inline F extrude_face_from_plan_uidRotate(
    SM& sm,
    const FacePlan& plan,
    double distExtrude, double amountExtrude,
    const Point_3& meshC,
    SM::Property_map<V,std::uint64_t>& vuid,
    SM::Property_map<F,std::uint64_t>& /*fuid*/,
    std::uint64_t& next_vuid, std::uint64_t& /*next_fuid*/,
    std::unordered_map<std::uint64_t,V>& vByUid,
    std::unordered_map<std::uint64_t,F>& fByUid,
    ExtrudeLists* out,
    double twist_rad = 0.0,   // NEW: скрутка вокруг нормали (рад)
    double tilt_rad  = 0.0  )
{


    //quick_self_test();

    using H = typename SM::Halfedge_index;
    using V = typename SM::Vertex_index;
    using F = typename SM::Face_index;

    // --- 0) найти исходную грань f по uid ---
    auto itF = fByUid.find(plan.fuid);
    if (itF == fByUid.end()) return SM::null_face();
    F f = itF->second;
    if (f == SM::null_face() || sm.is_removed(f)) return SM::null_face();

    // --- 1) кольцо исходной грани (CCW), R[i].h_ab : a_i -> b_i ---
    struct RingCorner { V a; H h_ab; bool was_border_ba; };
    std::vector<RingCorner> R; R.reserve(32);
    std::vector<Point_3>    baseP; baseP.reserve(32);

    H h0 = sm.halfedge(f);
    if (h0 == SM::null_halfedge())
        return SM::null_face();

    {
        H h = h0; std::unordered_set<V> seen;
        do{
            if (h == SM::null_halfedge())
                return SM::null_face();
            V a = sm.source(h);
            if (a == SM::null_vertex() || !seen.insert(a).second)
                return SM::null_face();
            H ho = sm.opposite(h);
            bool wasB = (ho != SM::null_halfedge() && sm.face(ho) == SM::null_face());
            R.push_back({a, h, wasB});
            baseP.push_back(sm.point(a));
            h = sm.next(h);
        } while(h != h0);
    }

    const size_t k = R.size();
    if (k < 3) return SM::null_face();
    auto nexti = [&](size_t i){ return (i+1)%k; };

    // --- 2) верхний контур с twist/tilt (стабильная касательная) ---
    const Point_3 c  = centroid_points(baseP);
    Vector_3 n0 = newell_normal(baseP);
    Vector_3      n_unit = vnorm(n0);

    // Каноническое основание
    Vector_3 t0, t1;
    canonical_tangent_basis(n_unit, t0, t1);

    // Наклоняем нормаль вокруг t0
    Vector_3 n_tilt = rotate_around_axis(n_unit, t0, tilt_rad);

    // Центр крышки после смещения вдоль наклонённой нормали
    const Point_3 cc(c.x()+n_tilt.x()*distExtrude,
                     c.y()+n_tilt.y()*distExtrude,
                     c.z()+n_tilt.z()*distExtrude);

    // Верхние вершины
    std::vector<V> top(k, SM::null_vertex());
    for (size_t i=0; i<k; ++i){
        Vector_3 d = baseP[i] - c;
        double dn  = CGAL::to_double(CGAL::scalar_product(d, n_unit));
        Vector_3 r_plane = d - n_unit*dn;                 // радиус в плоскости грани

        // Хочешь, чтобы «скрутка» считалась уже в наклонённой плоскости —
        // крути вокруг n_tilt (а не n_unit). Оба варианта валидны:
        Vector_3 r_twist = rotate_around_axis(r_plane, /* n_unit или */ n_tilt, twist_rad);
        Vector_3 r_tilt  = rotate_around_axis(r_twist, t0,          tilt_rad);

        const double r_len = std::sqrt(CGAL::to_double(r_plane.squared_length()));
        Vector_3 offset    = vnorm(r_tilt) * (amountExtrude * r_len);

        const Point_3 pt(cc.x()+offset.x(), cc.y()+offset.y(), cc.z()+offset.z());

        V tv = sm.add_vertex(pt);
        vuid[tv] = next_vuid++;
        vByUid[vuid[tv]] = tv;
        top[i] = tv;
    }



    // --- 3) превратить f в "дырку": все h_ab -> border-цикл по порядку ---
    for (size_t i=0;i<k;++i) sm.set_face(R[i].h_ab, SM::null_face());
    for (size_t i=0;i<k;++i) sm.set_next(R[i].h_ab, R[(i+1)%k].h_ab);

    // локальные соседи в «дырке» (чтоб не звать чужие бордер-обходы)
    std::vector<H> hole_prev(k), hole_next(k);
    for (size_t i=0;i<k;++i){ hole_prev[i]=R[(i+k-1)%k].h_ab; hole_next[i]=R[(i+1)%k].h_ab; }

    // 4) подготовить стойки и «верх» без дублей
    std::vector<H> e_up(k), e_top(k);
    for (size_t i=0;i<k;++i){
        size_t j=(i+1)%k;
        e_up[i]  = ensure_oriented_halfedge(sm, R[i].a, top[i]); // a->ta
        e_top[i] = ensure_oriented_halfedge(sm, top[i], top[j]); // ta->tb
        if (e_up[i]==SM::null_halfedge() || e_top[i]==SM::null_halfedge())
            return SM::null_face();
    }

    // 5) стены + локальный сплайс бордера вместо h_ab
    for (size_t i=0;i<k;++i){
        size_t j=(i+1)%k;
        H h_ab    = R[i].h_ab;                 // низ a->b (уже border)
        V a       = R[i].a;
        V b       = sm.target(h_ab);
        V ta      = top[i];
        V tb      = top[j];

        H h_b_tb  = e_up[j];                 // b(=a_j)   -> tb(=ta_j)
        H h_tb_ta = sm.opposite(e_top[i]);   // tb        -> ta
        H h_ta_a  = sm.opposite(e_up[i]);    // ta        -> a

        // кольцо квадра
        if (sm.face(h_ab)!=SM::null_face() || sm.face(h_b_tb)!=SM::null_face()
            || sm.face(h_tb_ta)!=SM::null_face() || sm.face(h_ta_a)!=SM::null_face())
            continue;
        if (sm.target(h_ab)!=sm.source(h_b_tb) ||
            sm.target(h_b_tb)!=sm.source(h_tb_ta) ||
            sm.target(h_tb_ta)!=sm.source(h_ta_a) ||
            sm.target(h_ta_a)!=sm.source(h_ab))
            continue;

        F fw = sm.add_face(); if (fw==SM::null_face())
            continue;
        H ring[4] = { h_ab, h_b_tb, h_tb_ta, h_ta_a };
        for (int t=0;t<4;++t){
            sm.set_face(ring[t], fw); sm.set_next(ring[t], ring[(t+1)&3]);
        }
        sm.set_halfedge(fw, h_ab);

        sm.set_halfedge(sm.target(h_ab),   h_ab);    // target(h_ab) = b
        sm.set_halfedge(sm.target(h_b_tb), h_b_tb);  // target = tb
        sm.set_halfedge(sm.target(h_tb_ta),h_tb_ta); // target = ta
        sm.set_halfedge(sm.target(h_ta_a), h_ta_a);  // target = a


    }

    // 6) крышка одним n-угольником по ta->tb (e_top)
    std::vector<H> cap_ring; cap_ring.reserve(k);
    for (size_t i=0;i<k;++i){
        H h = e_top[i]; // ta->tb
        if (h==SM::null_halfedge() || sm.face(h)!=SM::null_face()) { cap_ring.clear(); break; }
        if (!cap_ring.empty() && sm.target(cap_ring.back())!=sm.source(h)) { cap_ring.clear(); break; }
        cap_ring.push_back(h);
    }
    F fcap = SM::null_face();
    if (!cap_ring.empty()){
        fcap = attach_polygon_cap_from_ring_manual(sm, cap_ring);
        // входящий для вершин крышки — удобно выставить
        for (size_t i=0;i<k;++i){
            V ta = top[i];
            H incoming = sm.opposite(e_top[(i+k-1)%k]); // (tb->ta)
            set_vertex_incoming_if_any(sm, ta, incoming);
        }
    }

    // 7) удалить исходную грань и её uid — только теперь!
    fByUid.erase(plan.fuid);
    sm.set_halfedge(f, SM::null_halfedge());
    sm.remove_face(f);

    return fcap;
}

std::vector<F> CgalMeshBuilder::extrudeFaces_collectBothRotate(
    SM& sm,
    const std::vector<F>& faces,
    double distance,
    double scale,
    double twist_rad ,   // NEW: скрутка вокруг нормали (рад)
    double tilt_rad)
{
    std::vector<F> todo;
    for (auto f: faces) if (f!=SM::null_face() && !sm.is_removed(f)) todo.push_back(f);

    // UID и планы
    SM::Property_map<V,std::uint64_t> vuid;
    SM::Property_map<F,std::uint64_t> fuid;
    std::uint64_t next_vuid=0, next_fuid=0;
    ensure_uid_maps_and_assign_all(sm, vuid, fuid, next_vuid, next_fuid);

    std::vector<FacePlan> plans;
    build_face_plans_snapshot(sm, todo, vuid, fuid, plans);

    std::unordered_map<std::uint64_t,V> vByUid; rebuild_vertex_uid_map(sm, vuid, vByUid);
    std::unordered_map<std::uint64_t,F> fByUid; rebuild_face_uid_map(sm, fuid, fByUid);

    const Point_3 MC = mesh_centroid(sm);

    std::vector<F> caps; caps.reserve(plans.size());
    ExtrudeLists dump; // если нужно копить стены; иначе можно убрать

    for (auto p: plans) {
        F cap = extrude_face_from_plan_uidRotate(
            sm, p, distance, scale, MC,
            vuid, fuid, next_vuid, next_fuid,
            vByUid, fByUid, &dump, twist_rad, tilt_rad);

        if (cap != SM::null_face()) caps.push_back(cap);

        // обновляем словари под новые вершины/удалённые грани
        rebuild_vertex_uid_map(sm, vuid, vByUid);
        rebuild_face_uid_map(sm, fuid, fByUid);
    }
    // без collect_garbage, чтобы дескрипторы остались валидными
    return caps;
}

// Экструзия ОДНОЙ грани по текущему f, с UID/halfedge-сшивкой.
// (обёртка: строит FacePlan на лету, полезно для единичного вызова)
SurfaceMesh::Face_index CgalMeshBuilder::extrude_face_like_ts(
    SM& sm, F f,
    double distExtrude, double amountExtrude,
    ExtrudeLists* out /*nullable*/)
{
    if (f == SM::null_face() || sm.is_removed(f)) return SM::null_face();
    if (out) { out->all.clear(); out->caps.clear(); }

    // 1) UID-карты и счётчики (создадутся, если их ещё нет).
    SM::Property_map<V, std::uint64_t> vuid;
    SM::Property_map<F, std::uint64_t> fuid;
    std::uint64_t next_vuid = 0, next_fuid = 0;
    ensure_uid_maps_and_assign_all(sm, vuid, fuid, next_vuid, next_fuid);

    // 2) Снимок только для этой грани (фиксируем кольцо вершин по UID и флаги wasBorder).
    std::vector<FacePlan> plans;
    build_face_plans_snapshot(sm, std::vector<F>{f}, vuid, fuid, plans);
    if (plans.empty()) return SM::null_face();

    // 3) Текущие словари UID→дескриптор.
    std::unordered_map<std::uint64_t, V> vByUid; rebuild_vertex_uid_map(sm, vuid, vByUid);
    std::unordered_map<std::uint64_t, F> fByUid; rebuild_face_uid_map(sm, fuid, fByUid);

    // 4) Центр всей сетки — чтобы направлять экструзию «наружу».
    const Point_3 MC = mesh_centroid(sm);

    // 5) Собственно экструзия по слепку (строит стены через halfedge’ы, создаёт крышку).
    return extrude_face_from_plan_uid(
        sm,
        plans.front(),
        distExtrude, amountExtrude,
        MC,
        vuid, fuid, next_vuid, next_fuid,
        vByUid, fByUid,
        out);
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
                G[j][i] = sm.add_vertex( bilerp(p00,p10,p11,p01, u,v) );
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

static inline void draw_halfedge_arrow_inset(
    const SurfaceMesh& sm,
    SurfaceMesh::Halfedge_index h,
    const glm::vec3& faceN,          // нормаль грани
    const glm::vec3& faceCenter,     // центр грани (в мировых координатах)
    const glm::vec3& col,
    float insetRel,                  // 0..1 — доля длины ребра
    float headRel,                   // 0..1 — доля длины ребра
    std::vector<Vertex>& out)
{
    using H = SurfaceMesh::Halfedge_index;
    if (h == SurfaceMesh::null_halfedge()) return;

    auto va = source(h, sm);
    auto vb = target(h, sm);
    if (va == SurfaceMesh::null_vertex() || vb == SurfaceMesh::null_vertex()) return;

    glm::vec3 A = to_glm3(sm.point(va));
    glm::vec3 B = to_glm3(sm.point(vb));
    glm::vec3 C = faceCenter;

    glm::vec3 dir  = safe_norm(B - A);                  // вдоль ребра
    glm::vec3 side = safe_norm(glm::cross(faceN, dir)); // в плоскости грани, перпендикул. ребру

    // «внутрь» — туда, где центр грани
    glm::vec3 M      = 0.5f * (A + B);
    float     signIn = glm::dot(safe_norm(C - M), side) >= 0.f ? 1.f : -1.f;

    float L         = glm::length(B - A);
    float insetAbs  = insetRel * L;
    glm::vec3 shift = side * signIn * insetAbs;

    // немного укорачиваем, чтобы не прилипать к вершинам
    glm::vec3 A2 = glm::mix(A, B, 0.15f) + shift;
    glm::vec3 B2 = glm::mix(A, B, 0.85f) + shift;

    auto push = [&](const glm::vec3& P, const glm::vec3& Q){
        out.push_back({ glm::vec4(P,1), glm::vec4(faceN,0), glm::vec4(col,1) });
        out.push_back({ glm::vec4(Q,1), glm::vec4(faceN,0), glm::vec4(col,1) });
    };

    // ствол
    push(A2, B2);

    // наконечник
    float hl   = std::min(L*0.25f, L*headRel);
    glm::vec3 base = B2 - dir * hl;
    glm::vec3 Lft  = base + side * (0.6f*hl) * signIn;
    glm::vec3 Rgt  = base - side * (0.6f*hl) * signIn;
    push(B2, Lft);
    push(B2, Rgt);
}


void CgalMeshBuilder::buildHalfedgeArrows(
    const SurfaceMesh& sm,
    std::vector<Vertex>& outLineVerts,
    float inset, float headRel, bool draw_border)
{
    outLineVerts.clear();

    // --- 1) по всем граням ---
    for (auto f : sm.faces()) {
        if (sm.is_removed(f)) continue;

        // собрать кольцо, центр и нормаль (Newell вокруг центра)
        std::vector<SurfaceMesh::Vertex_index> ring;
        auto h0 = sm.halfedge(f); if (h0 == SurfaceMesh::null_halfedge()) continue;
        auto h = h0;

        Point_3 C(0,0,0); size_t cnt=0;
        do {
            auto v = target(h, sm);
            ring.push_back(v);
            const auto& p = sm.point(v);
            C = Point_3(C.x()+p.x(), C.y()+p.y(), C.z()+p.z());
            ++cnt; h = next(h, sm);
        } while (h != h0);
        if (cnt < 3) continue;
        C = Point_3(C.x()/cnt, C.y()/cnt, C.z()/cnt);

        Vector_3 Nsum(0,0,0);
        for (size_t i=0;i<ring.size();++i){
            const auto& pi = sm.point(ring[i]);
            const auto& pj = sm.point(ring[(i+1)%ring.size()]);
            Nsum = Nsum + CGAL::cross_product(pj - C, pi - C);
        }
        glm::vec3 N  = safe_norm(to_glm3(Nsum));
        glm::vec3 Cg = to_glm3(C);

        // стойкий seed: индекс грани
        uint64_t seed = static_cast<uint64_t>(std::size_t(f));

        // стрелки для halfedge’ов этой грани — смещаем к её центру
        h = h0;
        do {
            glm::vec3 col = pick_color(false, seed);
            draw_halfedge_arrow_inset(sm, h, N, Cg, col, inset, headRel, outLineVerts);
            h = next(h, sm);
        } while (h != h0);
    }

    if (!draw_border) return;

    // --- 2) для граничных рёбер — смещаем к центру соседней (внутренней) грани ---
    for (auto e : sm.edges()) {
        auto h  = halfedge(e, sm); if (h == SurfaceMesh::null_halfedge()) continue;
        auto ho = opposite(h, sm);

        SurfaceMesh::Halfedge_index hb = SurfaceMesh::null_halfedge();
        SurfaceMesh::Halfedge_index hi = SurfaceMesh::null_halfedge();

        if (CGAL::is_border(h, sm)  && !CGAL::is_border(ho, sm)) { hb = h;  hi = ho; }
        else if (CGAL::is_border(ho, sm) && !CGAL::is_border(h, sm)) { hb = ho; hi = h; }
        else continue;

        auto ff = face(hi, sm); if (ff == SurfaceMesh::null_face()) continue;

        // центр + нормаль внутренней грани
        auto hf = sm.halfedge(ff);
        auto t  = hf;
        Point_3 C(0,0,0); size_t cnt=0;
        do { auto v = target(t, sm); const auto& p=sm.point(v);
            C = Point_3(C.x()+p.x(), C.y()+p.y(), C.z()+p.z());
            ++cnt; t = next(t, sm);
        } while (t != hf);
        C = Point_3(C.x()/cnt, C.y()/cnt, C.z()/cnt);

        Vector_3 Nsum(0,0,0);
        t = hf;
        do {
            auto vi = target(t, sm);
            auto vj = target(next(t, sm), sm);
            Nsum = Nsum + CGAL::cross_product(sm.point(vj) - C, sm.point(vi) - C);
            t = next(t, sm);
        } while (t != hf);

        glm::vec3 N  = safe_norm(to_glm3(Nsum));
        glm::vec3 Cg = to_glm3(C);

        glm::vec3 col = pick_color(true, 0);
        draw_halfedge_arrow_inset(sm, hb, N, Cg, col, inset, headRel, outLineVerts);
    }
}



// == helpers (unchanged) ==
namespace {
inline double vlen(const CgalMeshBuilder::V3& v){
    return std::sqrt(CGAL::to_double(v.squared_length()));
}

inline CgalMeshBuilder::V3 rotate_axis_angle(const CgalMeshBuilder::V3& v,
                                             const CgalMeshBuilder::V3& axis_unit,
                                             double ang){
    if (std::abs(ang) < 1e-15) return v;
    const double c=std::cos(ang), s=std::sin(ang);
    const auto& a = axis_unit;
    return v*c + CGAL::cross_product(a,v)*s + a*CGAL::scalar_product(a,v)*(1.0-c);
}

inline CgalMeshBuilder::V3 orient_from_Z_to_up(const CgalMeshBuilder::V3& p,
                                               const CgalMeshBuilder::V3& up_unit,
                                               double spin){
    CgalMeshBuilder::V3 z(0,0,1);
    double cosang = CGAL::to_double(CGAL::scalar_product(z, up_unit));
    cosang = std::max(-1.0, std::min(1.0, cosang));
    double ang = std::acos(cosang);
    CgalMeshBuilder::V3 axis = vnorm(CGAL::cross_product(z, up_unit));
    CgalMeshBuilder::V3 p1 = (vlen(axis) < 1e-12) ? p : rotate_axis_angle(p, axis, ang);
    return rotate_axis_angle(p1, up_unit, spin);
}

template<class SM>
inline typename SM::Face_index add_ngon_safe(SM& sm, std::vector<typename SM::Vertex_index>& ring){
    if (ring.size() < 3) return SM::null_face();
    auto f = sm.add_face(ring);
    if (f != SM::null_face()) return f;
    std::reverse(ring.begin(), ring.end());
    return sm.add_face(ring);
}
} // namespace


/**
 * @brief Builds a watertight hexsphere by generating the dual of a subdivided icosahedron.
 *
 * This function is completely self-contained. It works by:
 * 1. Creating and subdividing a perfect icosahedron.
 * 2. Calculating the center of each resulting triangle. These centers become the vertices of the hexsphere.
 * 3. For each vertex in the subdivided mesh, it creates a dual face (hex/penta) from the centers of the triangles that surround it.
 * This method guarantees a topologically perfect and watertight mesh.
 */
std::vector<CgalMeshBuilder::F> CgalMeshBuilder::buildHexSphereOriented(
    SM& sm, int resolution, double radius,
    const P3& center, const V3& up, double seamRotate, double mergeEpsRel)
{
    std::vector<F> added;
   // sm.clear();

    // == Step 1: Create a Base Icosahedron ==
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<V3> ico_vertices;
    ico_vertices.emplace_back(-1,  t,  0); ico_vertices.emplace_back( 1,  t,  0);
    ico_vertices.emplace_back(-1, -t,  0); ico_vertices.emplace_back( 1, -t,  0);
    ico_vertices.emplace_back( 0, -1,  t); ico_vertices.emplace_back( 0,  1,  t);
    ico_vertices.emplace_back( 0, -1, -t); ico_vertices.emplace_back( 0,  1, -t);
    ico_vertices.emplace_back( t,  0, -1); ico_vertices.emplace_back( t,  0,  1);
    ico_vertices.emplace_back(-t,  0, -1); ico_vertices.emplace_back(-t,  0,  1);

    for(auto& v : ico_vertices) v = vnorm(v);

    std::vector<std::vector<int>> faces;
    faces.push_back({0, 11, 5}); faces.push_back({0, 5, 1}); faces.push_back({0, 1, 7});
    faces.push_back({0, 7, 10}); faces.push_back({0, 10, 11}); faces.push_back({1, 5, 9});
    faces.push_back({5, 11, 4}); faces.push_back({11, 10, 2}); faces.push_back({10, 7, 6});
    faces.push_back({7, 1, 8}); faces.push_back({3, 9, 4}); faces.push_back({3, 4, 2});
    faces.push_back({3, 2, 6}); faces.push_back({3, 6, 8}); faces.push_back({3, 8, 9});
    faces.push_back({4, 9, 5}); faces.push_back({2, 4, 11}); faces.push_back({6, 2, 10});
    faces.push_back({8, 6, 7}); faces.push_back({9, 8, 1});

    // == Step 2: Subdivide the Icosahedron ==
    std::map<std::pair<int, int>, int> midpoint_cache;
    for (int i = 0; i < resolution; ++i) {
        std::vector<std::vector<int>> new_faces;
        midpoint_cache.clear();
        auto get_midpoint = [&](int i1, int i2) {
            auto key = std::minmax(i1, i2);
            if (midpoint_cache.count(key)) return midpoint_cache[key];
            const V3& v1 = ico_vertices[i1];
            const V3& v2 = ico_vertices[i2];
            ico_vertices.push_back(vnorm(v1 + v2));
            int new_idx = ico_vertices.size() - 1;
            midpoint_cache[key] = new_idx;
            return new_idx;
        };
        for (const auto& face : faces) {
            int v1 = face[0], v2 = face[1], v3 = face[2];
            int m12 = get_midpoint(v1, v2), m23 = get_midpoint(v2, v3), m31 = get_midpoint(v3, v1);
            new_faces.push_back({v1, m12, m31}); new_faces.push_back({v2, m23, m12});
            new_faces.push_back({v3, m31, m23}); new_faces.push_back({m12, m23, m31});
        }
        faces = new_faces;
    }

    // == Step 3: Create the Dual Mesh ==
    const V3 up_unit = vnorm(up);

    // 3a. Calculate face centers, which will be the vertices of our hexsphere.
    std::vector<V> hex_vertices;
    for (const auto& face : faces) {
        V3 v1 = ico_vertices[face[0]], v2 = ico_vertices[face[1]], v3 = ico_vertices[face[2]];
        V3 face_center = vnorm(v1 + v2 + v3);
        V3 oriented_pos = orient_from_Z_to_up(face_center * radius, up_unit, seamRotate);
        P3 final_pos = center + oriented_pos;
        hex_vertices.push_back(sm.add_vertex(final_pos));
    }

    // 3b. Map each ico vertex to the faces (now hex vertices) that surround it.
    std::vector<std::vector<int>> vertex_to_face_map(ico_vertices.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        vertex_to_face_map[faces[i][0]].push_back(i);
        vertex_to_face_map[faces[i][1]].push_back(i);
        vertex_to_face_map[faces[i][2]].push_back(i);
    }

    // 3c. For each ico vertex, create a face from its surrounding face centers.
    for (size_t i = 0; i < ico_vertices.size(); ++i) {
        const auto& face_indices = vertex_to_face_map[i];
        if (face_indices.size() < 3) continue;

        // Order the faces (hex vertices) correctly around the central ico vertex.
        V3 center_v = ico_vertices[i];
        V3 first_v = ico_vertices[faces[face_indices[0]][0]] != center_v ? ico_vertices[faces[face_indices[0]][0]] : ico_vertices[faces[face_indices[0]][1]];
        V3 initial_axis = vnorm(first_v - center_v);

        std::vector<std::pair<double, int>> ordered_faces;
        for (int face_idx : face_indices) {
            V3 p1 = ico_vertices[faces[face_idx][0]];
            V3 p2 = ico_vertices[faces[face_idx][1]];
            V3 p3 = ico_vertices[faces[face_idx][2]];
            V3 face_center_ico = vnorm(p1 + p2 + p3);

            V3 arm = vnorm(face_center_ico - center_v);
            double angle = std::atan2(CGAL::to_double(CGAL::scalar_product(CGAL::cross_product(initial_axis, arm), center_v)),
                                      CGAL::to_double(CGAL::scalar_product(initial_axis, arm)));
            ordered_faces.push_back({angle, face_idx});
        }
        std::sort(ordered_faces.begin(), ordered_faces.end());

        std::vector<V> ring;
        for (const auto& p : ordered_faces) {
            ring.push_back(hex_vertices[p.second]);
        }

        F f = add_ngon_safe(sm, ring);
        if (f != SM::null_face()) {
            added.push_back(f);
        }
    }

    std::cerr << "[hexsphere] build complete: " << num_vertices(sm) << " verts, "
              << num_faces(sm) << " faces." << std::endl;

    return added;
}
