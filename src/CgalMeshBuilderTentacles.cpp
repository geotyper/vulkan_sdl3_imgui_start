#include "CgalMeshBuilderTentacles.h"

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


#include <unordered_map>

namespace S3 = CGAL::Subdivision_method_3;
namespace PMP = CGAL::Polygon_mesh_processing;
using SM = SurfaceMesh;

using V  = SM::Vertex_index;
using H  = SM::Halfedge_index;
using F  = SM::Face_index;
using E = SM::Edge_index;
using P =SM::Point;

// ---------------------------------------------------------------
// Вспомогательное: перемешать, выбрать count лиц (ненулевых/неудалённых).
// ---------------------------------------------------------------
std::vector<CgalMeshBuilderTentacles::F>
CgalMeshBuilderTentacles::pick_random_faces(const SM& sm, size_t count, uint32_t seed)
{
    std::vector<F> all;
    all.reserve(num_faces(sm));
    for (auto f : sm.faces()) {
        if (!sm.is_removed(f) && f != SM::null_face()) {
            // Можно фильтровать по степени или площади, если нужно
            all.push_back(f);
        }
    }
    std::mt19937 rng(seed);
    std::shuffle(all.begin(), all.end(), rng);
    if (count == 0 || count >= all.size()) return all;
    return std::vector<F>(all.begin(), all.begin() + count);
}

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


//// ---------------------------------------------------------------
//// Главный «роутер» роста: по одной тентакли за раз, каждая — steps шагов.
//// На каждом шаге генерируем новый twist/tilt и экструзим «крышки».
//// При TryAvoid делаем до avoidTries попыток скорректировать углы.
//// ---------------------------------------------------------------
//std::vector<CgalMeshBuilderTentacles::F>
//CgalMeshBuilderTentacles::extrudeTentaclesSequential(
//    SM& sm,
//    const std::vector<F>& seedFaces,
//    const GrowParams& gp,
//    CollisionCallback collision_cb,
//    std::vector<std::vector<F>>* outCapsPerFace)
//{
//    // RNG
//    std::mt19937 rng(gp.seed);
//    std::uniform_real_distribution<double> rndTwist(gp.twistMinRad, gp.twistMaxRad);
//    std::uniform_real_distribution<double> rndTilt (gp.tiltMinRad,  gp.tiltMaxRad);
//
//    // Случайный порядок стартовых граней
//    std::vector<F> order = seedFaces;
//    std::shuffle(order.begin(), order.end(), rng);
//
//    // Все финальные «крышки» (по последнему шагу каждой тентакли)
//    std::vector<F> finalCaps;
//    finalCaps.reserve(order.size());
//
//    if (outCapsPerFace) outCapsPerFace->clear();
//
//    for (size_t fi = 0; fi < order.size(); ++fi) {
//        F f0 = order[fi];
//        if (f0 == SM::null_face() || sm.is_removed(f0)) continue;
//
//        // Текущий фронт роста — начинаем с одной стартовой грани
//        std::vector<F> front = { f0 };
//        bool stoppedByCollision = false;
//
//        // Храним историю крышек для этой тентакли (если нужно наружу)
//        std::vector<F> lastCaps = front;
//
//        for (int step = 0; step < gp.steps; ++step) {
//            // Случайные углы на шаг
//            double twist = rndTwist(rng);
//            double tilt  = rndTilt(rng);
//
//            auto try_step = [&](double tw, double tl, std::vector<F>& front, std::vector<F>& outCaps)->bool {
//                front = extrudeFaces_collectBothRotate(sm, front, gp.distPerStep, gp.amountPerStep, tw, tl);
//                if (collision_cb) {
//                    if (collision_cb(sm, outCaps)) return false; // пересечение — попытка неудачна
//                }
//                return true; // успех (или нет проверки)
//            };
//
//            std::vector<F> caps;
//            bool ok = try_step(twist, tilt, front, caps);
//
//           // if (!ok && gp.policy == CollisionPolicy::TryAvoid) {
//           //     // Пытаемся «уйти» — увеличиваем углы в случайную сторону
//           //     std::uniform_int_distribution<int> sign01(0, 1);
//           //     for (int a = 0; a < gp.avoidTries && !ok; ++a) {
//           //         double tw = twist + (sign01(rng) ? +gp.avoidTwistJitter : -gp.avoidTwistJitter);
//           //         double tl = tilt  + (sign01(rng) ? +gp.avoidTiltJitter  : -gp.avoidTiltJitter);
//           //         ok = try_step(tw, tl, front, caps);
//           //     }
//           // }
//
//           // if (!ok) {
//           //     // Не удалось — политика
//           //     if (gp.policy == CollisionPolicy::MarkAndStop) {
//           //         mark_tentacle_collided(sm, lastCaps);
//           //         stoppedByCollision = true;
//           //     }
//           //     break; // выходим из цикла шагов для этой тентакли
//           // }
//
//            // Обновляем фронт
//            //front = caps;
//            lastCaps = caps;
//
//            //if (gp.collectEachStep) sm.collect_garbage();
//        }
//
//        if (!lastCaps.empty()) {
//            // Возьмём любую «крышку» как представителя финала (или можно добавить все)
//            finalCaps.push_back(front.back());
//        }
//
//        if (outCapsPerFace) outCapsPerFace->push_back(std::move(lastCaps));
//
//        if (gp.collectBetweenFaces) sm.collect_garbage();
//
//        finalCaps.push_back(front[0]);
//    }
//
//    return finalCaps;
//}



// ---------------------------------------------------------------
// Маркер столкновения — заглушка: например, можно покрасить вершины/грани,
// или проставить флаг в property_map. Здесь просто оставлен stub.
// ---------------------------------------------------------------
void CgalMeshBuilderTentacles::mark_tentacle_collided(SM& /*sm*/, const std::vector<F>& /*caps*/)
{
    // TODO: set face property "collided" = true, или окрасить в отладочном выводе.
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




// ———— вспомогательное: собрать кольцо вершин грани (CCW) и их точки ————
static inline void face_ring_vertices(const SM& sm, F f,
                                      std::vector<V>& ringV,
                                      std::vector<Point_3>& ringP)
{
    ringV.clear(); ringP.clear();
    if (f == SM::null_face()) return;
    H h0 = sm.halfedge(f);
    if (h0 == SM::null_halfedge()) return;

    H h = h0;
    std::unordered_set<V> seen;
    do {
        V v = source(h, sm);
        if (v == SM::null_vertex() || !seen.insert(v).second) { ringV.clear(); ringP.clear(); return; }
        ringV.push_back(v);
        ringP.push_back(sm.point(v));
        h = next(h, sm);
    } while (h != h0);
}

// ————  локальный юнит-икосаэдр + сабдив до икосферы ————
static std::vector<std::array<int,3>> build_icosphere(std::vector<Vector_3>& outUnitVerts,
                                                       int subdivisions)
{
    // 12 вершин икосаэдра (ориентация наружу)
    const double phi = (1.0 + std::sqrt(5.0)) * 0.5;
    std::vector<Vector_3> Vloc = {
        { -1,  phi,  0 }, {  1,  phi,  0 }, { -1, -phi,  0 }, {  1, -phi,  0 },
        {  0, -1,  phi }, {  0,  1,  phi }, {  0, -1, -phi }, {  0,  1, -phi },
        {  phi,  0, -1 }, {  phi,  0,  1 }, { -phi,  0, -1 }, { -phi,  0,  1 }
    };

    // нормализуем к единичной сфере
    for (auto& v : Vloc) v = vnorm(v);

    // 20 треугольников икосаэдра
    std::vector<std::array<int,3>> Tris = {
        {0,11,5},  {0,5,1},   {0,1,7},   {0,7,10},  {0,10,11},
        {1,5,9},   {5,11,4},  {11,10,2}, {10,7,6},  {7,1,8},
        {3,9,4},   {3,4,2},   {3,2,6},   {3,6,8},   {3,8,9},
        {4,9,5},   {2,4,11},  {6,2,10},  {8,6,7},   {9,8,1}
    };

    auto key_pair = [](int a, int b)->uint64_t{
        if (a>b) std::swap(a,b);
        return (uint64_t(uint32_t(a))<<32) | uint32_t(b);
    };

    for (int it = 0; it < subdivisions; ++it) {
        std::unordered_map<uint64_t,int> midCache;
        std::vector<std::array<int,3>> Tris2; Tris2.reserve(Tris.size()*4);

        auto mid = [&](int a, int b)->int {
            uint64_t k = key_pair(a,b);
            auto it = midCache.find(k);
            if (it != midCache.end()) return it->second;
            Vector_3 m = vnorm( (Vloc[a] + Vloc[b]) * 0.5 );
            int idx = (int)Vloc.size();
            Vloc.push_back(m);
            midCache.emplace(k, idx);
            return idx;
        };

        for (auto t : Tris) {
            int a = t[0], b = t[1], c = t[2];
            int ab = mid(a,b), bc = mid(b,c), ca = mid(c,a);
            Tris2.push_back({a,  ab, ca});
            Tris2.push_back({b,  bc, ab});
            Tris2.push_back({c,  ca, bc});
            Tris2.push_back({ab, bc, ca});
        }
        Tris.swap(Tris2);
    }

    outUnitVerts = std::move(Vloc);
    return Tris;
}

// ———— добавить одну икосферу в sm вокруг center с радиусом r ————
static std::vector<F> append_icosphere_to_surface_mesh(SM& sm,
                                                       const Point_3& center,
                                                       double r,
                                                       int subdivisions)
{
    std::vector<Vector_3> U;                    // unit vertices
    auto Tris = build_icosphere(U, subdivisions);

    // добавляем вершины
    std::vector<V> Vh; Vh.reserve(U.size());
    for (const auto& u : U) {
        Point_3 p(center.x() + u.x()*r,
                  center.y() + u.y()*r,
                  center.z() + u.z()*r);
        Vh.push_back(sm.add_vertex(p));
    }

    // добавляем треугольники (ориентация наружу сохранена)
    std::vector<F> addedFaces; addedFaces.reserve(Tris.size());
    for (auto t : Tris) {
        F f = sm.add_face(Vh[t[0]], Vh[t[1]], Vh[t[2]]);
        if (f != SM::null_face()) addedFaces.push_back(f);
    }
    return addedFaces;
}

// ———— публичный метод: шары на концах тентаклей (caps) ————
std::vector<CgalMeshBuilderTentacles::F>
CgalMeshBuilderTentacles::add_balls_on_caps(
    SM& sm,
    const std::vector<F>& caps,
    double radius,
    int subdivisions,
    double center_offset_k)
{
    std::vector<F> allNewFaces; allNewFaces.reserve(caps.size() * (20<<subdivisions));

    const Point_3 MC = mesh_centroid(sm); // для ориентации нормали «наружу»

    for (F fcap : caps) {
        if (fcap == SM::null_face() || sm.is_removed(fcap)) continue;

        // центроид и нормаль крышки
        std::vector<V> ringV; std::vector<Point_3> ringP;
        face_ring_vertices(sm, fcap, ringV, ringP);
        if (ringP.size() < 3) continue;

        Point_3  C = centroid_points(ringP);
        Vector_3 n = newell_normal(ringP);
        // пусть нормаль смотрит ВНЕ (от общего центра меша)
        Vector_3 toOut = Vector_3(C.x()-MC.x(), C.y()-MC.y(), C.z()-MC.z());
        if (CGAL::to_double(CGAL::scalar_product(n, toOut)) < 0) n = -n;
        Vector_3 nu = vnorm(n);

        // центр шара чуть вперед по нормали, чтобы шар касался торца
        Point_3 S(C.x() + nu.x() * (center_offset_k * radius),
                  C.y() + nu.y() * (center_offset_k * radius),
                  C.z() + nu.z() * (center_offset_k * radius));

        // сам шар
        auto facesOfBall = append_icosphere_to_surface_mesh(sm, S, radius, subdivisions);
        allNewFaces.insert(allNewFaces.end(), facesOfBall.begin(), facesOfBall.end());
    }

    // можно прошить/подлечить, если нужно:
    // PMP::remove_isolated_vertices(sm);
    // sm.collect_garbage(); // если хочется сразу подчистить

    return allNewFaces;
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

// гарантирует ориентированный a->b: сначала ищем, иначе создаём ровно ОДНУ пару
static inline H ensure_oriented_halfedge(SM& sm, V a, V b) {
    if (a == b) return SM::null_halfedge();

    if (H h = find_halfedge_sm_only(sm, a, b); h != SM::null_halfedge())
        return h;

    if (H hb = find_halfedge_sm_only(sm, b, a); hb != SM::null_halfedge())
        return sm.opposite(hb);

    // ребра нет — создаём 1 пару halfedge'ов
    H h = sm.add_edge(a, b);              // возвращает половинку с target==b
    if (h == SM::null_halfedge()) return SM::null_halfedge();

    // необязательно, но полезно: один раз проставим опорные hv у вершин, если пусто
    if (sm.halfedge(b) == SM::null_halfedge()) sm.set_halfedge(b, h);
    H ho = sm.opposite(h);
    if (sm.halfedge(a) == SM::null_halfedge()) sm.set_halfedge(a, ho);

    return h;                              // ориентирован как a->b
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

static inline void set_vertex_incoming_if_any(SM& sm, V v, H prefer=SM::null_halfedge()){
    if (v==SM::null_vertex()) return;
    if (prefer!=SM::null_halfedge() && sm.target(prefer)==v){ sm.set_halfedge(v,prefer); return; }
    H hv = sm.halfedge(v);
    if (hv!=SM::null_halfedge() && sm.target(hv)==v) return;
    for (H h : sm.halfedges()) if (!sm.is_removed(h) && sm.target(h)==v){ sm.set_halfedge(v,h); return; }
}



struct ExtrudeLists {
    std::vector<SurfaceMesh::Face_index> all;   // всё, что создано
    std::vector<SurfaceMesh::Face_index> caps;  // только новая(ые) крышка(и)
};

struct FacePlan {
    std::uint64_t fuid = 0;
    std::vector<std::uint64_t> ring_vuid;  // CCW по исходной грани
    std::vector<char>          wasBorder;  // по каждому ребру (i->i+1) на момент слепка
};

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

std::vector<F> CgalMeshBuilderTentacles::extrudeFaces_collectBothRotate(
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



// ---- shared step: build plans for current front (caps) and extrude each cap ----
static inline std::vector<CgalMeshBuilderTentacles::F>
step_extrude_front(
    SM& sm,
    const std::vector<F>& front,      // faces (caps) to advance
    double distPerStep,
    double amount,
    double twist_rad,
    double tilt_rad,
    SM::Property_map<V,std::uint64_t>& vuid,
    SM::Property_map<F,std::uint64_t>& fuid,
    std::uint64_t& next_vuid,
    std::uint64_t& next_fuid,
    std::unordered_map<std::uint64_t,V>& vByUid,
    std::unordered_map<std::uint64_t,F>& fByUid,
    ExtrudeLists* dump)
{
    // plans from current caps
    std::vector<FacePlan> plans;
    build_face_plans_snapshot(sm, front, vuid, fuid, plans);
    if (plans.empty()) return {};

    // recompute centroid per step (robust on big edits)
    const Point_3 MC = mesh_centroid(sm);

    std::vector<F> caps; caps.reserve(plans.size());
    for (const FacePlan& p : plans) {
        F cap = extrude_face_from_plan_uidRotate(
            sm, p,
            distPerStep, amount,
            MC,
            vuid, fuid, next_vuid, next_fuid,
            vByUid, fByUid,
            dump,
            twist_rad, tilt_rad
            );
        if (cap != SM::null_face()) caps.push_back(cap);

        // mesh changed — refresh maps
        rebuild_vertex_uid_map(sm, vuid, vByUid);
        rebuild_face_uid_map(sm, fuid, fByUid);
    }
    return caps;
}

std::vector<CgalMeshBuilderTentacles::F>
CgalMeshBuilderTentacles::extrudeBulgeOnCaps(
    SM& sm,
    const std::vector<F>& caps,
    int    steps,
    double distPerStep,
    double baseAmount,
    double bulgeAmount,
    double twist_rad,
    double tilt_rad,
    bool   collectEachStep)
{
    std::vector<F> result;                 // all final caps from all seeds
    if (steps <= 0 || caps.empty()) return result;

    // UID maps once
    SM::Property_map<V,std::uint64_t> vuid;
    SM::Property_map<F,std::uint64_t> fuid;
    std::uint64_t next_vuid = 0, next_fuid = 0;
    ensure_uid_maps_and_assign_all(sm, vuid, fuid, next_vuid, next_fuid);

    std::unordered_map<std::uint64_t,V> vByUid; rebuild_vertex_uid_map(sm, vuid, vByUid);
    std::unordered_map<std::uint64_t,F> fByUid; rebuild_face_uid_map(sm, fuid, fByUid);

    ExtrudeLists dump; // optional
    const double PI = std::acos(-1.0);

    // -------- iterate ON CAPS (outer) ----------
    for (F seedCap : caps) {
        if (seedCap == SM::null_face() || sm.is_removed(seedCap)) continue;

        std::vector<F> front{ seedCap };   // grow this cap independently

        for (int i = 0; i < steps; ++i) {
            const double t   = (steps == 1) ? 0.5 : double(i) / double(steps - 1);
            const double amt = baseAmount + bulgeAmount * std::sin(PI * t);

            // one step for this cap (front has 1 face, but keep it generic)
            std::vector<FacePlan> plans;
            build_face_plans_snapshot(sm, front, vuid, fuid, plans);
            if (plans.empty()) break;

            const Point_3 MC = mesh_centroid(sm);

            std::vector<F> newFront;
            newFront.reserve(plans.size());
            for (const FacePlan& p : plans) {
                F cap = extrude_face_from_plan_uidRotate(
                    sm, p, distPerStep, amt, MC,
                    vuid, fuid, next_vuid, next_fuid,
                    vByUid, fByUid, &dump,
                    twist_rad, tilt_rad
                    );
                if (cap != SM::null_face()) newFront.push_back(cap);

                // mesh changed — refresh maps
                rebuild_vertex_uid_map(sm, vuid, vByUid);
                rebuild_face_uid_map(sm, fuid, fByUid);
            }

            if (newFront.empty()) break;
            front.swap(newFront);
           // if (collectEachStep) sm.collect_garbage();
        }

        // collect final caps from this seed
        result.insert(result.end(), front.begin(), front.end());
    }
    // -------------------------------------------

    return result;
}



std::vector<CgalMeshBuilderTentacles::F>
CgalMeshBuilderTentacles::extrudeTentaclesSequential(
    SM& sm,
    const std::vector<F>& seedFaces,
    const GrowParams& gp,
    CollisionCallback collision_cb,
    std::vector<std::vector<F>>* outCapsPerFace)
{
    // RNG
    std::mt19937 rng(gp.seed);
    std::uniform_real_distribution<double> rndTwist(gp.twistMinRad, gp.twistMaxRad);
    std::uniform_real_distribution<double> rndTilt (gp.tiltMinRad,  gp.tiltMaxRad);

    // Перемешаем порядок стартовых граней
    std::vector<F> order = seedFaces;
    std::shuffle(order.begin(), order.end(), rng);

    // UID-карты и словари — создаём один раз
    SM::Property_map<V, std::uint64_t> vuid;
    SM::Property_map<F, std::uint64_t> fuid;
    std::uint64_t next_vuid = 0, next_fuid = 0;
    ensure_uid_maps_and_assign_all(sm, vuid, fuid, next_vuid, next_fuid);

    std::unordered_map<std::uint64_t, V> vByUid; rebuild_vertex_uid_map(sm, vuid, vByUid);
    std::unordered_map<std::uint64_t, F> fByUid; rebuild_face_uid_map(sm, fuid, fByUid);

    std::vector<F> finalCaps;
    finalCaps.reserve(order.size());
    if (outCapsPerFace) outCapsPerFace->clear();

    // Буфер для (опционального) сбора стенок; можно убрать
    ExtrudeLists dump;

    for (F f0 : order) {
        if (f0 == SM::null_face() || sm.is_removed(f0)) continue;

        // Фронт: текущие "крышки" для этой тентакли
        std::vector<F> front{ f0 };
        std::vector<F> lastCaps = front;

        for (int step = 0; step < gp.steps; ++step) {
            const double twist = rndTwist(rng);
            const double tilt  = rndTilt (rng);

            // Построим планы только по текущему фронту
            std::vector<FacePlan> plans;
            build_face_plans_snapshot(sm, front, vuid, fuid, plans);
            if (plans.empty()) break;

            // Центр меша можно пересчитывать на шаг (стабильнее для больших изменений)
            const Point_3 MC = mesh_centroid(sm);

            std::vector<F> caps; caps.reserve(plans.size());

            // Экструдим по каждому плану => собираем новые "крышки"
            for (const FacePlan& p : plans) {
                F cap = extrude_face_from_plan_uidRotate(
                    sm, p,
                    gp.distPerStep, gp.amountPerStep,
                    MC,
                    vuid, fuid, next_vuid, next_fuid,
                    vByUid, fByUid,
                    &dump,
                    twist, tilt
                    );
                if (cap != SM::null_face())
                    caps.push_back(cap);

                // После изменения графа — обновим словари
                rebuild_vertex_uid_map(sm, vuid, vByUid);
                rebuild_face_uid_map(sm, fuid, fByUid);
            }

            // Коллизии проверяем именно на новых caps
            if (collision_cb && collision_cb(sm, caps)) {
                // тут можно реализовать TryAvoid/MarkAndStop по gp.policy
                break;
            }

            if (caps.empty()) break;

            lastCaps = caps;
            front.swap(caps);

           // if (gp.collectEachStep)
           //     sm.collect_garbage();
        }

        // Финальные крышки этой тентакли — текущий фронт
        if (!front.empty())
            finalCaps.insert(finalCaps.end(), front.begin(), front.end());

        if (outCapsPerFace)
            outCapsPerFace->push_back(std::move(lastCaps));

       // if (gp.collectBetweenFaces) sm.collect_garbage();
    }

    return finalCaps;
}
