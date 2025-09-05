#pragma once
#include <vector>
#include <random>
#include <algorithm>
#include <functional>
#include <utility>
#include <cmath>

#include <glm/glm.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

using Kernel      = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3     = Kernel::Point_3;
using Vector_3    = Kernel::Vector_3;
using SurfaceMesh = CGAL::Surface_mesh<Point_3>;

struct CgalMeshBuilderTentacles {
    // Алиасы — поправь под свой тип:
    using SM = SurfaceMesh;                    // CGAL::Surface_mesh<Point_3>
    using F  = SM::Face_index;

    enum class CollisionPolicy {
        Ignore,        // ничего не делаем
        MarkAndStop,   // отметить и остановить рост текущей тентакли
        TryAvoid       // пытаемся отклониться (несколько попыток)
    };

    struct GrowParams {
        int    steps             = 10;          // сколько «шагов» роста на грань
        double distPerStep       = 0.15;        // экструзия вдоль нормали/наклонённой
        double amountPerStep     = 0.75;        // радиальный масштаб (как у тебя)
        double twistMinRad       = -0.5, twistMaxRad = +0.5; // рандом на шаг
        double tiltMinRad        = -0.2, tiltMaxRad  = +0.2;
        uint32_t seed            = 0xC0FFEEu;
        bool collectEachStep     = true;        // sm.collect_garbage() после шага
        bool collectBetweenFaces = true;        // …и между гранями
        CollisionPolicy policy   = CollisionPolicy::Ignore;

        // Настройки попыток уйти от коллизии (если policy == TryAvoid)
        int    avoidTries        = 6;
        double avoidTwistJitter  = 0.35;        // добавка к |twist|
        double avoidTiltJitter   = 0.20;        // добавка к |tilt|
    };

    // Хук на проверку коллизий: верни true, если пересечение найдено.
    // Передаём последние «крышки» (лица), которые только что выросли.
    using CollisionCallback = std::function<bool(const SM&, const std::vector<F>&)>;

    // Построить случайный список стартовых граней (если его нет).
    static std::vector<F> pick_random_faces(const SM& sm, size_t count, uint32_t seed);

    // Главный метод: растим тентакли по очереди.
    // Если seedFaces пуст, можно отобрать случайно через pick_random_faces(...).
    // outCapsPerFace — опционально: «крышки» на каждом шаге для каждой стартовой грани.
    static std::vector<F> extrudeTentaclesSequential(
        SM& sm,
        const std::vector<F>& seedFaces,
        const GrowParams& gp,
        CollisionCallback collision_cb = nullptr,
        std::vector<std::vector<F>>* outCapsPerFace = nullptr
        );

    // Ты уже реализовал это (исправленная версия с каноническим t0):
    // Возвращает «крышки» — новые верхние faces после экструзии.
    static std::vector<F> extrudeFaces_collectBothRotate(
        SM& sm,
        const std::vector<F>& faces,
        double distExtrude,
        double amountExtrude,
        double twist_rad,
        double tilt_rad
        );

    // (Опционально) Пометить, что тентакль столкнулась
    static void mark_tentacle_collided(SM& sm, const std::vector<F>& caps);

    // Добавь в public-секцию класса
    static std::vector<F> add_balls_on_caps(
        SM& sm,
        const std::vector<F>& caps,   // концевые "крышки" тентаклей
        double radius,                // радиус шаров
        int subdivisions = 2,         // 0..3 обычно достаточно
        double center_offset_k = 1.0  // множитель смещения центра: center += n * (k*radius)
        );

    // Утолщение на концах тентаклей "колоколом" без отдельной сферы.
    // baseAmount — базовый масштаб (как обычно: 0.6..0.9),
    // bulgeAmount — добавка в пике (на t=0.5), можно > 1.0 для сильного распухания.
    static std::vector<F> extrudeBulgeOnCaps(
        SM& sm,
        const std::vector<F>& caps,
        int    steps,                // 6..18 обычно хватает
        double distPerStep,          // шаг вдоль нормали
        double baseAmount,           // базовый scale (твоя amountExtrude)
        double bulgeAmount,          // добавка в пике (t=0.5): amount = base + bulge*bell(t)
        double twist_rad  = 0.0,
        double tilt_rad   = 0.0,
        bool   collectEachStep = true
        );


};
