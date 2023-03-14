//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_POISSON_DISK_SAMPLE_2D_H_
#define CODELIBRARY_GEOMETRY_UTIL_POISSON_DISK_SAMPLE_2D_H_

#include <queue>

#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/generator_2d.h"
#include "codelibrary/geometry/util/snapper_2d.h"

namespace cl {
namespace geometry {

/**
 * Poisson-Disk sampling for points. It ensures that no two points have a
 * distance smaller than the given resolution after sampling.
 *
 * If the distance between two points is smaller than 'threshold', the later one
 * will be removed.
 */
template <typename T>
void PoissonDiskSample2D(const Array<Point2D<T>>& points, double resolution,
                         Array<Point2D<T>>* samples) {
    CHECK(resolution >= 0.0);
    CHECK(samples);

    Snapper2D<T> snapper(resolution);
    snapper.Reset(points);
    snapper.GetSnapPoints(samples);
}

/**
 * Generate Poisson-Disk points in 2D box.
 *
 * Reference:
 *   Bridson, R. (2007). Fast Poisson disk sampling in arbitrary dimensions.
 *   SIGGRAPH sketches, 10(1), 1.
 */
template <typename T, typename RandomEngine>
void PoissonDiskSample2D(const Box2D<T>& box, double resolution,
                         RandomEngine* random, Array<Point2D<T>>* points) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(resolution > 0.0);
    CHECK(points);

    std::uniform_real_distribution<T> uniform_angle(T(0), T(2.0 * M_PI));
    std::uniform_real_distribution<T> uniform_r(T(0.5), T(1));

    double x = box.x_length() / resolution;
    CHECK(x > 0.0);
    double y = box.y_length() / resolution;
    CHECK(y > 0.0);
    CHECK(x * y < INT_MAX);

    Snapper2D<T> snapper(resolution);
    snapper.Reset(box, static_cast<int>(x + 0.5), static_cast<int>(y + 0.5));

    RandomPointInBox2D<T> ran(box);

    using Point = Point2D<T>;

    const int n_trys = 30;
    std::queue<Point> active_list;
    active_list.push(ran(*random));

    while (!active_list.empty()) {
        Point p = active_list.front();
        active_list.pop();

        for (int k = 0; k < n_trys; ++k) {
            T angle = uniform_angle(*random);
            T sqrt_r = std::sqrt(uniform_r(*random));
            T x = sqrt_r * std::cos(angle) * resolution * 2;
            T y = sqrt_r * std::sin(angle) * resolution * 2;
            Point q(x + p.x, y + p.y);
            if (!Intersect(q, box)) continue;

            Point2D<T> v;
            if (!snapper.FindSnapVertex(q, &v)) {
                active_list.push(q);
                snapper.InsertSnapVertex(q);
                points->push_back(q);
            }
        }
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_POISSON_DISK_SAMPLE_2D_H_
