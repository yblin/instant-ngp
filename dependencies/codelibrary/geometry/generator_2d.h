//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// 2D geometric objects generators.
//

#ifndef CODELIBRARY_GEOMETRY_GENERATOR_2D_H_
#define CODELIBRARY_GEOMETRY_GENERATOR_2D_H_

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <cmath>
#endif // _USE_MATH_DEFINES

#include <queue>
#include <random>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/geometry/vector_2d.h"

namespace cl {
namespace geometry {

/**
 * Randomly generate 2D points in the box.
 */
template <typename T>
class RandomPointInBox2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInBox2D(const Box2D<T>& box)
        : uniform_x_(box.x_min(), box.x_max()),
          uniform_y_(box.y_min(), box.y_max()) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        return Point2D<T>(uniform_x_(engine), uniform_y_(engine));
    }

private:
    // For generating uniform 2D points.
    std::uniform_real_distribution<T> uniform_x_, uniform_y_;
};

/**
 * Randomly generate real point in 2D circle.
 */
template <typename T>
class RandomPointInCircle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInCircle2D(const Circle2D<T>& circle)
        : uniform_angle_(0, T(2) * M_PI),
          uniform_r_(0, T(1)),
          radius_(circle.radius()),
          center_x_(circle.center().x),
          center_y_(circle.center().y) {}

    /**
     * Generate a 2D point in the circle.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T angle = uniform_angle_(engine);
        T sqrt_r = std::sqrt(uniform_r_(engine));
        T x = sqrt_r * std::cos(angle) * radius_;
        T y = sqrt_r * std::sin(angle) * radius_;

        return Point2D<T>(x + center_x_, y + center_y_);
    }

private:
    // For generating uniform angles in range [0, 2 * PI).
    std::uniform_real_distribution<T> uniform_angle_;

    // For generating uniform value in range [0, 1].
    std::uniform_real_distribution<T> uniform_r_;

    // The circle that is used to generate the random points in it.
    T radius_, center_x_, center_y_;
};

/**
 * Randomly generate 2D points (almost) on a 2D circle.
 */
template <typename T>
class RandomPointOnCircle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointOnCircle2D(const Circle2D<T>& circle)
        : uniform_angle_(0, T(2) * M_PI),
          radius_(circle.radius()),
          center_x_(circle.center().x),
          center_y_(circle.center().y) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        // Generate the random point in the unite circle.
        T angle = uniform_angle_(engine);
        T x = std::cos(angle) * radius_ + center_x_;
        T y = std::sin(angle) * radius_ + center_y_;
        return RPoint2D(x, y);
    }

private:
    // For generating uniform angles in range [0, 2PI).
    std::uniform_real_distribution<T> uniform_angle_;

    // The circle that is used to generate the random points in it.
    T radius_, center_x_, center_y_;
};

/**
 * Randomly generate 2D point in the 2D triangle.
 *
 * According to [1], a random point, P, uniformly from within triangle ABC,
 * can be computed by the following convex combination of the vertices:
 *
 *   P = (1 - \sqrt(r1)) A + (\sqrt(r1)(1-r2)) B + (r2 \sqrt(r1)) C
 *
 * where r1, r2 ~ U[0, 1]
 *
 * Note that the generated points just almost on the triangle, not exact.
 *
 * Reference:
 * [1] Osada R, Funkhouser T, Chazelle B, et al. Shape distributions[J]. ACM
 *     Transactions on Graphics, 2002, 21(4):807-832.
 */
template <typename T>
class RandomPointInTriangle2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointInTriangle2D(const Triangle2D<T>& triangle)
        : x1_(triangle.vertices()[0].x),
          x2_(triangle.vertices()[1].x),
          x3_(triangle.vertices()[2].x),
          y1_(triangle.vertices()[0].y),
          y2_(triangle.vertices()[1].y),
          y3_(triangle.vertices()[2].y),
          uniform_(0, 1) {}

    /**
     * Random generate point on the given triangle.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T r1 = uniform_(engine);
        T r2 = uniform_(engine);
        T r1_sqrt = std::sqrt(r1);
        T t1 = 1 - r1_sqrt;
        T t2 = r1_sqrt * (1 - r2);
        T t3 = r2 * r1_sqrt;

        return Point2D<T>(t1 * x1_ + t2 * x2_ + t3 * x3_,
                          t1 * y1_ + t2 * y2_ + t3 * y3_);
    }

private:
    T x1_, x2_, x3_, y1_, y2_, y3_;
    std::uniform_real_distribution<T> uniform_;
};

/**
 * Generate random point (almost) on a 2D line segment.
 */
template <typename T>
class RandomPointOnSegment2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    explicit RandomPointOnSegment2D(const Segment2D<T>& line)
        : uniform_(0, 1),
          direction_(line.direction().x, line.direction().y),
          x_(line.lower_point().x),
          y_(line.lower_point().y) {}

    /**
     * Generate a 2D point.
     */
    template <typename RandomEngine>
    Point2D<T> operator()(RandomEngine& engine) {
        T t = uniform_(engine);
        return Point2D<T>(t * direction_.x + x_, t * direction_.y + y_);
    }

private:
    // Uniform generator.
    std::uniform_real_distribution<T> uniform_;

    // The input segment.
    Vector2D<T> direction_;
    T x_, y_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_GENERATOR_2D_H_
