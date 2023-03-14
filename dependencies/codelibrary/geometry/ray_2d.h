//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_RAY_2D_H_
#define CODELIBRARY_GEOMETRY_RAY_2D_H_

#include "codelibrary/geometry/segment_2d.h"

namespace cl {

/**
 * A 2D ray can be uniquely defined by its origin point and direction vector.
 */
template <typename T>
class Ray2D {
public:
    using value_type = T;

    Ray2D() = default;

    /**
     * Construct from a origin point and a direction vector.
     */
    Ray2D(const Point2D<T>& origin, const Vector2D<T>& direction)
        : origin_(origin), direction_(direction) {}

    /**
     * Construct from two distinct points.
     */
    Ray2D(const Point2D<T>& p1, const Point2D<T>& p2)
        : Ray2D(p1, p2 - p1) {}

    const Vector2D<T>& direction() const {
        return direction_;
    }

    const Point2D<T>& origin() const {
        return origin_;
    }

private:
    Point2D<T> origin_;     // The origin point of the ray.
    Vector2D<T> direction_; // The direction vector of line.
};

using IRay2D = Ray2D<int>;
using FRay2D = Ray2D<float>;
using RRay2D = Ray2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_RAY_2D_H_
