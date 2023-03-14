//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_LINE_3D_H_
#define CODELIBRARY_GEOMETRY_LINE_3D_H_

#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/vector_3d.h"

namespace cl {

/**
 * 3D line class.
 *
 * A 3D line can be uniquely defined by a point and a direction vector.
 */
template <typename T>
class Line3D {
public:
    using value_type = T;

    Line3D() = default;

    /**
     * Construct from a point on the line and direction of line.
     */
    Line3D(const Point3D<T>& p, const Vector3D<T>& direction)
        : point1_(p), point2_(p + direction), direction_(direction) {}

    /**
     * Construct from two distinct points.
     */
    Line3D(const Point3D<T>& p1, const Point3D<T>& p2)
        : point1_(p1), point2_(p2), direction_(p2 - p1) {}

    /**
     * Construct from segment.
     */
    explicit Line3D(const Segment3D<T>& segment)
        : Line3D(segment.lower_point(), segment.upper_point()) {}

    const Vector3D<T>& direction() const {
        return direction_;
    }

    const Point3D<T>& point() const {
        return point1_;
    }

    const Point3D<T>& point1() const {
        return point1_;
    }

    const Point3D<T>& point2() const {
        return point2_;
    }

protected:
    Point3D<T> point1_, point2_; // Two points on the line.
    Vector3D<T> direction_;      // Direction of the line.
};

using ILine3D = Line3D<int>;
using FLine3D = Line3D<float>;
using RLine3D = Line3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_LINE_3D_H_
