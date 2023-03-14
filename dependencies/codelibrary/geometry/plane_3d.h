//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_PLANE_3D_H_
#define CODELIBRARY_GEOMETRY_PLANE_3D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * 3D plane class.
 *
 * A 3D plane can be uniquely defined by a point 'o' on plane and a normal
 * vector 'n', i.e., the desired plane can be described as the set of all
 * points r such that:
 *
 *   n (r - o) = 0
 */
template <typename T>
class Plane3D {
    static_assert(std::is_floating_point<T>::value,
                  "We only allow floating point type for 3D plane.");

public:
    using value_type = T;

    Plane3D() = default;

    Plane3D(const Point3D<T>& point, const Vector3D<T>& normal)
        : point_(point), normal_(normal) {}

    /**
     * Construct plane by three points.
     */
    Plane3D(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c) {
        Construct(a, b, c);
    }

    /**
     * Construct plane by three points.
     */
    Plane3D(const Array<Point3D<T>>& points) {
        CHECK(points.size() == 3);

        Construct(points[0], points[1], points[2]);
    }

    const Vector3D<T>& normal() const { return normal_; }
    const Point3D<T>& point()   const { return point_;  }

private:
    /**
     * Construct 3D plane from three points.
     */
    void Construct(const Point3D<T>& a, const Point3D<T>& b,
                   const Point3D<T>& c) {
        Vector3D<T> v1(b - a), v2(c - a);
        normal_ = CrossProduct(v1, v2);
        point_ = a;
    }

    Point3D<T> point_;   // A point on the plane.
    Vector3D<T> normal_; // The normal vector of plane.
};

using FPlane3D = Plane3D<float>;
using RPlane3D = Plane3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_PLANE_3D_H_
