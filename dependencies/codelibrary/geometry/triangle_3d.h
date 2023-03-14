//
// Copyright 2018-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRIANGLE_3D_H_
#define CODELIBRARY_GEOMETRY_TRIANGLE_3D_H_

#include <cmath>

#include "codelibrary/base/log.h"
#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * 3D triangle.
 */
template <typename T>
class Triangle3D {
public:
    Triangle3D()
        : vertices_(3) {}

    Triangle3D(const Point3D<T>& p1, const Point3D<T>& p2,
               const Point3D<T>& p3)
        : vertices_(3) {
        vertices_[0] = p1;
        vertices_[1] = p2;
        vertices_[2] = p3;
    }

    explicit Triangle3D(const Array<Point3D<T>>& vertices)
        : vertices_(vertices) {
        CHECK(vertices.size() == 3);
    }

    /**
     * Return the normal vector of the triangle.
     */
    Vector3D<T> normal() const {
        static_assert(std::is_floating_point<T>::value, "");

        return CrossProduct(vertices_[1] - vertices_[0],
                            vertices_[2] - vertices_[0]);
    }

    /**
     * Return the area of the triangle.
     */
    double Area() const {
        RVector3D v0(vertices_[1].x - vertices_[0].x,
                     vertices_[1].y - vertices_[0].y,
                     vertices_[1].z - vertices_[0].z);
        RVector3D v1(vertices_[2].x - vertices_[0].x,
                     vertices_[2].y - vertices_[0].y,
                     vertices_[2].z - vertices_[0].z);
        RVector3D v = CrossProduct(v0, v1);
        return 0.5 * v.norm();
    }

    const Array<Point3D<T>>& vertices() const {
        return vertices_;
    }

    Box3D<T> bounding_box() const {
        return Box3D<T>(vertices_.begin(), vertices_.end());
    }

private:
    Array<Point3D<T>> vertices_;
};

using ITriangle3D = Triangle3D<int>;
using FTriangle3D = Triangle3D<float>;
using RTriangle3D = Triangle3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRIANGLE_3D_H_
