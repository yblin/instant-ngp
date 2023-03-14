//
// Copyright 2018-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_
#define CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {

/**
 * 2D triangle.
 */
template <typename T>
class Triangle2D {
public:
    Triangle2D()
        : vertices_(3) {}

    Triangle2D(const Point2D<T>& p1, const Point2D<T>& p2,
               const Point2D<T>& p3)
        : vertices_(3) {
        vertices_[0] = p1;
        vertices_[1] = p2;
        vertices_[2] = p3;
    }

    explicit Triangle2D(const Array<Point2D<T>>& vertices)
        : vertices_(vertices) {
        CHECK(vertices.size() == 3);
    }

    /**
     * Return the area of the triangle.
     */
    double Area() {
        double t1 = static_cast<double>(vertices_[0].x) * vertices_[2].y -
                    static_cast<double>(vertices_[2].x) * vertices_[1].y;
        double t2 = static_cast<double>(vertices_[2].x) * vertices_[0].y -
                    static_cast<double>(vertices_[0].x) * vertices_[2].y;
        double t3 = static_cast<double>(vertices_[0].x) * vertices_[1].y -
                    static_cast<double>(vertices_[1].x) * vertices_[0].y;
        return 0.5 * std::sqrt(t1 * t1 + t2 * t2 + t3 * t3);
    }

    const Array<Point2D<T>>& vertices() const {
        return vertices_;
    }

    Box2D<T> bounding_box() const {
        return Box2D<T>(vertices_.begin(), vertices_.end());
    }

private:
    Array<Point2D<T>> vertices_;
};

using ITriangle2D = Triangle2D<int>;
using FTriangle2D = Triangle2D<float>;
using RTriangle2D = Triangle2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRIANGLE_2D_H_
