//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Computer the center of geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_CENTER_3D_H_
#define CODELIBRARY_GEOMETRY_CENTER_3D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/segment_3d.h"

namespace cl {

/**
 * Compute the centroid of two points.
 */
template <typename T>
Point3D<T> Center(const Point2D<T>& p1, const Point2D<T>& p2) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point2D<T>((p1.x + p2.x) * T(0.5), (p1.y + p2.y) * T(0.5));
}template <typename T>
Point3D<T> Center(const Point3D<T>& p1, const Point3D<T>& p2) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>((p1.x + p2.x) * T(0.5), (p1.y + p2.y) * T(0.5),
                      (p1.z + p2.z) * T(0.5));
}

/**
 * Compute the centroid of a bounding box.
 */
template <typename T>
Point3D<T> Center(const Box2D<T>& box) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>((box.x_min() + box.x_max()) * T(0.5),
                      (box.y_min() + box.y_max()) * T(0.5));
}
template <typename T>
Point3D<T> Center(const Box3D<T>& box) {
    static_assert(std::is_floating_point<T>::value, "");

    return Point3D<T>((box.x_min() + box.x_max()) * T(0.5),
                      (box.y_min() + box.y_max()) * T(0.5),
                      (box.z_min() + box.z_max()) * T(0.5));
}

/**
 * Compute the centroid of a line segment.
 */
template <typename T>
Point2D<T> Center(const Segment2D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    return Center(line.lower_point(), line.upper_point());
}
template <typename T>
Point3D<T> Center(const Segment3D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    return Center(line.lower_point(), line.upper_point());
}

/**
 * Compute the centroid of a point set.
 */
template <typename Point>
Point Center(const Array<Point>& points) {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!points.empty());

    const int dimension = points.front().size();

    Array<T> v(dimension, 0);
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < dimension; ++j) {
            v[j] += points[i][j];
        }
    }

    return Point(v.begin(), v.end()) * (T(1) / points.size());
}

/**
 * Compute the weighted centroid of a point set.
 */
template <typename Point, typename T>
Point Center(const Array<Point>& points, const Array<T>& weights) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(!points.empty());
    CHECK(points.size() == weights.size());

    const int dimension = points.front().size();

    T sum = 0;
    Array<T> v(dimension, 0);
    for (int i = 0; i < points.size(); ++i) {
        T w = weights[i];
        for (int j = 0; j < dimension; ++j) {
            v[j] += w * points[i][j];
        }
        sum += w;
    }
    CHECK(sum != 0);

    return Point(v.begin(), v.end()) * (T(1) / sum);
}

/**
 * Circumcenter of three 2D points.
 */
template <typename T>
Point2D<T> Circumcenter(const Point2D<T>& p1, const Point2D<T>& p2,
                        const Point2D<T>& p3) {
    static_assert(std::is_floating_point<T>::value, "");

    // Use 'double' to reduce floating point error.
    T x1 = p1.x, y1 = p1.y, x2 = p2.x, y2 = p2.y, x3 = p3.x, y3 = p3.y;
    T x0 = 0.5 * ((y3 - y1) * (y2 * y2 - y1 * y1) +
                  (y3 - y1) * (x2 * x2 - x1 * x1) -
                  (y1 - y2) * (y1 * y1 - y3 * y3) -
                  (y1 - y2) * (x1 * x1 - x3 * x3)) /
                 ((y1 - y2) * (x3 - x1) - (y3 - y1) * (x1 - x2));
    T y0 = (y3 * y3 - y1 * y1 - 2.0 * x0 * (x3 - x1) - x1 * x1 + x3 * x3) /
           (2.0 * (y3 - y1));
    return Point2D<T>(x0, y0);
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CENTER_3D_H_
