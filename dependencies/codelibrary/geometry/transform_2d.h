//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_
#define CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_

#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Project point 'p' on a line and return the projection point.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Line2D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point2D<T>& q = line.point();
    const Vector2D<T>& v0 = line.direction();
    Vector2D<T> v1 = p - q;

    T norm = DotProduct(v0, v0);
    if (norm == 0) return q;

    T b = DotProduct(v0, v1) / norm;
    return Point2D<T>(b * v0.x + q.x, b * v0.y + q.y);
}

/**
 * Project point 'p' on a line segment and return the projection point, i.e.,
 * the nearest point on line segment from 'p'.
 */
template <typename T>
Point2D<T> Project(const Point2D<T>& p, const Segment2D<T>& seg) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point2D<T>& q = seg.lower_point();
    const Vector2D<T> v0 = seg.direction();
    Vector2D<T> v1 = p - q;

    T norm = DotProduct(v0, v0);
    if (norm == 0) return q;

    T b = DotProduct(v0, v1) / norm;
    Point2D<T> res(b * v0.x + q.x, b * v0.y + q.y);
    if (seg.lower_point() <= res && res <= seg.upper_point()) return res;
    return SquaredDistance(seg.lower_point(), res) <=
           SquaredDistance(seg.upper_point(), res) ? seg.lower_point()
                                                   : seg.upper_point();
}

/**
 * Translate point.
 */
template <typename T>
Point2D<T> Translate(const Point2D<T>& point, const Vector2D<T>& v) {
    return point + v;
}

/**
 * Translate box according to the given vector.
 */
template <typename T>
Box2D<T> Translate(const Box2D<T>& box, const Vector2D<T>& v) {
    return Box2D<T>(box.x_min() + v.x, box.x_max() + v.x,
                    box.y_min() + v.y, box.y_max() + v.y);
}

/**
 * Translate segment according to the given vector.
 */
template <typename T>
Segment2D<T> Translate(const Segment2D<T>& seg, const Vector2D<T>& v) {
    return Segment2D<T>(seg.lower_point() + v, seg.upper_point() + v);
}

/**
 * Translate line according to the given vector.
 */
template <typename T>
Line2D<T> Translate(const Line2D<T>& line, const Vector2D<T>& v) {
    return Line2D<T>(line.point() + v, line.direction());
}

/**
 * Translate circle.
 */
template <typename T>
Circle2D<T> Translate(const Circle2D<T>& circle, const Vector2D<T>& v) {
    return Circle2D<T>(circle.center() + v, circle.radius());
}

/**
 * Translate polygon.
 */
template <typename T>
Polygon2D<T> Translate(const Polygon2D<T>& polygon, const Vector2D<T>& v) {
    Array<Point2D<T>> points = polygon.vertices();
    for (Point2D<T>& p : points) {
        p = Translate(p, v);
    }
    return Polygon2D<T>(points);
}

/**
 * Scale the line segment.
 */
template <typename T>
Segment2D<T> Scale(const Segment2D<T>& seg, const T& v) {
    return Segment2D<T>(seg.lower_point() * v, seg.upper_point() * v);
}

/**
 * Scale the polygon.
 */
template <typename T>
Polygon2D<T> Scale(const Polygon2D<T>& polygon, const T& v) {
    Array<Point2D<T>> points = polygon.vertices();
    for (Point2D<T>& p : points) {
        p *= v;
    }
    return Polygon2D<T>(points);
}

/**
 * Counterclockwise rotate a point with 'a' radians.
 */
template <typename T>
Point2D<T> Rotate(const Point2D<T>& p, T a) {
    static_assert(std::is_floating_point<T>::value, "");

    T s = std::sin(a), c = std::cos(a);
    T x = c * p.x - s * p.y;
    T y = s * p.x + c * p.y;
    return Point2D<T>(x, y);
}

/**
 * Counterclockwise rotate polygon with 'a' radians.
 */
template <typename T>
Segment2D<T> Rotate(const Segment2D<T>& seg, T a) {
    static_assert(std::is_floating_point<T>::value, "");

    return Segment2D<T>(Rotate(seg.lower_point(), a),
                        Rotate(seg.upper_point()), a);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRANSFORM_2D_H_
