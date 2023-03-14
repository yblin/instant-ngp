//
// Copyright 2015-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_RECTANGLE_2D_H_
#define CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_RECTANGLE_2D_H_

#include <algorithm>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/envelope/convex_hull_2d.h"
#include "codelibrary/geometry/point_compare_2d.h"
#include "codelibrary/geometry/transform_2d.h"

namespace cl {
namespace geometry {

/**
 * Compute minimum area enclosing rectangle by rotating calipers algorithm.
 * The result rectangle is represented by four vertices in counter-clockwise.
 *
 * If the number of input points is less than 3, the return vertices are empty.
 */
template <typename T>
void MinEnclosingRectangle2D(const Array<Point2D<T>>& points,
                             Array<Point2D<T>>* rectangle_vertices) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(rectangle_vertices);

    using Point = Point2D<T>;

    rectangle_vertices->clear();

    // First step: compute the convex hull of input point.
    ConvexHull2D<T> hull(points);
    Polygon2D<T> polygon(hull.vertices());
    if (polygon.size() < 3) return;

    // Second step: do rotating calipers.
    double min_area = DBL_MAX;
    int cur = 0, farthest = 2;
    int left = 0, right = 1;

    // Find the left most vertex.
    PointDotCompare2D<T> compare(polygon[0], polygon[1]);
    while (compare(polygon.prev_vertex(left), polygon[left])) {
        left = polygon.prev_index(left);
    }

    // Find the minimum rectangle by rotating calipers.
    do {
        // Find the farthest point to the current edge.
        PointCrossCompare2D<T> compare1(polygon[cur], polygon.next_vertex(cur));
        while (compare1(polygon[farthest], polygon.next_vertex(farthest))) {
            farthest = polygon.next_index(farthest);
        }

        // Compute the left-most and right-most points under the current rotated
        // coordinate.
        PointDotCompare2D<T> compare2(polygon[cur], polygon.next_vertex(cur));
        while (compare2(polygon.next_vertex(left), polygon[left])) {
            left = polygon.next_index(left);
        }
        while (!compare2(polygon.next_vertex(right), polygon[right])) {
            right = polygon.next_index(right);
        }

        // Now we can determine a rectangle.
        T angle = Radian(polygon[cur].x,
                         polygon[cur].y,
                         polygon.next_vertex(cur).x,
                         polygon.next_vertex(cur).y);
        Array<Point> ps(4);
        ps[0] = Rotate(polygon[cur],      T(2 * M_PI) - angle);
        ps[1] = Rotate(polygon[farthest], T(2 * M_PI) - angle);
        ps[2] = Rotate(polygon[left],     T(2 * M_PI) - angle);
        ps[3] = Rotate(polygon[right],    T(2 * M_PI) - angle);
        Box2D<T> box(ps.begin(), ps.end());
        double area = static_cast<double>(box.x_length()) * box.y_length();
        if (area < min_area) {
            min_area = area;
            ps[0] = Rotate(Point(box.x_min(), box.y_min()), angle);
            ps[1] = Rotate(Point(box.x_max(), box.y_min()), angle);
            ps[2] = Rotate(Point(box.x_max(), box.y_max()), angle);
            ps[3] = Rotate(Point(box.x_min(), box.y_max()), angle);
            *rectangle_vertices = ps;
        }

        // Try the next vertex.
        cur = polygon.next_index(cur);
    } while (cur != 0);
}

/**
 * Compute minimum oriented enclosing rectangle with the given direction angle
 * (in radians).
 * The result rectangle is represented by four vertices in counter-clockwise.
 */
template <typename T>
void MinEnclosingRectangle2D(const Array<Point2D<T>>& points,
                             T angle,
                             Array<Point2D<T>>* rectangle_vertices) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(rectangle_vertices);

    rectangle_vertices->clear();
    if (points.size() < 3) return;

    Array<Point2D<T>> ps(points);
    for (auto& p : ps) {
        p = Rotate(p, T(2 * M_PI) - angle);
    }
    Box2D<T> box(ps.begin(), ps.end());
    rectangle_vertices->resize(4);
    (*rectangle_vertices)[0] = Rotate(Point(box.x_min(), box.y_min()), angle);
    (*rectangle_vertices)[1] = Rotate(Point(box.x_max(), box.y_min()), angle);
    (*rectangle_vertices)[2] = Rotate(Point(box.x_max(), box.y_max()), angle);
    (*rectangle_vertices)[3] = Rotate(Point(box.x_min(), box.y_max()), angle);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ENVELOPE_MIN_ENCLOSING_RECTANGLE_2D_H_
