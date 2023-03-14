//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Robust intersection test between 3D kernel geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_INTERSECT_3D_H_
#define CODELIBRARY_GEOMETRY_INTERSECT_3D_H_

#include <algorithm>
#include <limits>
#include <utility>

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/predicate_3d.h"
#include "codelibrary/geometry/range_ray_3d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/triangle_3d.h"

namespace cl {
namespace geometry {

////////////////////////////////////////////////////////////////////////////////
// All Intersect() functions are used to test weather two objects intersect
// (including touch and contain).
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication and construction.
 */
template <typename T>
bool Intersect(const Box3D<T>& box1, const Box3D<T>& box2,
               Box3D<T>* res = nullptr) {
    if (box1.x_max() < box2.x_min() || box1.x_min() > box2.x_max() ||
        box1.y_max() < box2.y_min() || box1.y_min() > box2.y_max() ||
        box1.z_max() < box2.z_min() || box1.z_min() > box2.z_max())
        return false;

    if (res) {
        *res = Box3D<T>(std::max(box1.x_min(), box2.x_min()),
                        std::min(box1.x_max(), box2.x_max()),
                        std::max(box1.y_min(), box2.y_min()),
                        std::min(box1.y_max(), box2.y_max()),
                        std::max(box1.z_min(), box2.z_min()),
                        std::min(box1.z_max(), box2.z_max()));
    }
    return true;
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point3D<T>& point, const Box3D<T>& box) {
    return (point.x >= box.x_min() && point.x <= box.x_max() &&
            point.y >= box.y_min() && point.y <= box.y_max() &&
            point.z >= box.z_min() && point.z <= box.z_max());
}
template <typename T>
bool Intersect(const Box3D<T>& box, const Point3D<T>& point) {
    return Intersect(point, box);
}

/**
 * Inexact predication.
 */
template <typename T>
bool Intersect(const Box3D<T>& box, const Plane3D<T>& plane) {
    static_assert(std::is_floating_point<T>::value, "");

    if (box.empty()) return false;

    // Convert AABB to center-extents representation.
    Point3D<T> c = Center(box);
    // Compute positive extents
    Point3D<T> e(box.x_max() - c.x, box.y_max() - c.y, box.z_max() - c.z);

    const Vector3D<T>& n = plane.normal();
    const Point3D<T>& p = plane.point();

    // Compute the projection interval radius of box onto Line(t) = c + t * n.
    T r = e.x * std::fabs(n.x) + e.y * std::fabs(n.y) + e.z * std::fabs(n.z);

    // Convert plane to the form of Ax + By + Cz + d = 0.
    T d = -n.x * p.x - n.y * p.y - n.z * p.z;

    // Compute distance of box center from plane.
    T s = n.x * c.x + n.y * c.y + n.z * c.z + d;

    // Intersection occurs when distance s falls within [-r,+r] interval.
    return std::fabs(s) <= r;
}
template <typename T>
bool Intersect(const Plane3D<T>& plane, const Box3D<T>& box) {
    return Intersect(box, plane);
}

////////////////////////////////////////////////////////////////////////////////
// All Cross() functions are used to test weather two objects are cross
// (excluding touch and contain).
//
// Exact construction requires high-precision computation, which can greatly
// slow down the speed. In this library, we completely avoid such operations.
////////////////////////////////////////////////////////////////////////////////
/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether two planes are cross is unnecessary in most cases.
 * Because it is very rare that two planes are strictly parallel.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane1, const Plane3D<T>& plane2,
           Line3D<T>* res) {
    static_assert(std::is_floating_point<T>::value, "");

    const Vector3D<T>& v1 = plane1.normal();
    const Vector3D<T>& v2 = plane2.normal();
    const Vector3D<T> u = CrossProduct(v1, v2);
    T as[3] = { std::fabs(u.x), std::fabs(u.y), std::fabs(u.z) };

    // First, determine the maximum absolute value of cross production.
    int maxc = static_cast<int>(std::max_element(as + 0, as + 3) - as);

    // Next, compute a point on the intersection line.
    Point3D<T> p;
    T d1 = -DotProduct(v1, plane1.point().ToVector());
    T d2 = -DotProduct(v2, plane2.point().ToVector());

    switch (maxc) {
    case 0: // Intersected with plane x = 0.
        p.x = 0;
        p.y = (d2 * v1.z - d1 * v2.z) / u.x;
        p.z = (d1 * v2.y - d2 * v1.y) / u.x;
        break;
    case 1: // Intersected with plane y = 0.
        p.x = (d1 * v2.z - d2 * v1.z) / u.y;
        p.y = 0;
        p.z = (d2 * v1.x - d1 * v2.x) / u.y;
        break;
    case 2: // Intersected with plane z = 0.
        p.x = (d2 * v1.y - d1 * v2.y) / u.z;
        p.y = (d1 * v2.x - d2 * v1.x) / u.z;
        p.z = 0;
    }

    if (res) *res = Line3D<T>(p, u);
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/**
 * Exact predication whether a line segment is cross with a plane determined by
 * three points.
 */
template <typename T>
bool Cross(const Point3D<T>& a, const Point3D<T>& b, const Point3D<T>& c,
           const Segment3D<T>& seg) {
    int o1 = Orientation(a, b, c, seg.lower_point());
    if (o1 == 0) return false;
    int o2 = Orientation(a, b, c, seg.upper_point());
    if (o2 == 0) return false;
    return o1 != o2;
}

/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether a plane cross with a line is unnecessary in most
 * cases. Because it is very rare for a plane an a line to be strictly parallel.
 * If there is a real need, it is better to convert lines into long line
 * segments and then use the above function.
 */
template <typename T>
bool Cross(const Plane3D<T>& plane, const Line3D<T>& line,
           Point3D<T>* res = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    const Vector3D<T> w = plane.point() - line.point1();
    const Vector3D<T> direction = line.point2() - line.point1();

    T d = DotProduct(plane.normal(), direction);
    T n = DotProduct(plane.normal(), w);
    T t = n / d;
    Point3D<T> p = line.point1() + t * line.direction();

    if (std::isfinite(p.x) && std::isfinite(p.y)) {
        if (res) *res = p;
        return true;
    }

    return false;
}
template <typename T>
bool Cross(const Line3D<T>& line, const Plane3D<T>& plane,
           Point3D<T>* res = nullptr) {
    return Cross(plane, line, res);
}

/**
 * Inexact predication.
 *
 * Reference:
 *  Akenine-Möller, Tomas. "Fast 3D triangle-box overlap testing." ACM siggraph
 *  2005 courses. 2005. 8-es.
 */
template <typename T>
bool Cross(const Box3D<T>& box, const Triangle3D<T>& triangle) {
    static_assert(std::is_floating_point<T>::value, "");

    // First: test If the triangle is entirely inside the bounding box.
    if (Intersect(box, triangle.vertices()[0]) &&
        Intersect(box, triangle.vertices()[1]) &&
        Intersect(box, triangle.vertices()[2])) return true;

    // Convert AABB to center-extents form.
    Point3D<T> box_center = Center(box);
    Vector3D<T> box_halfsize(box.x_length() / 2, box.y_length() / 2,
                             box.z_length() / 2);

    // Move everything so that the boxcenter is in (0,0,0).
    Vector3D<T> v0 = triangle.vertices()[0] - box_center;
    Vector3D<T> v1 = triangle.vertices()[1] - box_center;
    Vector3D<T> v2 = triangle.vertices()[2] - box_center;

    // Compute the edge vectors of the triangle  (ABC)
    Vector3D<T> e0 = v1 - v0;
    Vector3D<T> e1 = v2 - v1;
    Vector3D<T> e2 = v0 - v2;


    T p0, p1, p2, d;

    // 9 tests for axis overlapping

    // Axis test x01
    Vector3D<T> fe(std::fabs(e0.x), std::fabs(e0.y), std::fabs(e0.z));
    p0 = e0.z * v0.y - e0.y * v0.z;
    p2 = e0.z * v2.y - e0.y * v2.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test y02
    p0 = -e0.z * v0.x + e0.x * v0.z;
    p2 = -e0.z * v2.x + e0.x * v2.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test z12
    p1 = e0.y * v1.x - e0.x * v1.y;
    p2 = e0.y * v2.x - e0.x * v2.y;
    d = fe.y * box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p1, p2) > d || std::max(p1, p2) < -d) return false;

    fe = Vector3D<T>(std::fabs(e1.x), std::fabs(e1.y), std::fabs(e1.z));
    // Axis test x01
    p0 = e1.z * v0.y - e1.y * v0.z;
    p2 = e1.z * v2.y - e1.y * v2.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test y02
    p0 = -e1.z * v0.x + e1.x * v0.z;
    p2 = -e1.z * v2.x + e1.x * v2.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p2) > d || std::max(p0, p2) < -d) return false;

    // Axis test z0
    p0 = e1.y * v0.x - e1.x * v0.y;
    p1 = e1.y * v1.x - e1.x * v1.y;
    d = fe.y* box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    fe = Vector3D<T>(std::fabs(e2.x), std::fabs(e2.y), std::fabs(e2.z));

    // Axis test x02
    p0 = e2.z * v0.y - e2.y * v0.z;
    p1 = e2.z * v1.y - e2.y * v1.z;
    d = fe.z * box_halfsize.y + fe.y * box_halfsize.z;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    // Axis test y01
    p0 = -e2.z * v0.x + e2.x * v0.z;
    p1 = -e2.z * v1.x + e2.x * v1.z;
    d = fe.z * box_halfsize.x + fe.x * box_halfsize.z;
    if (std::min(p0, p1) > d || std::max(p0, p1) < -d) return false;

    // Axis test z12
    p1 = e2.y * v1.x - e2.x * v1.y;
    p2 = e2.y * v2.x - e2.x * v2.y;
    d = fe.y * box_halfsize.x + fe.x * box_halfsize.y;
    if (std::min(p1, p2) > d || std::max(p1, p2) < -d) return false;

    // Test in X-direction.
    if (std::min(std::min(v0.x, v1.x), v2.x) >  box_halfsize.x ||
        std::max(std::max(v0.x, v1.x), v2.x) < -box_halfsize.x) return false;

    // Test in Y-direction.
    if (std::min(std::min(v0.y, v1.y), v2.y) >  box_halfsize.y ||
        std::max(std::max(v0.y, v1.y), v2.y) < -box_halfsize.y) return false;

    // Test in Z-direction.
    if (std::min(std::min(v0.z, v1.z), v2.z) >  box_halfsize.z ||
        std::max(std::max(v0.z, v1.z), v2.z) < -box_halfsize.z) return false;

    // Test if the box intersects the plane of the triangle.
    Plane3D<T> plane(triangle.vertices());
    if (!Intersect(plane, box)) return false;

    return true;
}
template <typename T>
bool Cross(const Triangle3D<T>& triangle, const Box3D<T>& box) {
    return Cross(box, triangle);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_INTERSECT_3D_H_
