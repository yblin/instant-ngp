//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_
#define CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_

#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/quaternion.h"
#include "codelibrary/geometry/sphere_3d.h"
#include "codelibrary/math/matrix/matrix4.h"

namespace cl {
namespace geometry {

/**
 * Return the orthogonal projection of a point 'p' on a line.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Line3D<T>& line) {
    static_assert(std::is_floating_point<T>::value, "");

    const Point3D<T>& q = line.point();
    const Vector3D<T>& v = line.direction();

    Vector3D<T> v0(v.x, v.y, v.z);
    T norm = DotProduct(v0, v0);
    if (norm == 0) return line.point();

    Vector3D<T> v1 = p - q;

    T b = DotProduct(v0, v1) / norm;
    return Point3D<T>(b * v0.x + q.x, b * v0.y + q.y, b * v0.z + q.z);
}

/**
 * Return the orthogonal projection of a point 'p' on a plane.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Plane3D<T>& plane) {
    static_assert(std::is_floating_point<T>::value, "");

    Line3D<T> line(p, plane.normal());
    return Project(plane.point(), line);
}

/**
 * Return the projection of a point 'p' on a sphere.
 */
template <typename T>
Point3D<T> Project(const Point3D<T>& p, const Sphere3D<T>& sphere) {
    static_assert(std::is_floating_point<T>::value, "");

    T radius = sphere.radius();

    if (p == sphere.center()) {
        // If p is the center of sphere, return the point that has with maximal
        // Z value.
        return sphere.center() + Vector3D<T>(0, 0, radius);
    }

    Vector3D<T> v = p - sphere.center();
    v *= radius / v.norm();

    return sphere.center() + v;
}

/**
 * Translate point.
 */
template <typename T>
Point3D<T> Translate(const Point3D<T>& point, const Vector3D<T>& v) {
    return point + v;
}

/**
 * Translate box according to the given vector.
 */
template <typename T>
Box3D<T> Translate(const Box3D<T>& box, const Vector3D<T>& v) {
    return Box3D<T>(box.x_min() + v.x, box.x_max() + v.x,
                    box.y_min() + v.y, box.y_max() + v.y,
                    box.z_min() + v.z, box.z_max() + v.z);
}

/**
 * Translate segment according to the given vector.
 */
template <typename T>
Segment3D<T> Translate(const Segment3D<T>& seg, const Vector3D<T>& v) {
    return Segment3D<T>(Translate(seg.lower_point(), v),
                        Translate(seg.upper_point(), v));
}

/**
 * Translate line according to the given vector.
 */
template <typename T>
Line3D<T> Translate(const Line3D<T>& line, const Vector3D<T>& v) {
    return Line3D<T>(line.point() + v, line.direction());
}

/**
 * Translate sphere.
 */
template <typename T>
Sphere3D<T> Translate(const Sphere3D<T>& sphere, const Vector3D<T>& v) {
    return Sphere3D<T>(sphere.center() + v, sphere.radius());
}

/**
 * Translate plane.
 */
template <typename T>
Plane3D<T> Translate(const Plane3D<T>& plane, const Vector3D<T>& v) {
    return Plane3D<T>(plane.point() + v, plane.normal());
}

/**
 * Rotate the given point.
 */
template <typename T>
Point3D<T> Rotate(const Point3D<T>& point, const Quaternion<T>& rotation) {
    static_assert(std::is_floating_point<T>::value, "");

    Quaternion<T> q(point.x, point.y, point.z, 0);
    q = rotation * q * rotation.inverse();

    return Point3D<T>(q.x, q.y, q.z);
}

/**
 * Rotate a given vector.
 */
template <typename T>
Vector3D<T> Rotate(const Vector3D<T>& v, const Quaternion<T>& rotation) {
    static_assert(std::is_floating_point<T>::value, "");

    return rotation.Rotate(v);
}

/**
 * Rotate the given plane.
 */
template <typename T>
Plane3D<T> Rotate(const Plane3D<T>& plane, const Quaternion<T>& rotation) {
    return Plane3D<T>(Rotate(plane.point(), rotation),
                      rotation.Rotate(plane.normal()));
}

/**
 * Rotate the given segment.
 */
template <typename T>
Segment3D<T> Rotate(const Segment3D<T>& seg, const Quaternion<T>& rotation) {
    return Segment3D<T>(Rotate(seg.lower_point(), rotation),
                        Rotate(seg.upper_point(), rotation));
}

/**
 * Rotate the given line.
 */
template <typename T>
Line3D<T> Rotate(const Line3D<T>& line, const Quaternion<T>& rotation) {
    return Line3D<T>(Rotate(line.point(), rotation),
                     rotation.Rotate(line.direction()));
}

/**
 * Rigid transform: x = Rx + T.
 *
 * The input matrix should be in the form of [R, T]
 *                                           [0, 1].
 */
template <typename T>
Point3D<T> Rigid(const Point3D<T>& p, const Matrix4<T>& mat) {
    Point3D<T> q;
    q.x = mat[0] * p.x + mat[1] * p.y + mat[2]  * p.z + mat[3];
    q.y = mat[4] * p.x + mat[5] * p.y + mat[6]  * p.z + mat[7];
    q.z = mat[8] * p.x + mat[9] * p.y + mat[10] * p.z + mat[11];
    return q;
}
template <typename T>
Vector3D<T> Rigid(const Vector3D<T>& v, const Matrix4<T>& mat) {
    Vector3D<T> q;
    q.x = mat[0] * v.x + mat[1] * v.y + mat[2]  * v.z;
    q.y = mat[4] * v.x + mat[5] * v.y + mat[6]  * v.z;
    q.z = mat[8] * v.x + mat[9] * v.y + mat[10] * v.z;
    return q;
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TRANSFORM_3D_H_
