//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Distance between 3D geometric kernel objects.
//

#ifndef CODELIBRARY_GEOMETRY_DISTANCE_3D_H_
#define CODELIBRARY_GEOMETRY_DISTANCE_3D_H_

#include <cmath>

#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/transform_3d.h"

namespace cl {

/**
 * Return the squared Euclidean distance between two 3D points.
 */
template <typename T>
double SquaredDistance(const Point3D<T>& p1, const Point3D<T>& p2) {
    double t1 = p1.x - p2.x;
    double t2 = p1.y - p2.y;
    double t3 = p1.z - p2.z;
    return t1 * t1 + t2 * t2 + t3 * t3;
}

/**
 * Return the Euclidean distance between two 3D points.
 */
template <typename T>
double Distance(const Point3D<T>& p1, const Point3D<T>& p2) {
    return std::sqrt(SquaredDistance(p1, p2));
}

/**
 * Return the signed Euclidean distance from point to plane.
 */
template <typename T>
double SignedDistance(const Point3D<T>& p, const Plane3D<T>& plane) {
    double a = plane.normal().x;
    double b = plane.normal().y;
    double c = plane.normal().z;
    double d = -a * plane.point().x - b * plane.point().y - c * plane.point().z;

    double norm = std::sqrt(a * a + b * b + c * c);
    if (norm == 0.0) return Distance(p, plane.point());

    return (a * p.x + b * p.y + c * p.z + d) / norm;
}
template <typename T>
double SignedDistance(const Plane3D<T>& plane, const Point3D<T>& p) {
    return SignedDistance(p, plane);
}

/**
 * Return the Euclidean distance from point to plane.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Plane3D<T>& plane) {
    return std::fabs(SignedDistance(p, plane));
}
template <typename T>
double Distance(const Plane3D<T>& plane, const Point3D<T>& p) {
    return Distance(p, plane);
}

/**
 * Return the Euclidean distance from point to segment.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Segment3D<T>& segment) {
    if (segment.lower_point() == segment.upper_point()) {
        return Distance(p, segment.lower_point());
    }

    RVector3D v0(segment.upper_point().x - segment.lower_point().x,
                 segment.upper_point().y - segment.lower_point().y,
                 segment.upper_point().z - segment.lower_point().z);
    RVector3D v1(p.x - segment.lower_point().x,
                 p.y - segment.lower_point().y,
                 p.z - segment.lower_point().z);

    double t1 = DotProduct(v0, v1);
    if (t1 < 0.0) return Distance(p, segment.lower_point());

    double t2 = DotProduct(v0, v0);
    if (t2 <= t1) return Distance(p, segment.upper_point());

    double b = t1 / t2;
    RPoint3D pb(b * v0.x + segment.lower_point().x,
                b * v0.y + segment.lower_point().y,
                b * v0.z + segment.lower_point().z);
    RPoint3D pt(p.x, p.y, p.z);

    return Distance(pt, pb);
}
template <typename T>
double Distance(const Segment3D<T>& seg, const Point3D<T>& p) {
    return Distance(p, seg);
}

/**
 * Return the Euclidean distance from point to line.
 */
template <typename T>
double Distance(const Point3D<T>& p, const Line3D<T>& l) {
    RVector3D v0(l.direction().x, l.direction().y, l.direction().z);
    RVector3D v1(p.x - l.point().x, p.y - l.point().y, p.z - l.point().z);

    double t1 = DotProduct(v0, v1);
    double t2 = DotProduct(v0, v0);
    if (t2 == 0.0) return Distance(p, l.point());

    double b = t1 / t2;
    RPoint3D pb(b * v0.x + l.point().x, b * v0.y + l.point().y,
                b * v0.z + l.point().z);

    return Distance(RPoint3D(p.x, p.y, p.z), pb);
}
template <typename T>
double Distance(const Line3D<T>& line, const Point3D<T>& p) {
    return Distance(p, line);
}

/**
 * Return the shortest distance between two lines.
 * It also optional return the pair of the nearest points on two lines.
 */
template <typename T>
double Distance(const Line3D<T>& line1, const Line3D<T>& line2,
                Point3D<T>* point1 = nullptr, Point3D<T>* point2 = nullptr) {
    static_assert(std::is_floating_point<T>::value, "");

    RVector3D v1(line1.direction().x, line1.direction().y,
                 line1.direction().z);
    RVector3D v2(line2.direction().x, line2.direction().y,
                 line2.direction().z);

    RVector3D v = CrossProduct(v1, v2);
    double norm = v.norm();

    if (norm == 0.0) {
        // If two lines are parallel.
        double dis = Distance(line1.point(), line2);
        if (!point1 && !point2) return dis;

        if (point1) *point1 = line1.point();
        if (point2) *point2 = geometry::Project(line1.point(), line2);
        return dis;
    }

    RVector3D ab(static_cast<double>(line1.point().x) - line2.point().x,
                 static_cast<double>(line1.point().y) - line2.point().y,
                 static_cast<double>(line1.point().z) - line2.point().z);
    double dis = -DotProduct(v, ab) / norm;

    if (point1) {
        double t = DotProduct(CrossProduct(-ab, v2), v);
        t /= norm * norm;
        *point1 = line1.point() + line1.direction() * static_cast<T>(t);
    }

    if (point2) {
        double t = DotProduct(CrossProduct(-ab, v1), v);
        t /= norm * norm;
        *point2 = line2.point() + line2.direction() * static_cast<T>(t);
    }

    return dis;
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_DISTANCE_3D_H_
