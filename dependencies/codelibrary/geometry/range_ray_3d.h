//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_
#define CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_

#include <algorithm>
#include <limits>
#include <utility>

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/cylinder_3d.h"
#include "codelibrary/geometry/line_3d.h"
#include "codelibrary/geometry/plane_3d.h"
#include "codelibrary/geometry/ray_3d.h"
#include "codelibrary/geometry/segment_3d.h"
#include "codelibrary/geometry/sphere_3d.h"
#include "codelibrary/geometry/triangle_3d.h"

namespace cl {

/**
 * A range ray is a special ray. It is defined by an original point o, a
 * direction vector v, and a range pair p representing the current ray range.
 *
 * Specifically, a range ray (o, v, (l, u)) represents the point set:
 *   {q = o + v * t | t \in [l, u]}
 */
template <typename T>
class RangeRay3D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point3D<T>;
    using Vector = Vector3D<T>;

public:
    /**
     * A default range ray is a invalid ray.
     */
    RangeRay3D() = default;

    explicit RangeRay3D(const Ray3D<T>& ray)
        : origin_(ray.origin()),
          direction_(ray.direction()),
          lower_(0),
          upper_(std::numeric_limits<T>::max()) {}

    explicit RangeRay3D(const Segment3D<T>& seg)
        : origin_(seg.lower_point()),
          direction_(seg.direction()),
          lower_(0),
          upper_(1) {}

    explicit RangeRay3D(const Line3D<T>& line)
        : origin_(line.point()),
          direction_(line.direction()),
          lower_(-std::numeric_limits<T>::max()),
          upper_( std::numeric_limits<T>::max()) {}

    /**
     * Segment defined by (p1, p2).
     */
    RangeRay3D(const Point& p1, const Point& p2)
        : origin_(p1),
          direction_(p2 - p1),
          lower_(0),
          upper_(1) {}

    /**
     * Check if the current ray is empty.
     */
    bool empty() const {
        return lower_ > upper_;
    }

    /**
     * Reset this RangeRay to the overlapping part with a box.
     */
    bool Intersect(const Box3D<T>& box) {
        if (box.empty()) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        for (int i = 0; i < 3; ++i) {
            // It handles the INF case correctly.
            T inverse_direction = 1 / direction_[i];
            T t1 = (box.min(i) - origin_[i]) * inverse_direction;
            T t2 = (box.max(i) - origin_[i]) * inverse_direction;
            if (inverse_direction < 0) std::swap(t1, t2);

            lower_ = std::max(t1, lower_);
            upper_ = std::min(t2, upper_);
        }

        return lower_ <= upper_;
    }

    /**
     * Reset this RangeRay to the overlapping part with a sphere.
     */
    bool Intersect(const Sphere3D<T>& sphere) {
        T a = DotProduct(direction_, direction_);
        if (a == 0) return false;

        Vector diff = sphere.center() - origin_;
        T b = DotProduct(direction_, diff);
        T c = DotProduct(diff, diff) - sphere.radius() * sphere.radius();

        T discriminant = b * b - a * c;
        if (discriminant < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        T root = std::sqrt(discriminant);
        if (b + root < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        lower_ = std::max(lower_, (b - root) / a);
        upper_ = std::min(upper_, (b + root) / a);
        return true;
    }

    /**
     * Reset this RangeRay to the overlapping part with a cylinder.
     */
    bool Intersect(const Cylinder3D<T>& cylinder) {
        // Calculate quadratic equation to solve.
        Vector dp = (cylinder.point2() - cylinder.point1()).Normalize();
        Vector o = origin_ - cylinder.point1();
        const Vector& dir = direction_;

        const T dd = DotProduct(dir, dir);
        const T od = DotProduct(dir, o);
        const T oo = DotProduct(o, o);
        const T dz = DotProduct(dp, dir);
        const T oz = DotProduct(dp, o);

        const T a = dd - dz * dz;
        const T b = 2 * (od - dz * oz);
        const T c = oo - oz * oz - cylinder.radius() * cylinder.radius();

        // We miss the cylinder surface if determinant is smaller than zero.
        const T d = b * b - 4 * a * c;

        if (d < 0) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        const Vector w1 = cylinder.point1() - origin_;
        const Vector w2 = cylinder.point2() - origin_;

        T d1 = DotProduct(dp, direction_);
        T n1 = DotProduct(dp, w1);
        T n2 = DotProduct(dp, w2);
        T t1 = n1 / d1;
        T t2 = n2 / d1;

        T ulp = std::numeric_limits<T>::epsilon();
        std::pair<T, T> r;
        const T eps = 16 * ulp * std::max(std::abs(dd), std::abs(dz * dz));
        if (std::abs(a) < eps) {
            // Special case for rays that are parallel to the cylinder.
            if (c <= 0) {
                lower_ = std::max(lower_, std::min(t1, t2));
                upper_ = std::min(upper_, std::max(t1, t2));
                return lower_ <= upper_;
            } else {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }
        }

        // Standard case for rays that are not parallel to the cylinder.
        const T q = std::sqrt(d);
        const T rcp_2a = 1 / (2 * a);
        lower_ = std::max(lower_, (-b - q) * rcp_2a);
        upper_ = std::min(upper_, (-b + q) * rcp_2a);

        if (std::isfinite(t1) && std::isfinite(t2)) {
            lower_ = std::max(lower_, std::min(t1, t2));
            upper_ = std::min(upper_, std::max(t1, t2));
        } else {
            T t1 = DotProduct(this->lower_point() - cylinder.point1(), dp);
            T t2 = DotProduct(this->lower_point() - cylinder.point2(), -dp);
            if (t1 < 0 || t2 < 0) {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }

            T t3 = DotProduct(this->upper_point() - cylinder.point1(), dp);
            T t4 = DotProduct(this->upper_point() - cylinder.point2(), -dp);
            if (t3 < 0 || t4 < 0) {
                lower_ =  std::numeric_limits<T>::max();
                upper_ = -std::numeric_limits<T>::max();
                return false;
            }
        }

        return lower_ <= upper_;
    }

    /**
     * Check if the range ray hits on an object. It optional returns the nearest
     * hit position.
     */
    bool Hit(const Plane3D<T>& plane, Point* p = nullptr) const {
        const Vector w = plane.point() - origin_;

        T d = DotProduct(plane.normal(), direction_);
        T n = DotProduct(plane.normal(), w);
        T t = n / d;

        if (t >= lower_ && t <= upper_) {
            if (p) *p = origin_ + t * direction_;
            return true;
        }
        return false;
    }
    bool Hit(const Triangle3D<T>& triangle, Point* p = nullptr) const {
        Point r;
        if (!Hit(Plane3D<T>(triangle.vertices()), &r)) return false;

        // We use fast point inside triangle test here.
        Vector o0 = CrossProduct(triangle.vertices()[1] -
                                 triangle.vertices()[0],
                                 r - triangle.vertices()[0]);
        Vector o1 = CrossProduct(triangle.vertices()[2] -
                                 triangle.vertices()[1],
                                 r - triangle.vertices()[1]);
        if (DotProduct(o0, o1) < 0) return false;

        Vector o2 = CrossProduct(triangle.vertices()[0] -
                                 triangle.vertices()[2],
                                 r - triangle.vertices()[2]);
        if (DotProduct(o0, o2) < 0) return false;

        if (p) *p = r;
        return true;
    }

    /**
     * Return the first end point of range ray.
     */
    Point lower_point() const {
        return origin_ + direction_ * lower_;
    }

    /**
     * Return the second end point of range ray.
     */
    Point upper_point() const {
        return origin_ + direction_ * upper_;
    }

    T lower()                 const { return lower_;     }
    T upper()                 const { return upper_;     }
    void set_lower(const T& lower)  { lower_ = lower;    }
    void set_upper(const T& upper)  { upper_ = upper;    }
    const Point& origin()     const { return origin_;    }
    const Vector& direction() const { return direction_; }

private:
    Point origin_;
    Vector direction_;
    T lower_ =  std::numeric_limits<T>::max();
    T upper_ = -std::numeric_limits<T>::max();
};

using IRangeRay3D = RangeRay3D<int>;
using FRangeRay3D = RangeRay3D<float>;
using RRangeRay3D = RangeRay3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_RANGE_RAY_3D_H_
