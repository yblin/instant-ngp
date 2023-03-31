//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_RANGE_RAY_2D_H_
#define CODELIBRARY_GEOMETRY_RANGE_RAY_2D_H_

#include <algorithm>
#include <limits>
#include <utility>

#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/ray_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {

/**
 * A range ray is a special ray. It is defined by an original point o, a
 * direction vector v, and a range pair p representing the current ray range.
 *
 * Specifically, a range ray (o, v, (l, u)) represents the point set:
 *   {q = o + v * t | t \in [l, u]}
 */
template <typename T>
class RangeRay2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;
    using Vector = Vector2D<T>;

public:
    /**
     * A default range ray is a invalid ray.
     */
    RangeRay2D() = default;

    explicit RangeRay2D(const Ray2D<T>& ray)
        : origin_(ray.origin()),
          direction_(ray.direction()),
          lower_(0),
          upper_(std::numeric_limits<T>::max()) {}

    explicit RangeRay2D(const Segment2D<T>& seg)
        : origin_(seg.lower_point()),
          direction_(seg.direction()),
          lower_(0),
          upper_(1) {}

    explicit RangeRay2D(const Line2D<T>& line)
        : origin_(line.point()),
          direction_(line.direction()),
          lower_(-std::numeric_limits<T>::max()),
          upper_( std::numeric_limits<T>::max()) {}

    /**
     * Segment defined by (p1, p2).
     */
    RangeRay2D(const Point& p1, const Point& p2)
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
    bool Intersect(const Box2D<T>& box) {
        if (empty() || box.empty()) return false;

        for (int i = 0; i < 2; ++i) {
            // It handles the inf cases correct.
            T inverse_direction = 1 / direction_[i];
            T t1 = (box.min(i) - origin_[i]) * inverse_direction;
            T t2 = (box.max(i) - origin_[i]) * inverse_direction;
            if (inverse_direction < 0) std::swap(t1, t2);

            lower_ = std::max(lower_, t1);
            upper_ = std::min(upper_, t2);

            if (upper_ < lower_) return false;
        }

        return true;
    }

    /**
     * Reset this RangeRay to the overlapping part with a circle.
     */
    bool Intersect(const Circle2D<T>& circle) {
        if (empty()) return false;

        Vector u = circle.center() - origin_;

        T norm = direction_.norm();
        T norm2 = norm * norm;
        if (Equal(norm2, 0)) {
            if (u.norm() <= circle.radius()) return true;

            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        T t = DotProduct(direction_, u);
        Vector u1 = t / norm2 * direction_;
        T d = (u - u1).norm();
        if (d > circle.radius()) {
            lower_ =  std::numeric_limits<T>::max();
            upper_ = -std::numeric_limits<T>::max();
            return false;
        }

        T m = std::sqrt(circle.radius() * circle.radius() - d * d);
        T u1_norm = u1.norm();
        if (t < 0) u1_norm = -u1_norm;
        lower_ = std::max((u1_norm - m) / norm, lower_);
        upper_ = std::min((u1_norm + m) / norm, upper_);
        return lower_ <= upper_;
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

using IRangeRay2D = RangeRay2D<int>;
using FRangeRay2D = RangeRay2D<float>;
using RRangeRay2D = RangeRay2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_RANGE_RAY_2D_H_
