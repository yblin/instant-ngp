//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_
#define CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_

#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/predicate_2d.h"

namespace cl {
namespace geometry {

/**
 * Exact dot comparison for points.
 *
 * lhs < rhs means lhs is before rhs along line (p1, p2).
 */
template <typename T>
class PointDotCompare2D {
public:
    PointDotCompare2D(const Point2D<T>& p1, const Point2D<T>& p2)
        : point1_(p1), point2_(p2) {}

    PointDotCompare2D(const Line2D<T>& line)
        : point1_(line.point1()), point2_(line.point2()) {}

    PointDotCompare2D(const Segment2D<T>& seg)
        : point1_(seg.lower_point()), point2_(seg.upper_point()) {}

    bool operator() (const Point2D<T>& lhs, const Point2D<T>& rhs) const {
        return DotProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Compute the dot product value: (q - p)(s - p).
     */
    template<typename T1>
    static T1 DotProduct(const T1& px, const T1& py,
                         const T1& qx, const T1& qy,
                         const T1& sx, const T1& sy) {
        return (qx - px) * (sx - px) + (qy - py) * (sy - py);
    }

    /**
     * Exact compare (q - p)(s - p) with (q - p)(r - p).
     */
    static int DotProductCompare(const Point2D<T>& p, const Point2D<T>& q,
                                 const Point2D<T>& s, const Point2D<T>& r) {
        // Interval filter.
        if (fegetround() != FE_UPWARD) fesetround(FE_UPWARD);

        IntervalFloat det1 = DotProduct(IntervalFloat(p.x), IntervalFloat(p.y),
                                        IntervalFloat(q.x), IntervalFloat(q.y),
                                        IntervalFloat(s.x), IntervalFloat(s.y));
        IntervalFloat det2 = DotProduct(IntervalFloat(p.x), IntervalFloat(p.y),
                                        IntervalFloat(q.x), IntervalFloat(q.y),
                                        IntervalFloat(r.x), IntervalFloat(r.y));
        det1 -= det2;
        if (det1.lower() > 0.0) return +1;
        if (det1.upper() < 0.0) return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0) return 0;

        ExactFloat e1 = DotProduct(ExactFloat(p.x), ExactFloat(p.y),
                                   ExactFloat(q.x), ExactFloat(q.y),
                                   ExactFloat(s.x), ExactFloat(s.y));
        ExactFloat e2 = DotProduct(ExactFloat(p.x), ExactFloat(p.y),
                                   ExactFloat(q.x), ExactFloat(q.y),
                                   ExactFloat(r.x), ExactFloat(r.y));
        e1 -= e2;
        return e1.sign();
    }

    Point2D<T> point1_, point2_;
};

/**
 * Exact cross compare for points.
 *
 * lhs < rhs means lhs is before rhs along the normal vector of line (p1, p2).
 */
template <typename T>
class PointCrossCompare2D {
public:
    PointCrossCompare2D(const Point2D<T>& p1, const Point2D<T>& p2)
        : point1_(p1), point2_(p2) {}

    bool operator() (const Point2D<T>& lhs, const Point2D<T>& rhs) const {
        return CrossProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Exact compare (q - p) x (s - p) with (q - p) x (r - p).
     */
    static int CrossProductCompare(const Point2D<T>& p, const Point2D<T>& q,
                                   const Point2D<T>& s, const Point2D<T>& r) {
        // Interval filter.
        if (fegetround() != FE_UPWARD) fesetround(FE_UPWARD);

        IntervalFloat det1 =
                OrientationDeterminant(IntervalFloat(p.x), IntervalFloat(p.y),
                                       IntervalFloat(q.x), IntervalFloat(q.y),
                                       IntervalFloat(s.x), IntervalFloat(s.y));
        IntervalFloat det2 =
                OrientationDeterminant(IntervalFloat(p.x), IntervalFloat(p.y),
                                       IntervalFloat(q.x), IntervalFloat(q.y),
                                       IntervalFloat(r.x), IntervalFloat(r.y));
        det1 -= det2;
        if (det1.lower() > 0.0) return +1;
        if (det1.upper() < 0.0) return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0) return 0;

        ExactFloat det3 =
                OrientationDeterminant(ExactFloat(p.x), ExactFloat(p.y),
                                       ExactFloat(q.x), ExactFloat(q.y),
                                       ExactFloat(s.x), ExactFloat(s.y));
        ExactFloat det4 =
                OrientationDeterminant(ExactFloat(p.x), ExactFloat(p.y),
                                       ExactFloat(q.x), ExactFloat(q.y),
                                       ExactFloat(r.x), ExactFloat(r.y));
        det3 -= det4;
        return det3.sign();
    }

    Point2D<T> point1_, point2_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POINT_COMPARE_2D_H_
