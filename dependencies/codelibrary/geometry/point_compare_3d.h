//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_
#define CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_

#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/predicate_3d.h"

namespace cl {
namespace geometry {

/**
 * Exact dot comparison for points.
 *
 * lhs < rhs means lhs is before rhs along line (p1, p2).
 */
template <typename T>
class PointDotCompare3D {
public:
    PointDotCompare3D(const Point3D<T>& p1, const Point3D<T>& p2)
        : point1_(p1), point2_(p2) {}

    bool operator() (const Point3D<T>& lhs, const Point3D<T>& rhs) const {
        return DotProductCompare(point1_, point2_, lhs, rhs) < 0;
    }

private:
    /**
     * Compute the dot product value: (q - p)(s - p).
     */
    template<typename T1>
    static T1 DotProduct(const T1& px, const T1& py, const T1& pz,
                         const T1& qx, const T1& qy, const T1& qz,
                         const T1& sx, const T1& sy, const T1& sz) {
        return (qx - px) * (sx - px) + (qy - py) * (sy - py) +
               (qz - pz) * (sz - pz);
    }

    /**
     * Exact compare (q - p)(s - p) with (q - p)(r - p).
     */
    static int DotProductCompare(const Point3D<T>& p, const Point3D<T>& q,
                                 const Point3D<T>& s, const Point3D<T>& r) {
        // Interval filter.
        if (fegetround() != FE_UPWARD) fesetround(FE_UPWARD);

        IntervalFloat det1 = DotProduct(IntervalFloat(p.x), IntervalFloat(p.y),
                                        IntervalFloat(p.z),
                                        IntervalFloat(q.x), IntervalFloat(q.y),
                                        IntervalFloat(q.z),
                                        IntervalFloat(s.x), IntervalFloat(s.y),
                                        IntervalFloat(s.z));
        IntervalFloat det2 = DotProduct(IntervalFloat(p.x), IntervalFloat(p.y),
                                        IntervalFloat(p.z),
                                        IntervalFloat(q.x), IntervalFloat(q.y),
                                        IntervalFloat(q.z),
                                        IntervalFloat(r.x), IntervalFloat(r.y),
                                        IntervalFloat(r.z));
        det1 -= det2;
        if (det1.lower() > 0.0) return +1;
        if (det1.upper() < 0.0) return -1;
        if (det1.lower() == 0.0 && det1.upper() == 0.0) return 0;

        ExactFloat e1 = DotProduct(ExactFloat(p.x), ExactFloat(p.y),
                                   ExactFloat(p.z),
                                   ExactFloat(q.x), ExactFloat(q.y),
                                   ExactFloat(q.z),
                                   ExactFloat(s.x), ExactFloat(s.y),
                                   ExactFloat(s.z));
        ExactFloat e2 = DotProduct(ExactFloat(p.x), ExactFloat(p.y),
                                   ExactFloat(p.z),
                                   ExactFloat(q.x), ExactFloat(q.y),
                                   ExactFloat(q.z),
                                   ExactFloat(r.x), ExactFloat(r.y),
                                   ExactFloat(r.z));
        e1 -= e2;
        return e1.sign();
    }

    Point3D<T> point1_, point2_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POINT_COMPARE_3D_H_
