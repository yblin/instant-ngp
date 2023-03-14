//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BEZIER_CURVE_3D_H_
#define CODELIBRARY_GEOMETRY_BEZIER_CURVE_3D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {

/**
 * A Bezier curve is a type of parametric curve.
 *
 * It is defined by control points and satisfy the following properties:
 *
 * 1. The point on curve is always inside the convex hull of control points.
 * 2. The bezier curve always passes through the first and last control points
 * 3. The curve is tangent to p_1 - p_0 and p_n - p_(n-1) at the endpoints.
 */
template <typename T>
class BezierCurve3D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point3D<T>;

public:
    /**
     * A default Bezier curve is an empty curve.
     */
    BezierCurve3D() = default;

    /**
     * Construct Bezier curve by control points. The number of control points
     * should be no less than 2.
     */
    BezierCurve3D(const Array<Point>& control_points)
        : control_points_(control_points) {
        CHECK(control_points_.size() >= 2);
    }

    /**
     * Split a Bezier curve into two curves for the given ratio t.
     *
     * We use De Casteljau's algorithm here. Although the algorithm is slower
     * for most architectures when compared with the direct approach, it is more
     * numerically stable.
     */
    void Split(const T& t, BezierCurve3D* curve1, BezierCurve3D* curve2) const {
        CHECK(t >= 0 && t <= 1);
        CHECK(curve1 && curve2);

        curve1->control_points_.clear();
        curve2->control_points_.clear();

        if (empty()) return;

        Array<Point> points = control_points_;
        while (points.size() > 1) {
            curve1->control_points_.push_back(points.front());
            curve2->control_points_.push_back(points.back());
            for (int i = 0; i + 1 < points.size(); ++i) {
                points[i] = GetRatioPoint(points[i], points[i + 1], t);
            }
            points.pop_back();
        }

        curve1->control_points_.push_back(points.front());
        curve2->control_points_.push_back(points.back());
        std::reverse(curve2->control_points_.begin(),
                     curve2->control_points_.end());
    }

    /**
     * Get the point on curve at ratio t.
     */
    Point GetCurvePoint(const T& t) const {
        CHECK(!empty());
        CHECK(t >= 0 && t <= 1);

        if (t == 0) return control_points_.front();
        if (t == 1) return control_points_.back();

        Array<Point> points = control_points_;
        while (points.size() > 1) {
            for (int i = 0; i + 1 < points.size(); ++i) {
                points[i] = GetRatioPoint(points[i], points[i + 1], t);
            }
            points.pop_back();
        }

        return points.front();
    }

    /**
     * Get n curve points.
     * n should be no less than 2.
     */
    void GetCurvePoints(int n, Array<Point>* points) const {
        CHECK(n >= 2);
        CHECK(!empty());
        CHECK(points);

        points->clear();
        for (int i = 0; i < n; ++i) {
            T t = static_cast<T>(i) / (n - 1);
            points->push_back(GetCurvePoint(t));
        }
    }

    /**
     * Return the degree of the curve.
     */
    int size() const {
        return control_points_.size();
    }

    bool empty() const {
        return size() < 2;
    }

    /**
     * Return the control points.
     */
    const Array<Point>& control_points() {
        return control_points_;
    }

protected:
    /**
     * Get a point at ratio t between p1 and p2.
     */
    static Point GetRatioPoint(const Point& p1, const Point& p2, const T& t) {
        return p1 + (p2 - p1) * t;
    }

    Array<Point> control_points_;
};

using FBezierCurve3D = BezierCurve3D<float>;
using RBezierCurve3D = BezierCurve3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BEZIER_CURVE_3D_H_
