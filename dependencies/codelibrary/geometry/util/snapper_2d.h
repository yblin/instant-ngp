//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_SNAPPER_2D_H_
#define CODELIBRARY_GEOMETRY_SNAPPER_2D_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_compare_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D snap system.
 */
template <typename T>
class Snapper2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;

public:
    Snapper2D(double threshold)
        : threshold_(threshold) {
        CHECK(threshold_ >= 0.0);
    }

    /**
     * Reset Snapper2D with the bounding box and the dimension of the grids.
     */
    void Reset(const Box2D<T>& box, int n_x_grids, int n_y_grids) {
        CHECK(!box.empty());
        CHECK(n_x_grids > 0);
        CHECK(n_y_grids > 0);
        CHECK(INT_MAX / n_x_grids >= n_y_grids);

        box_ = box;
        x_resolution_ = box_.x_length() / n_x_grids;
        y_resolution_ = box_.y_length() / n_y_grids;
        grid_.clear();
        grid_.reshape(n_x_grids, n_y_grids);
    }

    /**
     * Reset Snapper2D with the given incident points.
     */
    void Reset(const Array<Point>& points) {
        CHECK(!points.empty());

        box_ = Box2D<T>(points.begin(), points.end());
        double sqrt_n = std::ceil(std::sqrt(points.size()));

        int n1 = sqrt_n, n2 = sqrt_n;
        if (threshold_ > 0.0) {
            n1 = static_cast<int>(std::min(box_.x_length() / threshold_,
                                           sqrt_n));
            n2 = static_cast<int>(std::min(box_.y_length() / threshold_,
                                           sqrt_n));
        }
        CHECK(n1 > 0 && n2 > 0);

        x_resolution_ = box_.x_length() / n1;
        y_resolution_ = box_.y_length() / n2;
        grid_.clear();
        grid_.reshape(n1, n2);

        for (const Point& p : points) {
            InsertSnapVertex(p);
        }
    }

    /**
     * Insert a point 'p' as a snap vertex.
     *
     * If there is no snap vertex whose distance to 'p' smaller than threshold,
     * insert 'p' as a new snap vertex and return it. Otherwise, return the
     * existing snap vertex.
     */
    Point InsertSnapVertex(const Point& p) {
        CHECK(!grid_.empty());

        int n1 = grid_.shape(0), n2 = grid_.shape(1);
        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y);

        Point res;
        double dis = DBL_MAX;
        for (int x1 = -1; x1 <= 1; ++x1) {
            for (int y1 = -1; y1 <= 1; ++y1) {
                int x2 = x1 + g_x, y2 = y1 + g_y;
                if (x2 < 0 || x2 >= n1 || y2 < 0 || y2 >= n2) continue;

                for (const Point& q : grid_(x2, y2)) {
                    if (q == p) return q;
                    double d = SquaredDistance(p, q);
                    if (d <= dis) {
                        dis = d;
                        res = q;
                    }
                }
            }
        }
        if (dis <= threshold_ * threshold_) return res;

        grid_(g_x, g_y).push_back(p);
        return p;
    }

    /**
     * Find a snap vertex whose distance to 'p' is not greater than threshold.
     * If there are many candidate snap vertices, store the nearest one.
     *
     * It returns false if no vertex in snapper has a distance to 'p' smaller
     * than threshold.
     */
    bool FindSnapVertex(const Point& p, Point* snap_v) const {
        CHECK(!grid_.empty());
        CHECK(snap_v);

        int n1 = grid_.shape(0), n2 = grid_.shape(1);
        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y);

        double dis = DBL_MAX;
        for (int x1 = -1; x1 <= 1; ++x1) {
            for (int y1 = -1; y1 <= 1; ++y1) {
                int x2 = x1 + g_x, y2 = y1 + g_y;
                if (x2 < 0 || x2 >= n1 || y2 < 0 || y2 >= n2) continue;

                for (const Point& q : grid_(x2, y2)) {
                    if (q == p) {
                        *snap_v = q;
                        return true;
                    }
                    double d = SquaredDistance(p, q);
                    if (d <= dis) {
                        dis = d;
                        *snap_v = q;
                    }
                }
            }
        }
        return (dis <= threshold_ * threshold_);
    }

    /**
     * Find all snap vertices in box.
     */
    void FindSnapVertices(const Box2D<T>& box, Array<Point>* vertices) const {
        CHECK(vertices);

        int x_min = GetXIndex(box.x_min());
        int x_max = GetXIndex(box.x_max());
        int y_min = GetYIndex(box.y_min());
        int y_max = GetYIndex(box.y_max());

        vertices->clear();
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                for (const Point& p : grid_(x, y)) {
                    if (Intersect(box, p)) {
                        vertices->push_back(p);
                    }
                }
            }
        }
    }

    /**
     * Given a line segment (p, q), find the snapped polyline.
     */
    void FindSnapSegment(const Point& p, const Point& q, Array<Point>* line) {
        CHECK(line);

        Box2D<T> box(std::min(p.x, q.x) - threshold_,
                     std::max(p.x, q.x) + threshold_,
                     std::min(p.y, q.y) - threshold_,
                     std::max(p.y, q.y) + threshold_);
        Array<Point> vertices;
        FindSnapVertices(box, &vertices);

        line->clear();

        Segment2D<T> seg(p, q);
        for (const Point& p : vertices) {
            if (Distance(p, seg) <= threshold_) {
                line->push_back(p);
            }
        }
        PointDotCompare2D<T> compare(p, q);
        std::sort(line->begin(), line->end(), compare);
    }

    /**
     * Get all snap points in the Snapper2D.
     *
     * It is guaranteed that the distance between any pair of points is greater
     * than the set threshold.
     */
    void GetSnapPoints(Array<Point>* points) const {
        CHECK(points);

        points->clear();
        for (const auto& g : grid_) {
            points->insert(g);
        }
    }

    /**
     * Return the x-axis index of the value 'v'.
     */
    int GetXIndex(T v) const {
        int n = grid_.shape(0);
        return Clamp((v - box_.x_min()) / x_resolution_, 0.0, n - 1.0);
    }

    /**
     * Return the y-axis index of the value 'v'.
     */
    int GetYIndex(T v) const {
        int n = grid_.shape(1);
        return Clamp((v - box_.y_min()) / y_resolution_, 0.0, n - 1.0);
    }

    const Box2D<T>& box() const {
        return box_;
    }

    double threshold() const {
        return threshold_;
    }

private:
    // 2D spatial grid for fast access.
    ArrayND<Array<Point>> grid_;

    // Bounding box of vertices.
    Box2D<T> box_;

    // Threshld for snapping.
    double threshold_;

    // X and Y resolution.
    double x_resolution_, y_resolution_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_SNAPPER_2D_H_
