//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_SNAP_3D_H_
#define CODELIBRARY_GEOMETRY_SNAP_3D_H_

#include <cfloat>

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/geometry/point_compare_3d.h"

namespace cl {
namespace geometry {

/**
 * 3D snap system.
 */
template <typename T>
class Snap3D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point3D<T>;

public:
    explicit Snap3D(double threshold)
        : threshold_(threshold) {
        CHECK(threshold_ >= 0.0);
    }

    /**
     * Reset Snap3D with the bounding box and the dimension of the grids.
     */
    void Reset(const Box3D<T>& box, int n_x, int n_y, int n_z) {
        CHECK(!box.empty());
        CHECK(n_x > 0);
        CHECK(n_y > 0);
        CHECK(n_z > 0);
        CHECK(INT_MAX / n_x >= n_y);
        CHECK(INT_MAX / n_x / n_y >= n_z);

        box_ = box;
        x_resolution_ = box_.x_length() / n_x;
        y_resolution_ = box_.y_length() / n_y;
        z_resolution_ = box_.z_length() / n_z;
        grid_.clear();
        grid_.reshape(n_x, n_y, n_z);
    }

    /**
     * Reset Snap3D with the given incident points.
     */
    void Reset(const Array<Point>& points) {
        CHECK(!points.empty());

        box_ = Box3D<T>(points.begin(), points.end());
        double n = std::ceil(std::pow(points.size(), 1.0 / 3.0));

        int n1 = n, n2 = n, n3 = n;
        if (threshold_ > 0.0) {
            n1 = Clamp(double(n1), 1.0, box_.x_length() / threshold_);
            n2 = Clamp(double(n2), 1.0, box_.y_length() / threshold_);
            n3 = Clamp(double(n3), 1.0, box_.z_length() / threshold_);
        }

        x_resolution_ = box_.x_length() / n1;
        y_resolution_ = box_.y_length() / n2;
        z_resolution_ = box_.z_length() / n3;
        grid_.clear();
        grid_.reshape(n1, n2, n3);

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

        int n1 = grid_.shape(0), n2 = grid_.shape(1), n3 = grid_.shape(2);
        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y), g_z = GetZIndex(p.z);

        Point res;
        if (threshold_ == 0.0) {
            for (const Point& q : grid_(g_x, g_y, g_z)) {
                if (q == p) return q;
            }
            grid_(g_x, g_y, g_z).push_back(p);
            return p;
        }

        int lx = GetXIndex(p.x - threshold_), ux = GetXIndex(p.x + threshold_);
        int ly = GetYIndex(p.y - threshold_), uy = GetYIndex(p.y + threshold_);
        int lz = GetYIndex(p.y - threshold_), uz = GetYIndex(p.y + threshold_);
        double dis = DBL_MAX;
        for (int x1 = lx; x1 <= ux; ++x1) {
            for (int y1 = ly; y1 <= uy; ++y1) {
                for (int z1 = lz; z1 <= uz; ++z1) {
                    int x2 = x1 + g_x, y2 = y1 + g_y, z2 = z1 + g_z;
                    if (x2 < 0 || x2 >= n1 || y2 < 0 || y2 >= n2 ||
                        z2 < 0 || z2 >= n3) continue;

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
        }
        if (dis <= threshold_ * threshold_) return res;

        grid_(g_x, g_y).push_back(p);
        return p;
    }

    /**
     * Find a snap vertex whose distance to 'p' is not greater than threshold.
     * If there are many candidate snap vertices, return the nearest one.
     *
     * It returns false if no vertex in snap has a distance to 'p' smaller
     * than threshold.
     */
    bool FindSnapVertex(const Point& p, Point* snap_v) const {
        CHECK(!grid_.empty());
        CHECK(snap_v);

        int n1 = grid_.shape(0), n2 = grid_.shape(1), n3 = grid_.shape(2);
        int g_x = GetXIndex(p.x), g_y = GetYIndex(p.y), g_z = GetZIndex(p.z);

        if (threshold_ == 0.0) {
            for (const Point& q : grid_(g_x, g_y, g_z)) {
                if (q == p) return true;
            }
            return false;
        }

        int lx = GetXIndex(p.x - threshold_), ux = GetXIndex(p.x + threshold_);
        int ly = GetYIndex(p.y - threshold_), uy = GetYIndex(p.y + threshold_);
        int lz = GetYIndex(p.z - threshold_), uz = GetYIndex(p.z + threshold_);
        double dis = DBL_MAX;
        for (int x1 = lx; x1 <= ux; ++x1) {
            for (int y1 = ly; y1 <= uy; ++y1) {
                for (int z1 = lz; z1 <= uz; ++z1) {
                    int x2 = x1 + g_x, y2 = y1 + g_y, z2 = z1 + g_z;
                    if (x2 < 0 || x2 >= n1 || y2 < 0 || y2 >= n2 ||
                        z2 < 0 || z2 >= n3) continue;

                    for (const Point& q : grid_(x2, y2, z2)) {
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
        int z_min = GetZIndex(box.z_min());
        int z_max = GetZIndex(box.z_max());

        vertices->clear();
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                for (int z = z_min; z <= z_max; ++y) {
                    for (const Point& p : grid_(x, y, z)) {
                        if (Intersect(box, p)) {
                            vertices->push_back(p);
                        }
                    }
                }
            }
        }
    }

    /**
     * Return true if at least one point inside box.
     */
    bool ContainSnapVertex(const Box3D<T>& box) const {
        int x_min = GetXIndex(box.x_min());
        int x_max = GetXIndex(box.x_max());
        int y_min = GetYIndex(box.y_min());
        int y_max = GetYIndex(box.y_max());
        int z_min = GetZIndex(box.z_min());
        int z_max = GetZIndex(box.z_max());

        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                for (int z = z_min; z <= z_max; ++z) {
                    for (const Point& p : grid_(x, y, z)) {
                        if (Intersect(box, p)) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /**
     * Given a line segment (p, q), find the snapped polyline.
     */
    void FindSnapSegment(const Point& p, const Point& q, Array<Point>* line) {
        CHECK(line);

        Box3D<T> box(std::min(p.x, q.x) - threshold_,
                     std::max(p.x, q.x) + threshold_,
                     std::min(p.y, q.y) - threshold_,
                     std::max(p.y, q.y) + threshold_,
                     std::min(p.z, q.z) - threshold_,
                     std::max(p.z, q.z) + threshold_);
        Array<Point> vertices;
        FindSnapVertices(box, &vertices);

        line->clear();

        Segment3D<T> seg(p, q);
        for (const Point& p : vertices) {
            if (Distance(p, seg) <= threshold_) {
                line->push_back(p);
            }
        }
        PointDotCompare3D<T> compare(p, q);
        std::sort(line->begin(), line->end(), compare);
    }

    /**
     * Get all snap points in the Snap3D.
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

    /**
     * Return the z-axis index of the value 'v'.
     */
    int GetZIndex(T v) const {
        int n = grid_.shape(2);
        return Clamp((v - box_.z_min()) / z_resolution_, 0.0, n - 1.0);
    }

    const Box3D<T>& box() const {
        return box_;
    }

    double threshold() const {
        return threshold_;
    }

private:
    // 3D spatial grid for fast access.
    ArrayND<Array<Point>> grid_;

    // Bounding box of vertices.
    Box3D<T> box_;

    // Threshld for snapping.
    double threshold_;

    // X, Y, and Z resolution.
    double x_resolution_, y_resolution_, z_resolution_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_SNAP_3D_H_
