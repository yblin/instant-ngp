//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_GRID_2D_H_
#define CODELIBRARY_POINT_CLOUD_GRID_2D_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {
namespace point_cloud {

/**
 * Organize point cloud in 2D grids.
 */
template <typename Point>
class Grid2D {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value, "");

public:
    Grid2D() = default;

    /**
     * Construct a 2D grid with number of grids in X and Y direction and the
     * bounding box.
     */
    explicit Grid2D(const Box2D<T>& box, int n1, int n2)
        : box_(box) {
        CHECK(n1 > 0);
        CHECK(n2 > 0);
        CHECK(n1 <= INT_MAX / n2);

        x_resolution_ = box_.x_length() / n1;
        y_resolution_ = box_.y_length() / n2;
        grid_.reshape(n1, n2);
    }

    explicit Grid2D(const Array<Point>& points) {
        Reset(points);
    }

    /**
     * Reset point clouds in the grids.
     */
    void Reset(const Array<Point>& points) {
        CHECK(!points.empty());

        points_ = points;
        box_ = Box2D<T>(points.begin(), points.end());
        double sqrt_n = 0.5 * std::ceil(std::sqrt(points.size()));
        int n1 = sqrt_n, n2 = sqrt_n;

        x_resolution_ = box_.x_length() / n1;
        y_resolution_ = box_.y_length() / n2;
        grid_.clear();
        grid_.reshape(n1, n2);

        int id = 0;
        for (const Point& p : points) {
            grid_(GetXIndex(p.x), GetYIndex(p.y)).push_back(id++);
        }
    }

    /**
     * Insert a new point into grid.
     *
     * Return the index of this point.
     */
    int Insert(const Point& p) {
        CHECK(!grid_.empty());

        int id = points_.size();
        grid_(GetXIndex(p.x), GetYIndex(p.y)).push_back(id);
        points_.push_back(p);
        return id;
    }

    /**
     * Find all points in the 2D bounding box.
     */
    void Find(const Box2D<T>& box, Array<int>* indices) const {
        CHECK(indices);

        int x_min = GetXIndex(box.x_min());
        int x_max = GetXIndex(box.x_max());
        int y_min = GetYIndex(box.y_min());
        int y_max = GetYIndex(box.y_max());

        indices->clear();
        for (int x = x_min; x <= x_max; ++x) {
            for (int y = y_min; y <= y_max; ++y) {
                for (int id : grid_(x, y)) {
                    if (geometry::Intersect(box, points_[id])) {
                        indices->push_back(id);
                    }
                }
            }
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
     * Return the i-th point.
     */
    const Point& point(int index) const {
        CHECK(index >= 0 && index < points_.size());
        return points_[index];
    }

    bool empty() const {
        return points_.empty();
    }

private:
    // 2D spatial grid for fast access to point cloud.
    // We store the indices here.
    ArrayND<Array<int>> grid_;

    // Length of grids in X and Y direction.
    double x_resolution_, y_resolution_;

    // Bounding box of point clouds.
    Box2D<T> box_;

    // The input point cloud.
    Array<Point> points_;
};

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_GRID_2D_H_
