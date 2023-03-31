//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_SAMPLING_H_
#define CODELIBRARY_POINT_CLOUD_SAMPLING_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/util/tree/octree.h"

namespace cl {
namespace point_cloud {

/**
 * Grid sampling considers a regular grid covering the bounding box of the input
 * point cloud and clusters all points sharing the same cell of the grid by
 * picking as represent one arbitrarily chosen point.
 */
template <typename Point>
void GridSample3D(const Array<Point>& points, double resolution,
                  Array<int>* samples) {
    CHECK(samples);

    samples->clear();

    using T = typename Point::value_type;
    Box3D<T> box(points.begin(), points.end());
    CHECK(box.x_length() / resolution < INT_MAX);
    CHECK(box.y_length() / resolution < INT_MAX);
    CHECK(box.z_length() / resolution < INT_MAX);

    int size1 = static_cast<int>(box.x_length() / resolution) + 1;
    int size2 = static_cast<int>(box.y_length() / resolution) + 1;
    int size3 = static_cast<int>(box.z_length() / resolution) + 1;

    int size = std::max(std::max(size1, size2), size3);
    int depth = bits::Log2Ceil(size) + 1;
    CHECK(depth <= 21) << "The resolution is too small";

    Octree<bool, uint64_t> octree(depth);
    for (int i = 0; i < points.size(); ++i) {
        const Point& p = points[i];
        int x = static_cast<int>((p.x - box.x_min()) / resolution);
        int y = static_cast<int>((p.y - box.y_min()) / resolution);
        int z = static_cast<int>((p.z - box.z_min()) / resolution);
        x = Clamp(x, 0, size1 - 1);
        y = Clamp(y, 0, size2 - 1);
        z = Clamp(z, 0, size3 - 1);

        auto pair = octree.Insert(x, y, z, true);
        if (pair.second) {
            samples->push_back(i);
        }
    }
}
/**
 * Similar to the previous version but output the selected points.
 */
template <typename Point>
void GridSample3D(const Array<Point>& points, double resolution,
                  Array<Point>* sample_points) {
    sample_points->resize(points.size());
    Array<int> samples;
    GridSample3D(points, resolution, &samples);
    for (int i = 0; i < samples.size(); ++i) {
        (*sample_points)[i] = points[samples[i]];
    }
    sample_points->resize(samples.size());
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_GRID_2D_H_
