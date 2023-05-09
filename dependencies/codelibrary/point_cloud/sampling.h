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
#include "codelibrary/geometry/util/voxel_octree.h"

namespace cl {
namespace point_cloud {

/**
 * It first constructs an octree at the given depth and then samples the input
 * point cloud such that each voxel of the only contains only one point.
 */
template <typename Point>
void OctreeSample(const Array<Point>& points, int depth,
                  Array<int>* samples) {
    CHECK(depth > 0);
    CHECK(depth <= 21) << "The depth of octree is too large.";
    CHECK(samples);

    samples->clear();

    using T = typename Point::value_type;
    geometry::VoxelOctree<T, uint64_t> octree;
    Box3D<T> box(points.begin(), points.end());
    octree.Reset(box, depth);

    for (int i = 0; i < points.size(); ++i) {
        const Point& p = points[i];

        if (octree.InsertVoxel(p).second) {
            samples->push_back(i);
        }
    }
}
template <typename Point>
void OctreeSample(const Array<Point>& points, int depth,
                  Array<Point>* samples) {
    CHECK(samples);

    Array<int> indices;
    OctreeSample(points, depth, &indices);
    samples->resize(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        (*samples)[i] = points[indices[i]];
    }
}

/**
 * Similar to the previous one, but estimate the depth of octree according to
 * the given resolution.
 */
template <typename Point>
void OctreeSample(const Array<Point>& points, double resolution,
                  Array<int>* samples) {
    CHECK(resolution > 0.0);

    using T = typename Point::value_type;
    geometry::VoxelOctree<T, uint64_t> octree;
    Box3D<T> box(points.begin(), points.end());
    int size1 = static_cast<int>(box.x_length() / resolution) + 1;
    int size2 = static_cast<int>(box.y_length() / resolution) + 1;
    int size3 = static_cast<int>(box.z_length() / resolution) + 1;

    int size = std::max(std::max(size1, size2), size3);
    int depth = bits::Log2Ceil(size) + 1;
    CHECK(depth <= 21) << "The resolution is too small.";
    octree.ResetBox(box, depth);

    for (int i = 0; i < points.size(); ++i) {
        const Point& p = points[i];

        if (octree.InsertVoxel(p).second) {
            samples->push_back(i);
        }
    }
}
template <typename Point>
void OctreeSample(const Array<Point>& points, double resolution,
                  Array<Point>* samples) {
    CHECK(samples);

    Array<int> indices;
    OctreeSample(points, resolution, &indices);
    samples->resize(indices.size());
    for (int i = 0; i < indices.size(); ++i) {
        (*samples)[i] = points[indices[i]];
    }
}

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_GRID_2D_H_
