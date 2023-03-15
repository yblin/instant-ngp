//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_3D_H_
#define CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_3D_H_

#include <unordered_set>

#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/util/tree/octree.h"

namespace cl {
namespace geometry {

/**
 * Sparse voxel octree.
 */
template <typename T, typename Index>
class VoxelOctree : public Octree<bool, Index> {
public:    
    VoxelOctree() = default;

    /**
     * Reset MeshOctree to the given surface mesh, i.e., it voxelizes the given
     * mesh.
     *
     * Paramters:
     *  mesh  - the given surface mesh.
     *  depth - the depth of octree.
     */
    template <typename Point>
    void ResetMesh(const SurfaceMesh<Point>& mesh, int depth) {
        static_assert(std::is_same<typename Point::value_type, T>::value, "");

        CHECK(depth > 0);
        CHECK(depth <= (std::numeric_limits<Index>::digits - 1) / 3);

        box_ = mesh.GetBoundingBox();
        this->Reset(depth);

        x_resolution_ = box_.x_length() / this->resolution_;
        y_resolution_ = box_.y_length() / this->resolution_;
        z_resolution_ = box_.z_length() / this->resolution_;

        std::unordered_set<uint64_t> hash;

        for (auto face : mesh.faces()) {
            Triangle3D<T> triangle = face->GetTriangle();
            Box3D<T> box = triangle.bounding_box();
            int lx = GetXIndex(box.x_min());
            int ux = GetXIndex(box.x_max());
            int ly = GetYIndex(box.y_min());
            int uy = GetYIndex(box.y_max());
            int lz = GetZIndex(box.z_min());
            int uz = GetZIndex(box.z_max());
            for (int i = lx; i <= ux; ++i) {
                for (int j = ly; j <= uy; ++j) {
                    for (int k = lz; k <= uz; ++k) {
                        uint64_t c = (uint64_t(i) << 42) | (uint64_t(j) << 21) |
                                      uint64_t(k);
                        if (hash.find(c) != hash.end()) continue;

                        box = GetVoxel(i, j, k);
                        if (Intersect(box, triangle)) {
                            hash.insert(c);
                            this->Insert(i, j, k, true);
                        }
                    }
                }
            }
        }
    }

    /**
     * Return the x-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetXIndex(T v) const {
        int index = (v - box_.x_min()) / x_resolution_;
        return Clamp(index, 0, this->resolution_ - 1);
    }

    /**
     * Return the y-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetYIndex(T v) const {
        int index = (v - box_.y_min()) / y_resolution_;
        return Clamp(index, 0, this->resolution_ - 1);
    }

    /**
     * Return the z-axis index of the value 'v'.
     *
     * We assume 'v' is inside the box, otherwise it will be clamped.
     */
    int GetZIndex(T v) const {
        int index = (v - box_.z_min()) / z_resolution_;
        return Clamp(index, 0, this->resolution_ - 1);
    }

    /**
     * Get the voxel at index (i, j, k).
     */
    Box3D<T> GetVoxel(int i, int j, int k) const {
        CHECK(i >= 0 && i < this->resolution_);
        CHECK(j >= 0 && j < this->resolution_);
        CHECK(k >= 0 && k < this->resolution_);

        return Box3D<T>(box_.x_min() + i * x_resolution_,
                        box_.x_min() + (i + 1) * x_resolution_,
                        box_.y_min() + j * y_resolution_,
                        box_.y_min() + (j + 1) * y_resolution_,
                        box_.z_min() + k * z_resolution_,
                        box_.z_min() + (k + 1) * z_resolution_);
    }

    /**
     * Get the center of voxel(i, j, k).
     */
    Point3D<T> GetVoxelCenter(int i, int j, int k) const {
        CHECK(i >= 0 && i < this->size1_);
        CHECK(j >= 0 && j < this->size2_);
        CHECK(k >= 0 && k < this->size3_);

        return Point3D<T>(box_.x_min() + (i + 0.5) * x_resolution_,
                          box_.y_min() + (j + 0.5) * y_resolution_,
                          box_.z_min() + (k + 0.5) * z_resolution_);
    }

    const Box3D<T>& box() const {
        return box_;
    }

private:
    // Bounding box of the input mesh.
    Box3D<T> box_;

    // The X, Y, and Z lengh of the voxel.
    T x_resolution_ = 0, y_resolution_ = 0, z_resolution_ = 0;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_VOXEL_OCTREE_3D_H_
