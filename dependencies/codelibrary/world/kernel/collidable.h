//
// Copyright 2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_
#define CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_

#include "codelibrary/world/kernel/collision_shape.h"

namespace cl {
namespace world {

/**
 * Class to implement collision.
 *
 * It defines the basic collision operations for 'world::Node'.
 */
struct Collidable {
public:
    Collidable() = default;

    Collidable(const Collidable&) = delete;

    Collidable& operator=(const Collidable&) = delete;

    /**
     * Clear the collider.
     */
    void ClearCollider() {
        collision_shapes_.clear();
    }

    /**
     * Add a box collision into collider.
     */
    void AddBoxCollision(const FBox3D& box =
                         FBox3D(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f)) {
        collision_shapes_.emplace_back(new BoxCollision(box));
    }

    /**
     * Add a sphere collision into collider.
     */
    void AddSphereCollision(const FSphere3D& sphere =
                            FSphere3D(FPoint3D(0.0f, 0.0f, 0.0f), 1.0f)) {
        collision_shapes_.emplace_back(new SphereCollision(sphere));
    }

    /**
     * Add a cylinder collision into collider.
     */
    void AddCylinderCollision(const FCylinder3D& cylinder =
                              FCylinder3D(FPoint3D(0.0f, 0.0f, 0.0f),
                                          FPoint3D(0.0f, 0.0f, 1.0f),
                                          1.0f)) {
        collision_shapes_.emplace_back(new CylinderCollision(cylinder));
    }

    /**
     * Add a mesh collision into collider.
     */
    void AddMeshCollision(const Array<FTriangle3D>& mesh) {
        collision_shapes_.emplace_back(new MeshCollision(mesh));
    }

    /**
     * Add a mesh collision into collider.
     */
    void AddMeshCollision(const RenderData& mesh) {
        collision_shapes_.emplace_back(new MeshCollision(mesh));
    }

protected:
    /**
     * Pick this node by ray.
     *
     * It only works when collision shapes is not empty.
     */
    bool Pick(const FRay3D& ray, FPoint3D* point = nullptr) const {
        if (collision_shapes_.empty()) return false;

        double dis = DBL_MAX;
        for (const std::unique_ptr<CollisionShape>& shape : collision_shapes_) {
            FPoint3D p;
            if (shape.get()->Pick(ray, &p)) {
                if (point) {
                    double d = Distance(ray.origin(), p);
                    if (d < dis) {
                        dis = d;
                        *point = p;
                    }
                } else {
                    return true;
                }
            }
        }

        return dis < DBL_MAX;
    }

    // We allow muliple collision shapes.
    Array<std::unique_ptr<CollisionShape>> collision_shapes_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_COLLIDABLE_H_
