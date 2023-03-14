//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_
#define CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_

#include <cfloat>

#include "codelibrary/geometry/distance_3d.h"
#include "codelibrary/geometry/range_ray_3d.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

class CollisionShape {
public:
    CollisionShape() = default;

    /**
     * Pick the collision shape by ray.
     *
     * It also optionally computes the intersection point. If there are multiple
     * intersection points, only the nearest one will saved.
     *
     * Return true if the ray intersects with the collider.
     */
    virtual bool Pick(const FRay3D& ray, FPoint3D* point) const = 0;
};

class BoxCollision : public CollisionShape {
public:
    explicit BoxCollision(const FBox3D& box)
        : box_(box) {}

    bool Pick(const FRay3D& ray, FPoint3D* point = nullptr) const override {
        FRangeRay3D rray(ray);
        rray.set_lower(-FLT_MAX);
        if (!rray.Intersect(box_) || rray.upper() < 0.0f) return false;

        if (point) {
            if (rray.lower() < 0.0f)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    FBox3D box_;
};

class SphereCollision : public CollisionShape {
public:
    explicit SphereCollision(const FSphere3D& sphere)
        : sphere_(sphere) {}

    bool Pick(const FRay3D& ray, FPoint3D* point = nullptr) const override {
        FRangeRay3D rray(ray);
        rray.set_lower(-FLT_MAX);
        if (!rray.Intersect(sphere_) || rray.upper() < 0.0f) return false;

        if (point) {
            if (rray.lower() < 0.0f)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    FSphere3D sphere_;
};

class CylinderCollision : public CollisionShape {
public:
    explicit CylinderCollision(const FCylinder3D& cylinder)
        : cylinder_(cylinder) {}

    bool Pick(const FRay3D& ray, FPoint3D* point = nullptr) const override {
        FRangeRay3D rray(ray);
        rray.set_lower(-FLT_MAX);
        if (!rray.Intersect(cylinder_) || rray.upper() < 0.0f) return false;

        if (point) {
            if (rray.lower() < 0.0f)
                *point = rray.upper_point();
            else
                *point = rray.lower_point();
        }
        return true;
    }

protected:
    FCylinder3D cylinder_;
};

class MeshCollision : public CollisionShape {
public:
    explicit MeshCollision(const Array<FTriangle3D>& mesh)
        : mesh_(mesh) {}

    explicit MeshCollision(const RenderData& mesh) {
        assert(mesh.type == GL_TRIANGLES);
        assert(mesh.indices.size() % 3 == 0);

        mesh_.reserve(mesh.indices.size() / 3);

        for (int i = 0; i < mesh.indices.size(); i += 3) {
            int a = mesh.indices[i];
            int b = mesh.indices[i + 1];
            int c = mesh.indices[i + 2];
            mesh_.emplace_back(mesh.vertices[a], mesh.vertices[b],
                               mesh.vertices[c]);
        }
    }

    bool Pick(const FRay3D& ray, FPoint3D* point = nullptr) const override {
        FRangeRay3D rray(ray);

        if (!point) {
            for (const FTriangle3D& triangle : mesh_) {
                if (rray.Hit(triangle)) return true;
            }
        } else {
            double distance = DBL_MAX;
            FPoint3D p;
            for (const FTriangle3D& triangle : mesh_) {
                if (rray.Hit(triangle, &p)) {
                    double dis = Distance(p, ray.origin());
                    if (dis < distance) {
                        distance = dis;
                        *point = p;
                    }
                }
            }
            if (distance < DBL_MAX) return true;
        }
        return false;
    }

protected:
    Array<FTriangle3D> mesh_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_COLLISION_SHAPE_H_
