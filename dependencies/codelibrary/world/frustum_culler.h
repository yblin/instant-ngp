//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_FRUSTUM_CULLER_H_
#define CODELIBRARY_WORLD_FRUSTUM_CULLER_H_

#include "codelibrary/opengl/camera.h"
#include "codelibrary/world/node.h"

namespace cl {
namespace world {

/**
 * Frustum culling.
 */
class FrustumCuller {
    struct Plane {
        double a, b, c, d;
    };

    /**
     * Camera frustum for frustum culling.
     */
    struct Frustum {
        Plane planes[6];
    };

public:
    /**
     * Get camera's frustum.
     */
    FrustumCuller(const gl::Camera& camera) {
        Array<FPoint3D> vertices;
        camera.GetFrustum(&vertices);
        Array<FPlane3D> frustum_planes(6);
        frustum_planes[0] = FPlane3D(vertices[0], vertices[2], vertices[1]);
        frustum_planes[1] = FPlane3D(vertices[4], vertices[5], vertices[6]);
        frustum_planes[2] = FPlane3D(vertices[1], vertices[2], vertices[5]);
        frustum_planes[3] = FPlane3D(vertices[0], vertices[4], vertices[3]);
        frustum_planes[4] = FPlane3D(vertices[0], vertices[1], vertices[5]);
        frustum_planes[5] = FPlane3D(vertices[3], vertices[6], vertices[2]);
        for (int i = 0; i < 6; ++i) {
            const FPlane3D& plane = frustum_planes[i];
            frustum_.planes[i].a = plane.normal().x;
            frustum_.planes[i].b = plane.normal().y;
            frustum_.planes[i].c = plane.normal().z;
            frustum_.planes[i].d = -frustum_.planes[i].a * plane.point().x -
                                    frustum_.planes[i].b * plane.point().y -
                                    frustum_.planes[i].c * plane.point().z;
        }
    }

    /**
     * Check if the given node can be culled.
     */
    bool Cull(const Node* node) const {
        if (!node->is_cullable()) return false;

        // Compute the vertices of node's bounding box.
        FBox3D box = node->GetBoundingBox();
        Array<FPoint3D> vertices = {
            {box.x_min(), box.y_min(), box.z_min()},
            {box.x_min(), box.y_min(), box.z_max()},
            {box.x_min(), box.y_max(), box.z_min()},
            {box.x_min(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_min(), box.z_min()},
            {box.x_max(), box.y_min(), box.z_max()},
            {box.x_max(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_max(), box.z_max()}
        };

        for (const Plane& p : frustum_.planes) {
            bool is_forward = false;
            for (FPoint3D v : vertices) {
                v = node->global_transform()(v);
                if (p.a * v.x + p.b * v.y + p.c * v.z + p.d > 0) {
                    is_forward = true;
                    break;
                }
            }
            if (!is_forward) return true;
        }
        return false;
    }

    /**
     * Cull a set of nodes.
     */
    void Cull(Array<Node*>* nodes) const {
        int n = 0;
        for (Node* node : *nodes) {
            if (!Cull(node)) {
                (*nodes)[n++] = node;
            }
        }
        nodes->resize(n);
    }

private:
    Frustum frustum_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_FRUSTUM_CULLER_H_
