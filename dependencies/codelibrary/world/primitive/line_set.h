//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_LINE_SET_H_
#define CODELIBRARY_WORLD_PRIMITIVE_LINE_SET_H_

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Lines render data.
 */
struct LineSet : public RenderData {
    LineSet() : RenderData(GL_LINES) {}

    LineSet(const FPoint3D& p1, const FPoint3D& p2)
        : RenderData(GL_LINES) {
        vertices = {p1, p2};
        indices = {0, 1};
    }

    LineSet(const FPoint3D& p1, const FPoint3D& p2, const RGB32Color& c)
        : LineSet(p1, p2) {
        colors = {c, c};
    }

    /**
     * Construct a line set of box's contour.
     */
    explicit LineSet(const FBox3D& box)
        : RenderData(GL_LINES) {
        vertices = {
            {box.x_min(), box.y_min(), box.z_min()},
            {box.x_min(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_max(), box.z_min()},
            {box.x_max(), box.y_min(), box.z_min()},

            {box.x_min(), box.y_min(), box.z_max()},
            {box.x_min(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_max(), box.z_max()},
            {box.x_max(), box.y_min(), box.z_max()}
        };
        indices = {0, 1, 1, 2, 2, 3, 3, 0,
                   4, 5, 5, 6, 6, 7, 7, 4,
                   0, 4, 1, 5, 2, 6, 3, 7};
    }
    LineSet(const FBox3D& box, const RGB32Color& c)
        : LineSet(box) {
        colors.assign(8, c);
    }

    /**
     * Lines from surface mesh faces.
     */
    LineSet(const geometry::SurfaceMesh<FPoint3D>& mesh)
        : RenderData(GL_LINES) {
        Array<Array<int>> faces;
        mesh.ToCompressMesh(&vertices, &faces);
        for (auto& f : faces) {
            for (int i = 0; i < f.size(); ++i) {
                int next = i + 1 == f.size() ? 0 : i + 1;
                indices.push_back(f[i]);
                indices.push_back(f[next]);
            }
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_
