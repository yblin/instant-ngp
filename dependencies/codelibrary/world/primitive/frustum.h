//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_FRUSTUM_H_
#define CODELIBRARY_WORLD_PRIMITIVE_FRUSTUM_H_

#include "codelibrary/world/kernel/render_data.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * frustum is the portion of a solid pyramid that lies between two parallel
 * planes cutting it.
 */
struct Frustum : public RenderData {
    Frustum() = default;

    /**
     * Create a frustum without texture coordinates.
     */
    Frustum(const Array<FPoint3D>& frustum_vertices)
        : RenderData(GL_TRIANGLES), vertices_(frustum_vertices) {
        Array<Quad> quads = {
            Quad(frustum_vertices[3], frustum_vertices[2],
                 frustum_vertices[1], frustum_vertices[0],
                 FPoint2D(0.0f, 1.0f), FPoint2D(1.0f, 1.0f),
                 FPoint2D(1.0f, 0.0f), FPoint2D(0.0f, 0.0f)),
            Quad(frustum_vertices[4], frustum_vertices[5],
                 frustum_vertices[6], frustum_vertices[7],
                 FPoint2D(0.0f, 0.0f), FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f), FPoint2D(0.0f, 1.0f)),
            Quad(frustum_vertices[0], frustum_vertices[1],
                 frustum_vertices[5], frustum_vertices[4],
                 FPoint2D(0.0f, 0.0f), FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f), FPoint2D(0.0f, 1.0f)),
            Quad(frustum_vertices[2], frustum_vertices[3],
                 frustum_vertices[7], frustum_vertices[6],
                 FPoint2D(0.0f, 0.0f), FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f), FPoint2D(0.0f, 1.0f)),
            Quad(frustum_vertices[0], frustum_vertices[4],
                 frustum_vertices[7], frustum_vertices[3],
                 FPoint2D(1.0f, 0.0f), FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f), FPoint2D(0.0f, 0.0f)),
            Quad(frustum_vertices[1], frustum_vertices[2],
                 frustum_vertices[6], frustum_vertices[5],
                 FPoint2D(0.0f, 0.0f), FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f), FPoint2D(0.0f, 1.0f)),
        };

        int offset = 0;
        for (int i = 0; i < quads.size(); ++i) {
            vertices.insert(quads[i].vertices);
            normals.insert(quads[i].normals);
            texture_coords.insert(quads[i].texture_coords);
            for (int j = 0; j < quads[i].indices.size(); ++j) {
                indices.push_back(quads[i].indices[j] + offset);
            }
            offset += 4;
        }
    }

private:
    Array<FPoint3D> vertices_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_FRUSTUM_H_
