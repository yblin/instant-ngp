//
// Copyright 2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_CUBE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_CUBE_H_

#include "codelibrary/world/kernel/render_data.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * Create a cube where the center point is (0, 0, 0).
 */
struct Cube : public RenderData {
    Cube() : RenderData(GL_TRIANGLES) {
        Array<Quad> quads = {
            Quad(FPoint3D(-1.0f, -1.0f, -1.0f),
                 FPoint3D(-1.0f, +1.0f, -1.0f),
                 FPoint3D(+1.0f, +1.0f, -1.0f),
                 FPoint3D(+1.0f, -1.0f, -1.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f),
                 FPoint2D(0.0f, 0.0f)),
            Quad(FPoint3D(-1.0f, -1.0f, +1.0f),
                 FPoint3D(+1.0f, -1.0f, +1.0f),
                 FPoint3D(+1.0f, +1.0f, +1.0f),
                 FPoint3D(-1.0f, +1.0f, +1.0f),
                 FPoint2D(0.0f, 0.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f)),
            Quad(FPoint3D(-1.0f, -1.0f, -1.0f),
                 FPoint3D(+1.0f, -1.0f, -1.0f),
                 FPoint3D(+1.0f, -1.0f, +1.0f),
                 FPoint3D(-1.0f, -1.0f, +1.0f),
                 FPoint2D(0.0f, 0.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f)),
            Quad(FPoint3D(-1.0f, +1.0f, -1.0f),
                 FPoint3D(-1.0f, +1.0f, +1.0f),
                 FPoint3D(+1.0f, +1.0f, +1.0f),
                 FPoint3D(+1.0f, +1.0f, -1.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f),
                 FPoint2D(0.0f, 0.0f)),
            Quad(FPoint3D(-1.0f, -1.0f, -1.0f),
                 FPoint3D(-1.0f, -1.0f, +1.0f),
                 FPoint3D(-1.0f, +1.0f, +1.0f),
                 FPoint3D(-1.0f, +1.0f, -1.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f),
                 FPoint2D(0.0f, 0.0f)),
            Quad(FPoint3D(+1.0f, -1.0f, -1.0f),
                 FPoint3D(+1.0f, +1.0f, -1.0f),
                 FPoint3D(+1.0f, +1.0f, +1.0f),
                 FPoint3D(+1.0f, -1.0f, +1.0f),
                 FPoint2D(0.0f, 0.0f),
                 FPoint2D(1.0f, 0.0f),
                 FPoint2D(1.0f, 1.0f),
                 FPoint2D(0.0f, 1.0f))
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
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_CUBE_H_
