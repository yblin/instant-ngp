//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_QUAD_H_
#define CODELIBRARY_WORLD_PRIMITIVE_QUAD_H_

#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Quad is a 2D polygon with four 3D planar vertices.
 */
struct Quad : public RenderData {
    /**
     * Create a unit quad.
     */
    Quad()
        : RenderData(GL_TRIANGLES) {
        Initialize(FPoint3D(-1.0f, 1.0f, 0.0f), FPoint3D(-1.0f, -1.0f, 0.0f),
                   FPoint3D(1.0f, -1.0f, 0.0f), FPoint3D(1.0f, 1.0f, 0.0f));
        texture_coords = { FPoint2D(0.0f, 1.0f), FPoint2D(0.0f, 0.0f),
                           FPoint2D(1.0f, 0.0f), FPoint2D(1.0f, 1.0f) };
    }

    /**
     * Create a quad without texture coordinates.
     */
    Quad(const FPoint3D& p1, const FPoint3D& p2, const FPoint3D& p3,
         const FPoint3D& p4)
        : RenderData(GL_TRIANGLES) {
        Initialize(p1, p2, p3, p4);
    }

    /**
     * Construct a quad with texture coordinates.
     */
    Quad(const FPoint3D& p1, const FPoint3D& p2,
         const FPoint3D& p3, const FPoint3D& p4,
         const FPoint2D& t1, const FPoint2D& t2,
         const FPoint2D& t3, const FPoint2D& t4)
        : RenderData(GL_TRIANGLES) {
        Initialize(p1, p2, p3, p4);
        texture_coords = {t1, t2, t3, t4};
    }

private:
    void Initialize(const FPoint3D& p1, const FPoint3D& p2, const FPoint3D& p3,
                    const FPoint3D& p4) {
        normals.assign(4, Normalize(CrossProduct(p2 - p1, p3 - p1)));
        vertices = {p1, p2, p3, p4};
        indices = {0, 1, 2, 0, 2, 3};
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_QUAD_H_
