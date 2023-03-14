//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_
#define CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/util/color/rgb32_color.h"
#include "codelibrary/opengl/glad.h"

namespace cl {
namespace world {

/**
 * The data for 3D rendering.
 */
struct RenderData {
    // Type of render data. Should be one of the following value: GL_TRIANGLES,
    // GL_LINES, and GL_POINTS.
    int type;

    // Vertex position.
    Array<FPoint3D> vertices;

    // Color for each vertex.
    Array<RGB32Color> colors;

    // Normal vector for each vertex.
    Array<FVector3D> normals;

    // Texture coordinate for each vertex.
    Array<FPoint2D> texture_coords;

    // Element indices.
    Array<int> indices;

    RenderData(int t = GL_TRIANGLES)
        : type(t) {}

    bool empty() const {
        return vertices.empty();
    }

    void clear() {
        Array<FPoint3D>().swap(vertices);
        Array<RGB32Color>().swap(colors);
        Array<FVector3D>().swap(normals);
        Array<FPoint2D>().swap(texture_coords);
        Array<int>().swap(indices);
    }

    /**
     * Swap data to reduce memory copy.
     */
    void swap(RenderData* data) {
        CHECK(data);

        std::swap(data->type, type);
        vertices.swap(data->vertices);
        colors.swap(data->colors);
        normals.swap(data->normals);
        texture_coords.swap(data->texture_coords);
        indices.swap(data->indices);
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_RENDER_DATA_H_
