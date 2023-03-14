//
// Copyright 2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_CYLINDER_H_
#define CODELIBRARY_WORLD_PRIMITIVE_CYLINDER_H_

// The following code can be replaced by #include <numbers> in C++20
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <cmath>
#endif // _USE_MATH_DEFINES

#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Cylinder.
 */
struct Cylinder : public RenderData {
    /**
     * Construct a unit cylinder.
     */
    explicit Cylinder(int n_slices = 32)
        : RenderData(GL_TRIANGLES) {
        CHECK(n_slices >= 3);

        float angle = 2.0f * float(M_PI) / n_slices;
        float r = 0.0f;
        for (int i = 0; i <= n_slices; ++i, r += angle) {
            vertices.emplace_back(std::cos(r), std::sin(r), 0.0f);
            vertices.emplace_back(std::cos(r), std::sin(r), 1.0f);
        }
        normals.resize(vertices.size());
        texture_coords.resize(vertices.size());

        // Add cylinder surface.
        for (int i = 0; i <= 2 * n_slices; i += 2) {
            FVector3D v = vertices[i].ToVector();
            normals[i]     = v;
            normals[i + 1] = v;

            float s = 0.5f * i / n_slices;
            texture_coords[i]     = FPoint2D(s, 1.0f);
            texture_coords[i + 1] = FPoint2D(s, 0.0f);

            if (i != 2 * n_slices) {
                indices.push_back(i);
                indices.push_back(i + 2);
                indices.push_back(i + 1);

                indices.push_back(i + 1);
                indices.push_back(i + 2);
                indices.push_back(i + 3);
            }
        }

        // Add bottom and top surfaces.
        vertices.insert(vertices);
        // Add two center points.
        int bottom_center = vertices.size() - 2;
        int top_center = vertices.size() - 1;
        vertices[bottom_center] = FPoint3D(0.0f, 0.0f, 0.0f);
        vertices[top_center] = FPoint3D(0.0f, 0.0f, 1.0f);

        normals.insert(normals);
        normals[bottom_center] = FVector3D(0.0f, 0.0f, -1.0f);
        normals[top_center] = FVector3D(0.0f, 0.0f,  1.0f);

        texture_coords.insert(texture_coords);
        texture_coords[bottom_center] = FPoint2D(0.5f, 0.5f);
        texture_coords[top_center] = FPoint2D(0.5f, 0.5f);

        int offset = 2 * n_slices + 2;
        for (int i = offset; i < vertices.size() - 2; i += 2) {
            texture_coords[i] = FPoint2D(vertices[i].x * 0.5f + 0.5f,
                                         vertices[i].y * 0.5f + 0.5f);
            texture_coords[i + 1] = FPoint2D(vertices[i].x * 0.5f + 0.5f,
                                             -vertices[i].y * 0.5f + 0.5f);

            normals[i] = FVector3D(0.0f, 0.0f, -1.0f);
            normals[i + 1] = FVector3D(0.0f, 0.0f, 1.0f);

            int next = i + 2 < bottom_center ? i + 2 : offset;
            indices.push_back(bottom_center);
            indices.push_back(next);
            indices.push_back(i);

            indices.push_back(top_center);
            indices.push_back(i + 1);
            indices.push_back(next + 1);
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_CYLINDER_H_
