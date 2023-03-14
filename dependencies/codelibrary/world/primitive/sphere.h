//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_SPHERE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_SPHERE_H_

// The following code can be replaced by #include <numbers> in C++20
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <cmath>
#endif // _USE_MATH_DEFINES

#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/kernel/render_data.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * Build sphere render data with smooth shading using parametric equation:
 *   x = r * cos(u) * cos(v)
 *   y = r * cos(u) * sin(v)
 *   z = r * sin(u)
 * where u: stack(latitude) angle (-90 <= u <= 90)
 *       v: sector(longitude) angle (0 <= v <= 360)
 */
struct Sphere : public RenderData {
    explicit Sphere(int n_slices = 32)
        : RenderData(GL_TRIANGLES) {
        CHECK(n_slices >= 2 && n_slices <= 128);

        float sector_step = 2.0f * float(M_PI) / n_slices;
        float stack_step = float(M_PI) / n_slices;

        for (int i = 0; i <= n_slices; ++i) {
            // Starting from pi/2 to -pi/2
            float stack_angle = float(M_PI_2) - i * stack_step;
            float xy = cosf(stack_angle);
            float z = sinf(stack_angle);

            // Add (n_sectors + 1) vertices per stack.
            // The first and last vertices have same position and normal, but
            // different tex coords.
            for (int j = 0; j <= n_slices; ++j) {
                // Starting from 0 to 2pi
                float sector_angle = j * sector_step;

                // Vertex position.
                vertices.emplace_back(xy * std::cos(sector_angle),
                                      xy * std::sin(sector_angle),
                                      z);

                // Normalized vertex normal.
                normals.push_back(vertices.back().ToVector());

                // Vertex tex coord between [0, 1].
                texture_coords.emplace_back(static_cast<float>(j) / n_slices,
                                            static_cast<float>(i) / n_slices);
            }
        }

        // Indices:
        //
        //  k1--k1+1
        //  |  / |
        //  | /  |
        //  k2--k2+1
        for (int i = 0; i < n_slices; ++i) {
            int k1 = i * (n_slices + 1);
            int k2 = k1 + n_slices + 1;

            for (int j = 0; j < n_slices; ++j, ++k1, ++k2) {
                // 2 triangles per sector excluding 1st and last stacks.
                if (i != 0) {
                    indices.push_back(k1);
                    indices.push_back(k2);
                    indices.push_back(k1 + 1);
                }

                if (i != (n_slices - 1)) {
                    indices.push_back(k1 + 1);
                    indices.push_back(k2);
                    indices.push_back(k2 + 1);
                }
            }
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_SPHERE_H_
