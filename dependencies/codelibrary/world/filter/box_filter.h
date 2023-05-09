//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_FILTER_BOX_FILTER_H_
#define CODELIBRARY_WORLD_FILTER_BOX_FILTER_H_

#include "codelibrary/world/filter/filter.h"

namespace cl {
namespace world {
namespace filter {

/**
 * Horizental box filter.
 */
template <int KERNEL_RADIUS>
Shader* HorizentalBoxFilter() {
    static_assert(KERNEL_RADIUS > 0, "");

    static std::string fragment_shader = GLSL_SOURCE(
        in vec2 frag_uv;
        out vec4 out_color;

        uniform int kernel_radius;
        uniform sampler2D image;

        void main() {
            vec2 offset_x = vec2(1.0 / textureSize(image, 0).x, 0.0);
            out_color = vec4(0);
            for (int i = -kernel_radius; i <= kernel_radius; ++i) {
                out_color += texture(image, frag_uv + i * offset_x);
            }
            out_color /= (2.0 * kernel_radius + 1.0);
        }
    );

    Shader* shader = Filter(fragment_shader);
    shader->Use();
    shader->SetUniform("kernel_radius", KERNEL_RADIUS);
    return shader;
}

/**
 * Vertical box filter.
 */
template <int KERNEL_RADIUS>
Shader* VerticalBoxFilter() {
    static_assert(KERNEL_RADIUS > 0, "");

    static std::string fragment_shader = GLSL_SOURCE(
        in vec2 frag_uv;
        out vec4 out_color;

        uniform int kernel_radius;
        uniform sampler2D image;

        void main() {
            vec2 offset_y = vec2(0.0, 1.0 / textureSize(image, 0).y);
            out_color = vec4(0);
            for (int i = -kernel_radius; i <= kernel_radius; ++i) {
                out_color += texture(image, frag_uv + i * offset_y);
            }
            out_color /= (2.0 * kernel_radius + 1.0);
        }
    );

    Shader* shader = Filter(fragment_shader);
    shader->Use();
    shader->SetUniform("kernel_radius", KERNEL_RADIUS);
    return shader;
}

} // namespace filter
} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_FILTER_BOX_FILTER_H_
