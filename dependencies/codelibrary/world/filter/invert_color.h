//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_FILTER_INVERT_COLOR_H_
#define CODELIBRARY_WORLD_FILTER_INVERT_COLOR_H_

#include "codelibrary/world/filter/filter.h"

namespace cl {
namespace world {
namespace filter {

/**
 * Invert the intensity range of the input image.
 */
inline Shader* InvertColor() {
    static std::string frag = GLSL_SOURCE(
        in vec2 frag_uv;
        out vec4 out_color;

        uniform sampler2D image;

        void main() {
            out_color = vec4(vec3(1.0) - texture(image, frag_uv).rgb, 1.0);
        }
    );
    return Filter(frag);
}

} // namespace filter
} // namespace gl
} // namespace cl

#endif // CODELIBRARY_WORLD_FILTER_INVERT_COLOR_H_
