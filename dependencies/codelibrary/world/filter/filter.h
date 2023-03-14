//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_FILTER_UTIL_H_
#define CODELIBRARY_WORLD_FILTER_UTIL_H_

#include "codelibrary/opengl/shader_factory.h"

namespace cl {
namespace world {
namespace filter {

/**
 * Default vertex shader for post processing.
 */
inline const std::string& DefaultVertexShader() {
    static std::string vertex_shader = GLSL_SOURCE(
        layout (location = 0) in vec3 pos;
        layout (location = 3) in vec2 texcoord;

        out vec2 frag_uv;

        void main() {
            frag_uv = texcoord;
            gl_Position = vec4(pos, 1);
        }
    );

    return vertex_shader;
}

// Use Shader rather than gl::Shader.
using gl::Shader;

/**
 * Combine the given fragment shader with the default vertex shader for post
 * processing.
 */
inline Shader* Filter(const std::string& fragment_shader) {
    return GL_CREATE_SHADER(DefaultVertexShader(), fragment_shader);
}

} // namespace filter
} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_FILTER_UTIL_H_
