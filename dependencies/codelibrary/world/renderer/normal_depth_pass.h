//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_NORMAL_DEPTH_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_NORMAL_DEPTH_PASS_H_

#include "codelibrary/world/scene.h"

namespace cl {
namespace world {

/**
 * Render normal depth map of the scene.
 */
class NormalDepthPass {
    NormalDepthPass() {
        InitializeShader();
    }

public:
    static NormalDepthPass* GetInstance() {
        static NormalDepthPass instance;
        return &instance;
    }

    /**
     * Input nodes and output normal depth map.
     */
    void Pass(const gl::Camera& camera, const Array<Node*>& nodes) {
        shader_->Use();
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("view", camera.viewing());

        for (Node* node : nodes) {
            shader_->SetUniform("model", node->global_transform());
            shader_->SetUniform("is_instance", node->is_instance());
            node->Render();
        }
    }

private:
    /**
     * Shader to render the normal-depth map of the nodes.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 2) in vec3 norm;
            layout (location = 4) in mat4 instance_matrix;

            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;
            uniform bool is_instance;

            out vec3 view_normal;

            void main() {
                view_normal = normalize(mat3(transpose(inverse(view * model))) *
                                        norm);
                mat4 model_t = is_instance ? model * instance_matrix
                                           : model;
                vec4 world_pos = model_t * vec4(pos, 1.0);
                gl_Position = projection * view * world_pos;
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            in vec3 view_normal;

            out vec4 frag_normal_depth;

            void main() {
                vec3 normal = normalize(view_normal) * 0.5 + 0.5;
                float depth = gl_FragCoord.z;
                frag_normal_depth = vec4(normal, depth);
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    // Shader to render scene into normal-depth texture.
    gl::Shader* shader_ = nullptr;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_NORMAL_DEPTH_PASS_H_
