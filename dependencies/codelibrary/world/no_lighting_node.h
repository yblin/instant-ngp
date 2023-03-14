//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_NO_LIGHTING_NODE_H_
#define CODELIBRARY_WORLD_NO_LIGHTING_NODE_H_

#include "codelibrary/world/node.h"
#include "codelibrary/opengl/shader_factory.h"

namespace cl {
namespace world {

/**
 * Node that does not need lighting.
 */
class NoLightingNode : public Node {
public:
    NoLightingNode()
        : Node() {
        cast_shadow_ = false;
        receive_shadow_ = false;
    }

    explicit NoLightingNode(const std::string& name)
        : Node(name) {
        cast_shadow_ = false;
        receive_shadow_ = false;
    }

    NoLightingNode(const std::string& name, RenderObject* object)
        : Node(name, object) {
        cast_shadow_ = false;
        receive_shadow_ = false;
    }

    explicit NoLightingNode(RenderObject* object)
        : Node(object) {
        cast_shadow_ = false;
        receive_shadow_ = false;
    }

    /**
     * Do nothing in Render().
     */
    virtual void Render() override {}

    /**
     * Render parts that don't need lighting.
     */
    virtual void RenderWithoutLight(const gl::Camera& camera) override {
        if (!is_visible_) return;

        if (render_list_.empty()) return;

        if (!shader_) InitializeShader();
        CHECK(shader_);
        SetShaderParameters(camera);

        for (RenderObject* o : render_list_) {
            o->Render();
        }
    }

protected:
    virtual void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 1) in vec4 color;
            layout (location = 2) in vec3 normal;

            uniform mat4 view;
            uniform mat4 projection;
            uniform mat4 model;

            out vec4 frag_color;

            void main() {
                frag_color = color;
                gl_Position = projection * view * model * vec4(pos, 1.0);
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            in vec4 frag_color;

            out vec4 out_color;

            // Material parameters.
            uniform bool use_vertex_color;
            uniform vec4 albedo;

            void main() {
                out_color = use_vertex_color ? frag_color : albedo;
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    virtual void SetShaderParameters(const gl::Camera& camera) {
        shader_->Use();
        shader_->SetUniform("view", camera.viewing());
        shader_->SetUniform("projection", camera.projection());
        shader_->SetUniform("model", this->global_transform());

        if (material_) {
            shader_->SetUniform("use_vertex_color", false);
            shader_->SetUniform("albedo", material_->albedo);
        } else {
            shader_->SetUniform("use_vertex_color", true);
        }
    }

    // No lighting node use its own shader to render.
    gl::Shader* shader_ = nullptr;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_NO_LIGHTING_NODE_H_
