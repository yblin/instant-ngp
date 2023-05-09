//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_TEXTURE_RENDERER_H_
#define CODELIBRARY_WORLD_RENDERER_TEXTURE_RENDERER_H_

#include "codelibrary/opengl/depth_texture.h"
#include "codelibrary/opengl/framebuffer.h"
#include "codelibrary/opengl/shader_factory.h"
#include "codelibrary/opengl/texture.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * Used to render textures on the screen.
 */
class TextureRenderer {
    TextureRenderer() {
        Initialize();
        screen_quad_.SetRenderData(Quad());
    }

public:
    static TextureRenderer* GetInstance() {
        static TextureRenderer instance;
        return &instance;
    }

    /**
     * Render depth texture on the screen.
     */
    void RenderDepthTexture(GLuint texture_id) {
        CHECK(glIsTexture(texture_id));

        glDisable(GL_DEPTH_TEST);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        depth_texture_render_->Use();
        depth_texture_render_->SetUniform("tex", 0);
        screen_quad_.Render();

        glEnable(GL_DEPTH_TEST);
    }

    /**
     * Render depth texture on the screen.
     */
    void RenderDepthTexture(const gl::DepthTexture& texture) {
        RenderDepthTexture(texture.id());
    }

    /**
     * Render color texture on the screen.
     */
    void RenderColorTexture(GLuint texture_id) {
        CHECK(glIsTexture(texture_id));

        glDisable(GL_DEPTH_TEST);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        color_texture_render_->Use();
        color_texture_render_->SetUniform("tex", 0);
        screen_quad_.Render();

        glEnable(GL_DEPTH_TEST);
    }

    /**
     * Render color texture on the screen.
     */
    void RenderColorTexture(const gl::Texture& texture) {
        RenderColorTexture(texture.id());
    }

private:
    void Initialize() {
        static std::string vertex_shader = GLSL_SOURCE(
            layout (location = 0) in vec3 pos;
            layout (location = 3) in vec2 texcoord;

            out vec2 frag_uv;

            void main() {
                frag_uv = texcoord;
                gl_Position = vec4(pos, 1);
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            in vec2 frag_uv;
            out vec4 out_color;

            uniform sampler2D tex;

            void main() {
                out_color = vec4(vec3(texture(tex, frag_uv).r), 1.0);
            }
        );

        depth_texture_render_ = GL_CREATE_SHADER(vertex_shader,
                                                 fragment_shader);

        static std::string fragment_shader1 = GLSL_SOURCE(
            in vec2 frag_uv;
            out vec4 out_color;

            uniform sampler2D tex;

            void main() {
                out_color = texture(tex, frag_uv);
            }
        );
        color_texture_render_ = GL_CREATE_SHADER(vertex_shader,
                                                 fragment_shader1);
    }

    gl::Shader* depth_texture_render_ = nullptr;
    gl::Shader* color_texture_render_ = nullptr;
    RenderObject screen_quad_;
};

} // namespcae world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_TEXTURE_RENDERER_H_
