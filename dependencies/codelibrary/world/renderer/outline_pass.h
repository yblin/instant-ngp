//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_OUTLINE_PASS_H_
#define CODELIBRARY_WORLD_RENDERER_OUTLINE_PASS_H_

#include "codelibrary/world/scene.h"

namespace cl {
namespace world {

/**
 * Outline pass.
 */
class OutlinePass {
    OutlinePass() {
        InitializeShader();
        screen_quad_.SetRenderData(Quad());
    }

public:
    static OutlinePass* GetInstance() {
        static OutlinePass instance;
        return &instance;
    }

    /**
     * Input normal-depth texture and light texture, output blended texture with
     * outlines and light.
     */
    void Pass(const gl::Camera& camera,
              const gl::Texture& normal_depth_texture,
              const gl::Texture& light_texture) {
        int enable_depth_test;
        glGetIntegerv(GL_DEPTH_TEST, &enable_depth_test);
        glDisable(GL_DEPTH_TEST);

        shader_->Use();
        shader_->SetUniform("clip_to_view",
                                    camera.projection().Inverse());
        shader_->SetUniform("depth_threshold", depth_threshold_);
        shader_->SetUniform("depth_normal_threshold",
                                    depth_normal_threshold_);
        shader_->SetUniform("depth_normal_threshold_scale",
                                    depth_normal_threshold_scale_);
        shader_->SetUniform("normal_threshold", normal_threshold_);
        shader_->SetUniform("outline_width", outline_width_);
        shader_->SetUniform("outline_color", outline_color_);

        normal_depth_texture.Bind(0);
        light_texture.Bind(1);
        screen_quad_.Render();

        if (enable_depth_test) glEnable(GL_DEPTH_TEST);
    }

    void set_outline_width(float w) {
        CHECK(w > 0.0f);

        outline_width_ = w;
    }

    void set_outline_color(const RGB32Color& c) {
        outline_color_ = c;
    }

private:
    /**
     * Initialize outline shader.
     */
    void InitializeShader() {
        static std::string vertex_shader = GLSL_SOURCE(
            out vec2 texcoord;
            out vec3 view_space_dir;

            uniform mat4 clip_to_view;

            void main() {
                texcoord       = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
                vec4 pos       = vec4(texcoord * 2.0 - 1.0, 0.0, 1.0);
                view_space_dir = (clip_to_view * pos).xyz;

                gl_Position = pos;
            }
        );

        static std::string fragment_shader = GLSL_SOURCE(
            layout(binding = 0) uniform sampler2D normals_depth_texture;
            layout(binding = 1) uniform sampler2D main_texture;

            in vec2 texcoord;
            in vec3 view_space_dir;

            out vec4 frag_color;

            uniform float depth_threshold;
            uniform float depth_normal_threshold;
            uniform float depth_normal_threshold_scale;
            uniform float normal_threshold;

            uniform float outline_width;
            uniform vec4  outline_color;

            vec4 AlphaBlend(vec4 top, vec4 bottom) {
                vec3  color = (top.rgb * top.a) + (bottom.rgb * (1 - top.a));
                float alpha = top.a + bottom.a * (1 - top.a);

                return vec4(color, alpha);
            }

            void main() {
                float half_width_scale_floor = floor(outline_width);
                float half_width_scale_ceil  = ceil (outline_width);
                vec2 pixel_size = 1.0 / textureSize(normals_depth_texture, 0);
                vec4 normal_depth = texture(normals_depth_texture, texcoord);

                // Texcoords.
                vec2 bottom_left  = texcoord - pixel_size *
                                    half_width_scale_floor;
                vec2 top_right    = texcoord + pixel_size *
                                    half_width_scale_ceil;
                vec2 bottom_right = texcoord + vec2( pixel_size.x *
                                                     half_width_scale_ceil,
                                                     -pixel_size.y *
                                                     half_width_scale_floor);
                vec2 top_left     = texcoord + vec2(-pixel_size.x *
                                                    half_width_scale_floor,
                                                    -pixel_size.y *
                                                    half_width_scale_ceil);

                // Normal edge.
                vec3 normal0 = texture(normals_depth_texture, bottom_left).rgb;
                vec3 normal1 = texture(normals_depth_texture, top_right).rgb;
                vec3 normal2 = texture(normals_depth_texture, bottom_right).rgb;
                vec3 normal3 = texture(normals_depth_texture, top_left).rgb;

                // Depth edge.
                float depth0 = texture(normals_depth_texture, bottom_left).a;
                float depth1 = texture(normals_depth_texture, top_right).a;
                float depth2 = texture(normals_depth_texture, bottom_right).a;
                float depth3 = texture(normals_depth_texture, top_left).a;

                // Return a value from [0, 1] range to [-1, 1] range.
                vec3 view_normal = normal_depth.rgb * 2.0 - 1.0;
                float n_dot_v = 1.0 - dot(view_normal, -view_space_dir);

                float n_threshold = clamp((n_dot_v - depth_normal_threshold) /
                                          (1.0001 - depth_normal_threshold),
                                          0.0, 1.0);
                n_threshold = n_threshold * depth_normal_threshold_scale + 1.0;

                float d_threshold = depth_threshold * normal_depth.a *
                                    n_threshold;

                float depht_finite_diff0 = depth1 - depth0;
                float depht_finite_diff1 = depth3 - depth2;

                // edge_depth is calculated using the Roberts cross operator.
                // The same operation is applied to the normal below.
                float edge_depth = sqrt(pow(depht_finite_diff0, 2) +
                                        pow(depht_finite_diff1, 2)) * 100.0;
                edge_depth = edge_depth > d_threshold ? 1.0 : 0.0;

                vec3 normal_finite_diff0 = normal1 - normal0;
                vec3 normal_finite_diff1 = normal3 - normal2;

                float edge_normal = sqrt(dot(normal_finite_diff0,
                                             normal_finite_diff0) +
                                         dot(normal_finite_diff1,
                                             normal_finite_diff1));
                edge_normal = edge_normal > normal_threshold ? 1.0 : 0.0;

                float edge = max(edge_depth, edge_normal);

                vec4 edge_color = vec4(outline_color.rgb, 1.0 * edge);
                vec4 color = texture(main_texture, texcoord);
                frag_color = AlphaBlend(edge_color, color);
            }
        );

        shader_ = GL_CREATE_SHADER(vertex_shader, fragment_shader);
    }

    // Shader from textures to outline.
    gl::Shader* shader_ = nullptr;

    // Help to render to the framebuffer.
    RenderObject screen_quad_;

    // Parameters for outline pass.
    float depth_threshold_ = 0.8f;
    float depth_normal_threshold_ = 0.5f;
    float depth_normal_threshold_scale_ = 7.0f;
    float normal_threshold_ = 0.4f;
    float outline_width_ = 1.0f;
    RGB32Color outline_color_ = RGB32Color(255, 165, 0);
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_OUTLINE_PASS_H_
