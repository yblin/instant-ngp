//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_H_
#define CODELIBRARY_WORLD_RENDERER_H_

#include "codelibrary/base/timer.h"
#include "codelibrary/world/scene.h"
#include "codelibrary/world/renderer/basic_renderer.h"
#include "codelibrary/world/renderer/light_pass.h"
#include "codelibrary/world/renderer/normal_depth_pass.h"
#include "codelibrary/world/renderer/outline_pass.h"
#include "codelibrary/world/renderer/texture_renderer.h"

namespace cl {
namespace world {

/**
 * Forward renderer for 3D world
 */
class Renderer : public BasicRenderer {
    static const int N_INTER_FRAMEBUFFERS = 2;

public:
    /**
     * Setup renderer with window and corresponding UI.
     */
    Renderer(Window* window, gl::Camera* camera)
        : BasicRenderer(window, camera) {
        glGenQueries(1, &time_elapsed_query_);
    }

    virtual ~Renderer() {
        glDeleteQueries(1, &time_elapsed_query_);
    }

    /**
     * Render the scene.
     */
    virtual void Render(Scene* scene) override {
        CHECK(scene);

        glBeginQuery(GL_TIME_ELAPSED, time_elapsed_query_);

        this->SetupGL();
        scene->Update();

        if (scene->nodes().empty()) return;
        if (viewport_.x_length() == 0.0 || viewport_.y_length() == 0.0) return;

        // Important: because we render everything into framebuffer, the
        // viewport should be set to framebuffer's resolution.
        glViewport(0, 0, viewport_.x_length(), viewport_.y_length());

        LightPass(scene);
        OutlinePass(scene);

        // Restore the renderer's viewport, so that texture renderer will
        // render the target to the right place.
        glViewport(viewport_.x_min(), viewport_.y_min(),
                   viewport_.x_length(), viewport_.y_length());

        // Render the finial result.
        TextureRenderer::GetInstance()->RenderColorTexture(out_texture_);
        glEndQuery(GL_TIME_ELAPSED);

        GLuint64 elapsed_time;
        glGetQueryObjectui64v(time_elapsed_query_, GL_QUERY_RESULT,
                              &elapsed_time);
        rendering_time_ = elapsed_time * 1e-9;
    }

    /**
     * Show or hide outlines for checked nodes.
     */
    void set_show_outlines(bool flag) {
        show_outlines_ = flag;
    }

    /**
     * Return the current rendering time.
     */
    double rendering_time() {
        return rendering_time_;
    }

private:
    /**
     * Reset intermediate framebuffer if necessary.
     */
    void ResetInterFramebuffers(int n) {
        CHECK(n >= 1 && n <= N_INTER_FRAMEBUFFERS);

        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);
        int w = v[2], h = v[3];

        for (int i = 0; i < n; ++i) {
            if (w == inter_framebuffers_[i].width() &&
                h == inter_framebuffers_[i].height())
                continue;

            inter_textures_[i].Create(w, h, 4);

            inter_framebuffers_[i].Bind();
            inter_framebuffers_[i].Reset(w, h);
            inter_framebuffers_[i].CreateDepthBuffer(GL_DEPTH_COMPONENT);
            inter_framebuffers_[i].AttachColorTexture(GL_TEXTURE_2D,
                                                      inter_textures_[i].id());
            inter_framebuffers_[i].Unbind();
        }
    }

    /**
     * Do light pass.
     */
    void LightPass(Scene* scene) {
        ms_framebuffer_.Bind();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        auto light_pass = LightPass::GetInstance();
        light_pass->Pass(*camera_, scene);
        ms_framebuffer_.Unbind();
        ms_framebuffer_.TransferColorBuffer(0, &out_framebuffer_);
    }

    /**
     * Do outline pass.
     */
    void OutlinePass(Scene* scene) {
        if (!show_outlines_) return;

        Array<Node*> checked_nodes;
        scene->GetCheckedNodes(&checked_nodes);
        if (checked_nodes.empty()) return;

        // For checked nodes, we render outlines for them.
        ResetInterFramebuffers(2);

        // Get depth-normal map.
        inter_framebuffers_[0].Bind();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        auto normal_depth_pass = NormalDepthPass::GetInstance();
        normal_depth_pass->Pass(*camera_, checked_nodes);
        inter_framebuffers_[0].Unbind();

        // Get outlines.
        inter_framebuffers_[1].Bind();
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        auto outline_pass = OutlinePass::GetInstance();
        outline_pass->Pass(*camera_, inter_textures_[0], out_texture_);
        inter_framebuffers_[1].Unbind();

        inter_framebuffers_[1].TransferColorBuffer(0, &out_framebuffer_);
    }

    // Show or hide outlines for the checked nodes.
    bool show_outlines_ = false;

    // Query the rendering time.
    GLuint time_elapsed_query_ = 0;

    // Rendering time.
    double rendering_time_ = 0.0;

    // Intermediate texture for other passes.
    gl::Texture inter_textures_[N_INTER_FRAMEBUFFERS];

    // Except the ms_framebuffer and out_framebuffer, we also need more
    // intermediate framebuffer for other passes.
    gl::Framebuffer inter_framebuffers_[N_INTER_FRAMEBUFFERS];
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_H_
