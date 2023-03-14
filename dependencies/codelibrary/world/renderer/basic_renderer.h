//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_RENDERER_BASE_RENDERER_H_
#define CODELIBRARY_WORLD_RENDERER_BASE_RENDERER_H_

#include "codelibrary/opengl/camera.h"
#include "codelibrary/ui/window.h"
#include "codelibrary/world/scene.h"

namespace cl {
namespace world {

/**
 * Basic renderer for 3D world.
 */
class BasicRenderer {
public:
    /**
     * Setup renderer with window and viewing camera.
     */
    BasicRenderer(Window* window, gl::Camera* camera)
        : window_(window), camera_(camera) {
        CHECK(window_);
        CHECK(camera_);
    }

    virtual ~BasicRenderer() = default;

    /**
     * Render the scene.
     */
    virtual void Render(Scene* scene) = 0;

    /**
     * Set viewport.
     */
    void set_viewport(const FBox2D& box) {
        CHECK(!box.empty());

        viewport_ = box;
    }

    /**
     * Set background/clear color.
     */
    void set_clear_color(const RGB32Color& c) {
        clear_color_ = c;
    }

    /**
     * Return the parent window.
     */
    Window* window() const {
        return window_;
    }

    /**
     * Return the camera of this renderer.
     */
    gl::Camera* camera() const {
        return camera_;
    }

    /**
     * Return the output texture.
     */
    const gl::Texture& out_texture() const {
        return out_texture_;
    }

protected:
    /**
     * Setup GL. It will be called in every frame.
     */
    void SetupGL() {
        // Enable depth test.
        glEnable(GL_DEPTH_TEST);
        // Change depth function so depth test passes when values are equal to
        // depth buffer's content.
        glDepthFunc(GL_LEQUAL);
        // Enable seamless cubemap sampling for lower mip levels in the
        // pre-filter map.
        glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

        this->SetViewport();
        this->ClearColorAndDepthBuffer();
        this->ResetFramebuffers();
    }

    /**
     * Set viewport.
     */
    void SetViewport() {
        if (viewport_.empty()) {
            // If no viewport, the default viewport is the full window.
            viewport_ = FBox2D(0, window_->width(), 0, window_->height());
        }

        glViewport(viewport_.x_min(), viewport_.y_min(),
                   viewport_.x_length(), viewport_.y_length());
        camera_->SetViewport(viewport_);
    }

    /**
     * Clear color and depth buffer.
     */
    void ClearColorAndDepthBuffer() {
        // We enable scissor here to support multiple renderers. Only the data
        // in viewport will be clear.
        glEnable(GL_SCISSOR_TEST);
        glScissor(viewport_.x_min(), viewport_.y_min(),
                  viewport_.x_length(), viewport_.y_length());

        glClearColor(clear_color_.red()   / 255.0f,
                     clear_color_.green() / 255.0f,
                     clear_color_.blue()  / 255.0f,
                     clear_color_.alpha() / 255.0f);
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        // Remeber to distable it.
        glDisable(GL_SCISSOR_TEST);
    }

    /**
     * Reset framebuffers if necessary.
     */
    void ResetFramebuffers() {
        GLint v[4];
        glGetIntegerv(GL_VIEWPORT, v);
        int w = v[2], h = v[3];
        if (w == ms_framebuffer_.width() && h == ms_framebuffer_.height())
            return;

        int n_samples = window_->n_samples();

        ms_framebuffer_.Bind();
        ms_framebuffer_.Reset(w, h);
        ms_framebuffer_.CreateDepthBuffer(GL_DEPTH_COMPONENT, n_samples);
        ms_framebuffer_.CreateColorBuffer(GL_RGBA, n_samples);
        ms_framebuffer_.Unbind();

        out_framebuffer_.Reset(w, h);
        out_texture_.Create(w, h, 4);

        out_framebuffer_.Bind();
        out_framebuffer_.AttachColorTexture(GL_TEXTURE_2D, out_texture_.id());
        out_framebuffer_.Unbind();
    }

    // Parent window of this renderer.
    Window* window_ = nullptr;

    // Set background/clear color.
    RGB32Color clear_color_ = RGB32Color::White();

    // Viewport of this renderer.
    FBox2D viewport_;

    // Viewing camera.
    gl::Camera* camera_;

    // The output texture.
    gl::Texture out_texture_;

    // Multi-sampled framebuffer to store the result.
    gl::Framebuffer ms_framebuffer_;

    // Out framebuffer.
    gl::Framebuffer out_framebuffer_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_RENDERER_BASE_RENDERER_H_
