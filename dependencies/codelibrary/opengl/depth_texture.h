//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_DEPTH_TEXTURE_H_
#define CODELIBRARY_OPENGL_DEPTH_TEXTURE_H_

#include "codelibrary/opengl/glad.h"

namespace cl {
namespace gl {

/**
 * Depth texture used for framebuffer.
 */
class DepthTexture {
public:
    /**
     * Create a default empty depth texture.
     */
    DepthTexture() = default;

    DepthTexture(int width, int height) {
        Reset(width, height);
    }

    virtual ~DepthTexture() {
        clear();
    }

    /**
     * Reset the depth texture with the given size.
     */
    void Reset(int width, int height) {
        CHECK(height >= 0);
        CHECK(width >= 0);

        if (width_ == width && height_ == height) return;

        width_ = width;
        height_ = height;
        Initialize();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0,
                     GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    }

    /**
     * Bind this texture to the i-th OpenGL texture image unit.
     */
    void Bind(int unit) const {
        CHECK(unit >= 0);
        CHECK(unit < GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS);
        CHECK(id_ != 0);

        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, id_);
    }

    void clear() {
        if (id_) {
            glDeleteTextures(1, &id_);
            id_ = 0;
        }
    }

    bool empty() {
        return id_ == 0;
    }

    GLuint id() const {
        return id_;
    }

private:
    /**
     * Initialize a new texture and set the parameters.
     */
    void Initialize() {
        clear();

        glGenTextures(1, &id_);
        glBindTexture(GL_TEXTURE_2D, id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        float border_color[] = { 1.0, 1.0, 1.0, 1.0 };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border_color);
    }

    int height_ = 0, width_ = 0;
    GLuint id_ = 0;
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_DEPTH_TEXTURE_H_
