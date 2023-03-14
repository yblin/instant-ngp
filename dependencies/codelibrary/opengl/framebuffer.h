//
// Copyright 2021-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_FRAMEBUFFER_H_
#define CODELIBRARY_OPENGL_FRAMEBUFFER_H_

#include <cstring>
#include <unordered_set>

#include "codelibrary/base/array.h"
#include "codelibrary/base/log.h"
#include "codelibrary/opengl/glad.h"

namespace cl {
namespace gl {

/**
 * Framebuffer encapsulates an OpenGL framebuffer object for 3D rendering.
 */
class Framebuffer {
    static const int MAX_COLOR_ATTCHMENTS = 32;

public:
    Framebuffer() {
        glGenFramebuffers(1, &id_);
        memset(color_textures_, 0, sizeof(color_textures_));
        memset(color_buffers_, 0, sizeof(color_buffers_));
    }

    /**
     * Create a framebuffer object (FBO) with given width and height.
     */
    Framebuffer(int w, int h)
        : height_(h), width_(w) {
        CHECK(h >= 0 && w >= 0);

        glGenFramebuffers(1, &id_);
        memset(color_textures_, 0, sizeof(color_textures_));
        memset(color_buffers_, 0, sizeof(color_buffers_));
    }

    Framebuffer(const Framebuffer&) = delete;

    virtual ~Framebuffer() {
        clear();
        glDeleteFramebuffers(1, &id_);
        this->Unbind();
    }

    Framebuffer& operator=(const Framebuffer&) = delete;

    /**
     * Clear the attached buffers.
     */
    void clear() {
        if (depth_buffer_) {
            glDeleteRenderbuffers(1, &depth_buffer_);
            depth_buffer_ = 0;
        }
        for (int id : color_attachments_) {
            glDeleteRenderbuffers(1, &color_buffers_[id]);
            color_buffers_[id] = 0;
        }

        attached_depth_texture_ = 0;
        memset(color_textures_, 0, sizeof(color_textures_));
        color_attachments_.clear();
    }

    bool empty() const {
        return height_ == 0 || width_ == 0;
    }

    /**
     * Reset the framebuffer.
     */
    void Reset(int w, int h) {
        CHECK(w >= 0 && h >= 0);

        clear();

        width_ = w;
        height_ = h;
    }

    /**
     * Bind this framebuffer for rendering. Remember to Unbind() after Bind().
     */
    void Bind() const {
        GLuint id = GetCurrentFramebuffer();
        if (id_ != id) old_draw_framebuffer_ = id;
        glBindFramebuffer(GL_FRAMEBUFFER, id_);
    }

    /**
     * Unbind the framebuffer.
     */
    void Unbind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, old_draw_framebuffer_);
    }

    /**
     * Check if the current framebuffer status is complete.
     */
    bool Check() const {
        CHECK(GetCurrentFramebuffer() == id_)
                << "The framebuffer is not binded, call Bind() first.";

        GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

        switch (status) {
        case GL_NO_ERROR:
        case GL_FRAMEBUFFER_COMPLETE:
            return true;
        case GL_FRAMEBUFFER_UNSUPPORTED:
            LOG(INFO) << "Unsupported framebuffer format.";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
            LOG(INFO) << "Framebuffer incomplete attachment.";
            break;
        case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
            LOG(INFO) << "Framebuffer incomplete, missing attachment.";
            break;
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT
        case GL_FRAMEBUFFER_INCOMPLETE_DUPLICATE_ATTACHMENT:
            LOG(INFO) << "Framebuffer incomplete, duplicate attachment.";
            break;
    #endif
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS
        case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
            LOG(INFO) << "Framebuffer incomplete, attached images must have "
                         "same dimensions.";
            break;
    #endif
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_FORMATS
        case GL_FRAMEBUFFER_INCOMPLETE_FORMATS:
            LOG(INFO) << "Framebuffer incomplete, attached images must have "
                         "same format.";
            break;
    #endif
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER
        case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
            LOG(INFO) << "Framebuffer incomplete, missing draw buffer.";
            break;
    #endif
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER
        case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
            LOG(INFO) << "Framebuffer incomplete, missing read buffer.";
            break;
    #endif
    #ifdef GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE
        case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
            LOG(INFO) << "Framebuffer incomplete, attachments must have same "
                         "number of samples per pixel.";
            break;
    #endif
        default:
            LOG(INFO) << "An undefined error has occurred: " << status;
            break;
        }

        return false;
    }

    /**
     * Create a color render buffer and attach it to the framebuffer at
     * GL_COLOR_ATTACHMENTID.
     *
     * Parameter:
     *  format    - Specifies the internal format to use for the renderbuffer
     *              object's image.
     *  n_samples - Specifies the number of samples to be used for the
     *              renderbuffer object's storage.
     */
    template <int ID>
    bool CreateColorBuffer(GLenum format = GL_RGBA, int n_samples = 1) {
        static_assert(ID >= 0 && ID < MAX_COLOR_ATTCHMENTS, "");

        CHECK(GetCurrentFramebuffer() == id_)
                << "This framebuffer is not binded, call Bind() first.";

        if (!color_buffers_[ID]) {
            glGenRenderbuffers(1, &color_buffers_[ID]);
        }

        glBindRenderbuffer(GL_RENDERBUFFER, color_buffers_[ID]);
        if (n_samples > 1) {
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, n_samples, format,
                                             width_, height_);
        } else {
            glRenderbufferStorage(GL_RENDERBUFFER, format, width_, height_);
        }

        // Attach color texture into the render buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + ID,
                                  GL_RENDERBUFFER, color_buffers_[ID]);

        if (!Check()) {
            color_attachments_.erase(ID);
            EraseColorBuffer(ID);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);
            return false;
        }

        color_attachments_.insert(ID);        
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        return true;
    }
    bool CreateColorBuffer(GLenum format = GL_RGB, int n_samples = 1) {
        return CreateColorBuffer<0>(format, n_samples);
    }

    /**
     * Attach an existing 2D color texture to framebuffer.
     * Note that framebuffer does not own this texture.
     *
     * Paramters:
     *  target       - the target texture, e.g., GL_TEXTURE_2D,
     *                 GL_TEXTURE_CUBE_MAP_POSITIVE_X ...
     *  texture_id   - the texture object whose image is to be attached.
     *  mipmap_level - the mipmap level of the texture image to be attached.
     */
    template <int ID>
    bool AttachColorTexture(GLenum target, GLuint texture_id,
                            int mipmap_level = 0) {
        static_assert(ID >= 0 && ID < MAX_COLOR_ATTCHMENTS, "");

        CHECK(GetCurrentFramebuffer() == id_)
                << "The framebuffer is not binded, call Bind() first.";

        // We should clear the previous color buffer.
        EraseColorBuffer(ID);

        // Attach color texture to FBO.
        glFramebufferTexture2D(GL_FRAMEBUFFER,
                               GL_COLOR_ATTACHMENT0 + ID,
                               target,
                               texture_id,
                               mipmap_level);
        if (!Check()) return false;

        color_textures_[ID] = texture_id;
        color_attachments_.insert(ID);
        return true;
    }
    bool AttachColorTexture(GLenum target, GLuint texture_id,
                            int mipmap_level = 0) {
        return AttachColorTexture<0>(target, texture_id, mipmap_level);
    }

    /**
     * Create depth render buffer.
     *
     * A framebuffer only have one depth render buffer.
     */
    bool CreateDepthBuffer(GLenum format = GL_DEPTH_COMPONENT,
                           int n_samples = 1) {
        CHECK(GetCurrentFramebuffer() == id_)
                << "The framebuffer is not binded, call Bind() first.";

        GLenum attachment;
        if ((format == GL_DEPTH24_STENCIL8) ||
            (format == GL_DEPTH32F_STENCIL8)) {
            attachment = GL_DEPTH_STENCIL_ATTACHMENT;
        } else {
            attachment = GL_DEPTH_ATTACHMENT;
        }

        if (!depth_buffer_) {
            glGenRenderbuffers(1, &depth_buffer_);
        }

        glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
        if (n_samples > 1) {
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, n_samples, format,
                                             width_, height_);
        } else {
            glRenderbufferStorage(GL_RENDERBUFFER, format, width_, height_);
        }

        // Attach depth texture into the render buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, attachment, GL_RENDERBUFFER,
                                  depth_buffer_);

        if (!Check()) {
            glDeleteRenderbuffers(1, &depth_buffer_);
            depth_buffer_ = 0;
            glBindRenderbuffer(GL_RENDERBUFFER, 0);
            return false;
        }

        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        return true;
    }

    /**
     * Attach an existing 2D depth texture to framebuffer, so that framebuffer
     * will fill it.
     *
     * Note that framebuffer does not own this texture.
     */
    bool AttachDepthTexture(GLuint texture_id) {
        CHECK(GetCurrentFramebuffer() == id_)
                << "The framebuffer is not binded, call Bind() first.";

        // Attach depth texture to FBO.
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, texture_id,
                               0);
        if (!Check()) return false;

        attached_depth_texture_ = texture_id;
        return true;
    }

    /**
     * Attach an existing 3D depth texture to framebuffer, so that framebuffer
     * will fill it.
     *
     * Note that framebuffer does not own this texture.
     */
    bool AttachDepthTexture3D(GLuint texture_id) {
        CHECK(GetCurrentFramebuffer() == id_)
                << "The framebuffer is not binded, call Bind() first.";

        // Attach depth texture to FBO.
        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, texture_id,
                             0);
        if (!Check()) return false;

        attached_depth_texture_ = texture_id;
        return true;
    }

    /**
     * Specify the draw buffer of this framebuffer. It allow framebuffer render
     * to more than one color textures.
     */
    void SetDrawBuffers(const Array<int>& color_attachments_id) {
        Array<GLuint> color_attachments;
        for (int id : color_attachments_id) {
            CHECK(color_attachments_.find(id) != color_attachments_.end())
                    << "GL_COLOR_ATTACHMENT" + std::to_string(id)
                    << " are not attached to this framebuffer.";
            color_attachments.push_back(GL_COLOR_ATTACHMENT0 + id);
        }
        glDrawBuffers(color_attachments.size(), color_attachments.data());
    }

    /**
     * Transfer a given attached color buffer to another framebuffer.
     *
     * It is useful when to transfer the buffer from multi-sampled texture to
     * single-sampled texture, or from multi-render targets to single-render
     * target.
     */
    void TransferColorBuffer(int id, Framebuffer* target) const {
        CHECK(id >= 0 && id < MAX_COLOR_ATTCHMENTS);
        CHECK(target);

        if (color_attachments_.find(id) == color_attachments_.end()) return;

        this->Bind();
        glBindFramebuffer(GL_READ_FRAMEBUFFER, id_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, target->id());
        glReadBuffer(GL_COLOR_ATTACHMENT0 + id);
        glBlitFramebuffer(0, 0, width_, height_,
                          0, 0, target->width(), target->height(),
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        this->Unbind();
    }

    /**
     * Get current framebuffer ID in Opengl.
     */
    static GLuint GetCurrentFramebuffer() {
        int id = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &id);
        return GLuint(id);
    }

    /**
     * Get the color texture attched on GL_COLOR_ATTACHMENT(id).
     */
    GLuint color_texture(int id) const {
        CHECK(id >= 0 && id < MAX_COLOR_ATTCHMENTS);
        CHECK(GetCurrentFramebuffer() == id_) << "Need to call Bind().";
        return color_textures_[id];
    }

    /**
     * Get the color buffer attched on GL_COLOR_ATTACHMENT(id).
     */
    GLuint color_buffer(int id) const {
        CHECK(id >= 0 && id < MAX_COLOR_ATTCHMENTS);
        CHECK(GetCurrentFramebuffer() == id_) << "Need to call Bind().";
        return color_buffers_[id];
    }

    GLuint id()                     const { return id_;                     }
    int width()                     const { return width_;                  }
    int height()                    const { return height_;                 }
    GLuint depth_buffer()           const { return depth_buffer_;           }
    GLuint attached_depth_texture() const { return attached_depth_texture_; }

protected:
    /**
     * Erase id-th color buffer.
     */
    void EraseColorBuffer(int id) {
        if (color_buffers_[id]) {
            glDeleteRenderbuffers(1, &color_buffers_[id]);
            color_buffers_[id] = 0;
        }
    }

    // ID of FBO in OpenGL.
    GLuint id_ = 0;

    // Old draw framebuffer.
    mutable GLuint old_draw_framebuffer_ = 0;

    // The dimension of framebuffer.
    int height_ = 0;
    int width_ = 0;

    // Attached depth buffer.
    GLuint attached_depth_texture_ = 0;

    // Depth render buffer in framebuffer.
    GLuint depth_buffer_ = 0;

    // Attached color textures.
    GLuint color_textures_[MAX_COLOR_ATTCHMENTS];

    // Color render buffer in framebuffer.
    GLuint color_buffers_[MAX_COLOR_ATTCHMENTS];

    // Used to check if a color attachment is used.
    std::unordered_set<int> color_attachments_;
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_FRAMEBUFFER_H_
