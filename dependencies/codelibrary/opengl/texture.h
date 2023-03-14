//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_OPENGL_TEXTURE_H_
#define CODELIBRARY_OPENGL_TEXTURE_H_

#include <fstream>

#include "codelibrary/image/image.h"
#include "codelibrary/opengl/glad.h"

namespace cl {
namespace gl {

/**
 * 2D OpenGL texture from image.
 */
class Texture {
public:
    /**
     * Create a default empty texture.
     */
    Texture() = default;

    Texture(const Texture&) = delete;

    /**
     * Create a texture from file.
     */
    explicit Texture(const std::string& name, bool mipmap = false) {
        Load(name, mipmap);
    }

    explicit Texture(const Image& image, bool mipmap = false) {
        Load(image, mipmap);
    }

    explicit Texture(const ImageF& image, bool mipmap = false) {
        Load(image, mipmap);
    }

    Texture& operator =(const Texture&) = delete;

    virtual ~Texture() {
        clear();
    }

    void clear() {
        if (id_) {
            glDeleteTextures(1, &id_);
            id_ = 0;
        }
    }

    bool empty() const {
        return id_ == 0;
    }

    /**
     * Set texture minifying function.
     * It is used whenever the pixel being textured maps to an area greater than
     * one texture element.
     */
    void set_min_filter_function(int f) {
        CHECK(f == GL_LINEAR || f == GL_NEAREST);
        min_filter_function_ = f;
    }

    /**
     * Texture magnification function.
     * It is used when the pixel being textured maps to an area less than or
     * equal to one texture element.
     */
    void set_mag_filter_function(int f) {
        CHECK(f == GL_LINEAR || f == GL_NEAREST);
        mag_filter_function_ = f;
    }

    /**
     * Load texture from image.
     */
    bool Load(const Image& image, bool mipmap = false) {
        if (image.empty()) return false;

        Initialize(mipmap);

        int c = image.n_channels();
        CHECK(c > 0 && c <= 4);

        int w = image.width(), h = image.height();

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glBindTexture(GL_TEXTURE_2D, id_);
        switch (c) {
        case 1:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, w, h, 0, GL_RED,
                         GL_UNSIGNED_BYTE, image.data());
            break;
        case 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG, w, h, 0, GL_RG,
                         GL_UNSIGNED_BYTE, image.data());
            break;
        case 3:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
                         GL_UNSIGNED_BYTE, image.data());
            break;
        case 4:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, image.data());
            break;
        default:
            CHECK(false) << "Unknown image mode.";
        }

        if (mipmap) glGenerateMipmap(GL_TEXTURE_2D);

        return true;
    }

    /**
     * Load texture from image.
     */
    bool Load(const ImageF& image, bool mipmap = false) {
        if (image.empty()) return false;

        Initialize(mipmap);

        int c = image.n_channels();
        CHECK(c > 0 && c <= 4);

        int w = image.width(), h = image.height();

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glBindTexture(GL_TEXTURE_2D, id_);
        switch (c) {
        case 1:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, w, h, 0, GL_RED,
                         GL_FLOAT, image.data());
            break;
        case 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, w, h, 0, GL_RG,
                         GL_FLOAT, image.data());
            break;
        case 3:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB,
                         GL_FLOAT, image.data());
            break;
        case 4:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA,
                         GL_FLOAT, image.data());
            break;
        default:
            CHECK(false) << "Unknown image mode.";
        }

        if (mipmap) glGenerateMipmap(GL_TEXTURE_2D);

        return true;
    }

    /**
     * Load texture from image file.
     */
    bool Load(const std::string& file, bool mipmap = false, bool flip = false) {
        std::ifstream fin(file.c_str());
        if (!fin.good()) return false;
        fin.close();

        Image image;
        if (!image.Load(file, flip)) return false;
        return Load(image, mipmap);
    }

    /**
     * Create a empty unsigned byte texture.
     */
    void Create(int w, int h, int c, bool mipmap = false) {
        CHECK(h >= 0);
        CHECK(w >= 0);
        CHECK(c >= 0);

        Initialize(mipmap);

        switch (c) {
        case 1:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, w, h, 0, GL_RED,
                         GL_UNSIGNED_BYTE, nullptr);
            break;
        case 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG, w, h, 0, GL_RG,
                         GL_UNSIGNED_BYTE, nullptr);
            break;
        case 3:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
                         GL_UNSIGNED_BYTE, nullptr);
            break;
        case 4:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, nullptr);
            break;
        default:
            CHECK(false) << "Unknown image mode.";
        }

        if (mipmap) glGenerateMipmap(GL_TEXTURE_2D);
    }

    /**
     * Create a empty float texture.
     */
    void CreateF(int w, int h, int c, bool mipmap = false) {
        CHECK(h >= 0);
        CHECK(w >= 0);
        CHECK(c >= 0);

        Initialize(mipmap);

        switch (c) {
        case 1:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, w, h, 0, GL_RED,
                         GL_FLOAT, nullptr);
            break;
        case 2:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, w, h, 0, GL_RG,
                         GL_FLOAT, nullptr);
            break;
        case 3:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, w, h, 0, GL_RGB,
                         GL_FLOAT, nullptr);
            break;
        case 4:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, w, h, 0, GL_RGBA,
                         GL_FLOAT, nullptr);
            break;
        default:
            CHECK(false) << "Unknown image mode.";
        }

        if (mipmap) glGenerateMipmap(GL_TEXTURE_2D);
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

    GLuint id() const {
        return id_;
    }

private:
    /**
     * Initialize a new texture and set the parameters.
     */
    void Initialize(bool mipmap) {
        this->clear();
        glGenTextures(1, &id_);
        glBindTexture(GL_TEXTURE_2D, id_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        if (mipmap) {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                            GL_LINEAR_MIPMAP_LINEAR);
        } else {
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                            min_filter_function_);
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                        mag_filter_function_);
    }

    // Texture ID.
    GLuint id_ = 0;

    // Texture minifying function.
    // It is used whenever the pixel being textured maps to an area greater than
    // one texture element.
    int min_filter_function_ = GL_LINEAR;

    // Texture magnification function.
    // It is used when the pixel being textured maps to an area less than or
    // equal to one texture element.
    int mag_filter_function_ = GL_LINEAR;
};

} // namespace gl
} // namespace cl

#endif // CODELIBRARY_OPENGL_TEXTURE_H_
