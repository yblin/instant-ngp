//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_IMAGE_H_
#define CODELIBRARY_IMAGE_IMAGE_H_

#include <cstdint>

#include "codelibrary/base/log.h"
#include "codelibrary/util/io/file_util.h"
#include "codelibrary/util/color/rgb32_color.h"

#define STB_IMAGE_IMPLEMENTATION
#include "third_party/stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "third_party/stb/stb_image_write.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "third_party/stb/stb_image_resize.h"

namespace cl {

/**
 * Basic image class.
 */
template <typename T>
class BaseImage {
public:
    using Byte = T;
    using Iterator = Byte*;
    using ConstIterator = const Byte*;

    enum Mode {
        L, RGB, RGBA
    };

    BaseImage() = default;

    /**
     * Create an image.
     */
    BaseImage(int h, int w, int c = 1, Byte byte = Byte()) {
        Reset(h, w, c);
        Fill(byte);
    }

    /**
     * Create an image.
     */
    BaseImage(Mode mode, int h, int w, Byte byte = Byte()) {
        Reset(mode, w, h);
        Fill(byte);
    }

    /**
     * Create an image from the given file.
     */
    BaseImage(const std::string& filename, bool flip = false) {
        Load(filename, flip);
    }

    /**
     * Reset the dimension of the image.
     *
     * Note that it dose not clear the data.
     */
    void Reset(int h, int w, int c = 1) {
        CHECK(h >= 0 && w >= 0);
        CHECK(c > 0 && c <= 4);

        height_     = h;
        width_      = w;
        n_channels_ = c;
        ResetMode();

        CHECK(w == 0 || h <= INT_MAX / w) << "Image is too large.";
        CHECK(h * w <= INT_MAX / c) << "Image is too large.";

        data_.resize(h * w * c);
    }

    /**
     * Reset the image data and the dimension.
     */
    void Reset(int h, int w, int c, const Array<Byte>& data) {
        CHECK(h >= 0 && w >= 0);
        CHECK(c > 0 && c <= 4);
        CHECK(data.size() == h * w * c);

        height_     = h;
        width_      = w;
        n_channels_ = c;
        ResetMode();

        CHECK(w == 0 || h <= INT_MAX / w) << "Image is too large.";
        CHECK(h * w <= INT_MAX / c) << "Image is too large.";

        data_ = data;
    }

    /**
     * Reset the image mode and dimension.
     */
    void Reset(Mode mode, int h, int w) {
        CHECK(h >= 0 && w >= 0);

        mode_       = mode;
        height_     = h;
        width_      = w;
        ResetChannels();

        CHECK(w == 0 || h <= INT_MAX / w) << "Image is too large.";
        CHECK(h * w <= INT_MAX) << "Image is too large.";

        data_.resize(h * w * n_channels_);
    }

    /**
     * Reset the image data and the dimension.
     */
    void Reset(Mode mode, int h, int w, const Array<Byte>& data) {
        CHECK(h >= 0 && w >= 0);

        mode_       = mode;
        height_     = h;
        width_      = w;
        ResetChannels();

        CHECK(w == 0 || h <= INT_MAX / w) << "Image is too large.";
        CHECK(h * w <= INT_MAX / n_channels_) << "Image is too large.";

        CHECK(w * h * n_channels_ != data.size())
                << "The size of the image does not match the size of the data.";
        data_ = data;
    }

    /**
     * Load image from the file.
     */
    virtual bool Load(const std::string& filename, bool flip = false) {
        stbi_set_flip_vertically_on_load(flip);
        Byte* data = STBILoad(filename.c_str());
        if (!data) {
            LOG(INFO) << "Error when loading the image: '" << filename << "'";
            LOG(INFO) << "Error message: " << stbi_failure_reason();
            return false;
        }

        data_.resize(width_ * height_ * n_channels_);
        std::copy(data, data + size(), data_.data());
        stbi_image_free(data);
        ResetMode();
        return true;
    }

    /**
     * Save this image to the file.
     */
    virtual bool Save(const std::string& filename) const;

    /**
     * Resize the image.
     */
    virtual void Resize(int h, int w) {
        CHECK(h >= 0 && w >= 0);

        if (empty()) return;
        if (h == 0 || w == 0) {
            this->clear();
            return;
        }

        CHECK(h <= INT_MAX / w) << "Image is too large.";
        CHECK(h * w <= INT_MAX / n_channels_) << "Image is too large.";

        if (height_ == h && width_ == w) return;

        Array<Byte> data(h * w * n_channels_);
        STBIRResize(h, w, data.data());
        height_ = h;
        width_ = w;
        data_.swap(data);
    }

    /**
     * Fill the image with the given value.
     */
    void Fill(Byte value) {
        std::fill(data_.begin(), data_.end(), value);
    }

    /**
     * Clear the data of the image.
     */
    void clear() {
        data_.clear();
        height_ = width_ = 0;
        n_channels_ = 1;
        mode_ = L;
    }

    /**
     * Swap the image.
     */
    void swap(BaseImage* image) {
        std::swap(mode_, image->mode_);
        std::swap(width_, image->width_);
        std::swap(height_, image->height_);
        std::swap(n_channels_, image->n_channels_);
        data_.swap(image->data());
    }

    /**
     * Check if the image is empty.
     */
    bool empty() const {
        return data_.empty();
    }

    Iterator begin()            { return data_.begin(); }
    ConstIterator begin() const { return data_.begin(); }
    Iterator end()              { return data_.end();   }
    ConstIterator end()   const { return data_.end();   }

    /**
     * Get the pixel at (i, j, k).
     */
    Byte operator()(int i, int j, int k) const {
        return data_[i * width_ * n_channels_ + j * n_channels_ + k];
    }
    Byte& operator()(int i, int j, int k) {
        return data_[i * width_ * n_channels_ + j * n_channels_ + k];
    }

    /**
     * Get the pixel at (i, j, 0).
     */
    Byte operator()(int i, int j) const {
        return this->operator()(i, j, 0);
    }
    Byte& operator()(int i, int j) {
        return this->operator()(i, j, 0);
    }

    /**
     * Return the pixel value at (h, w, c).
     */
    Byte at(int h, int w, int c) const {
        CHECK(h >= 0 && h < height_);
        CHECK(w >= 0 && w < width_);
        CHECK(c >= 0 && c < n_channels_);

        return this->operator()(h, w, c);
    }

    Byte& at(int h, int w, int c) {
        CHECK(h >= 0 && h < height_);
        CHECK(w >= 0 && w < width_);
        CHECK(c >= 0 && c < n_channels_);

        return this->operator()(h, w, c);
    }

    /**
     * Return the pixel value at (h, w, 0).
     */
    Byte at(int h, int w) const {
        return at(h, w, 0);
    }
    Byte& at(int h, int w) {
        return at(h, w, 0);
    }

    /**
     * Get the data pointer at the i-th line.
     */
    const Byte* line(int i) const {
        CHECK(i >= 0 && i < height_);

        return data_.data() + i * line_bytes();
    }
    Byte* line(int i) {
        CHECK(i >= 0 && i < height_);

        return data_.data() + i * line_bytes();
    }

    /**
     * Convert this image to array.
     */
    const Array<T>& to_array() const {
        return data_;
    }

    Mode mode()        const { return mode_;                          }
    int height()       const { return height_;                        }
    int width()        const { return width_;                         }
    int n_channels()   const { return n_channels_;                    }
    int line_bytes()   const { return width_ * n_channels_;           }
    int n_pixels()     const { return height_ * width_;               }
    int size()         const { return height_ * width_ * n_channels_; }
    Byte* data()             { return data_.data();                   }
    const Byte* data() const { return data_.data();                   }

protected:
    /**
     * Reset mode of image according to the number of channels.
     */
    void ResetMode() {
        switch (n_channels_) {
        case 1:
            mode_ = L;
            break;
        case 3:
            mode_ = RGB;
            break;
        case 4:
            mode_ = RGBA;
            break;

        default:
            CHECK(false);
        }
    }

    /**
     * Reset number of channels accroding to the image mode.
     */
    void ResetChannels() {
        switch(mode_) {
        case L:
            n_channels_ = 1;
            break;
        case RGB:
            n_channels_ = 3;
            break;
        case RGBA:
            n_channels_ = 4;
            break;

        default:
            CHECK(false);
        }
    }

    /**
     * Lod image by STBI.
     */
    Byte* STBILoad(const char* filename);

    /**
     * Resize image by STBIR.
     */
    void STBIRResize(int w, int h, Byte* dst);

    Mode mode_ = L;      // Mode of the image.
    int width_ = 0;      // Width of the image.
    int height_ = 0;     // Height of the image.
    int n_channels_ = 1; // Number of channels.
    Array<Byte> data_;   // Image data.
};

template <>
uint8_t* BaseImage<uint8_t>::STBILoad(const char* filename) {
    return stbi_load(filename, &width_, &height_, &n_channels_, 0);
}

template <>
float* BaseImage<float>::STBILoad(const char* filename) {
    return stbi_loadf(filename, &width_, &height_, &n_channels_, 0);
}

template <>
bool BaseImage<uint8_t>::Save(const std::string& filename) const {
    if (empty()) {
        LOG(INFO) << "Warning, you are saving an empty image.";
        return true;
    }

    const std::string suffix = file_util::GetSuffix(filename);
    int success = 0;
    if (suffix == "jpg" || suffix == "jpeg") {
        success = stbi_write_jpg(filename.c_str(), width_, height_,
                                 n_channels_, data_.data(), 100);
    } else if (suffix == "tga") {
        success = stbi_write_tga(filename.c_str(), width_, height_,
                                 n_channels_, data_.data());
    } else if (suffix == "png") {
        success = stbi_write_tga(filename.c_str(), width_, height_,
                                 n_channels_, data_.data());
    } else if (suffix == "bmp") {
        success = stbi_write_bmp(filename.c_str(), width_, height_,
                                 n_channels_, data_.data());
    } else {
        LOG(INFO) << "Unsupported file suffix: " << suffix;
        return false;
    }

    if (!success) {
        LOG(INFO) << stbi_failure_reason();
    }

    return success;
}

template <>
bool BaseImage<float>::Save(const std::string& filename) const {
    if (empty()) {
        LOG(INFO) << "Warning, you are saving an empty image.";
        return true;
    }

    const std::string suffix = file_util::GetSuffix(filename);
    int success = 0;
    if (suffix == "hdr") {
        success = stbi_write_hdr(filename.c_str(), width_, height_,
                                 n_channels_, data_.data());
    } else {
        LOG(INFO) << "Unsupported file suffix: " << suffix;
        return false;
    }

    if (!success) {
        LOG(INFO) << stbi_failure_reason();
    }

    return success;
}

template <>
void BaseImage<uint8_t>::STBIRResize(int h, int w, uint8_t* dst) {
    stbir_resize_uint8(data(), width_, height_, 0, dst, w, h, 0, n_channels_);
}

template <>
void BaseImage<float>::STBIRResize(int h, int w, float* dst) {
    stbir_resize_float(data(), width_, height_, 0, dst, w, h, 0, n_channels_);
}

using Image  = BaseImage<uint8_t>;
using ImageF = BaseImage<float>;

} // namespace cl

#endif // CODELIBRARY_IMAGE_IMAGE_H_
