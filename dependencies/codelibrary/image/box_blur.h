//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_BOX_BLUR_H_
#define CODELIBRARY_IMAGE_BOX_BLUR_H_

#include "codelibrary/base/array_nd.h"
#include "codelibrary/image/image.h"

namespace cl {
namespace image {

/**
 * A box filter is a spatial domain linear filter in which each pixel in the
 * resulting image has a value equal to the average value of its neighboring
 * pixels in the input image.
 *
 * It is a form of low-pass ("blurring") filter. A 3 by 3 box blur ("radius 1")
 * can be written as matrix:
 *     [ 1 1 1 ]
 * 1/9 [ 1 1 1 ]
 *     [ 1 1 1 ]
 *
 * Due to its property of using equal weights, it can be implemented using a
 * much simpler accumulation algorithm, which is significantly faster than using
 * a sliding-window algorithm.
 *
 * Box filter are frequently used to approximate a Gaussian filter. By the
 * central limit theorem, repeated application of a box blur will approximate a
 * Gaussian blur.
 *
 * The following implement use a fast accumulator algorithm. The time cost is
 * O(N + k) not O(kN).
 */
template <typename T>
void BoxBlur(const BaseImage<T>& image, int kernel_radius,
             BaseImage<T>* filtered_image) {
    CHECK(kernel_radius >= 1);
    CHECK(kernel_radius < 16384);
    CHECK(filtered_image);

    const int w = image.width(), h = image.height(), c = image.n_channels();
    const int kernel_size = 2 * kernel_radius + 1;
    const int offset = kernel_radius + 1;
    ArrayND<double> accumulator(h + kernel_size, w + kernel_size, c);

    for (int i = 1; i < h + kernel_size; ++i) {
        for (int j = 1; j < w + kernel_size; ++j) {
            for (int k = 0; k < c; ++k) {
                int x = Clamp(i - offset, 0, h - 1);
                int y = Clamp(j - offset, 0, w - 1);
                accumulator(i, j, k) = accumulator(i - 1, j, k) +
                                       accumulator(i, j - 1, k) -
                                       accumulator(i - 1, j - 1, k) +
                                       image(x, y, k);
            }
        }
    }

    bool is_integer = std::is_integral<T>::value;

    filtered_image->Reset(h, w, c);
    T* out = filtered_image->data();
    const int r = kernel_radius;
    double norm = 1.0 / (kernel_size * kernel_size);
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            for (int k = 0; k < c; ++k) {
                double t = (accumulator(offset + i + r, offset + j + r, k) -
                            accumulator(i, offset + j + r, k) -
                            accumulator(offset + i + r, j, k) +
                            accumulator(i, j, k)) * norm;
                *out++ = is_integer ? static_cast<T>(t >= 0.0 ? t + 0.5
                                                              : t - 0.5)
                                    : t;
            }
        }
    }
}

} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_BOX_BLUR_H_
