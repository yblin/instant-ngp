//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_MORPHOLOGY_H_
#define CODELIBRARY_IMAGE_MORPHOLOGY_H_

#include <cmath>
#include <queue>

#include "codelibrary/base/array_nd.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/image/image.h"
#include "codelibrary/image/distance_transform.h"

namespace cl {
namespace image {
namespace morphology {

/**
 * In morphology, we only care about the coordinate of pixels.
 */
using Pixel = IPoint2D;

/**
 * In morphology, a structuring element is a shape, used to probe or interact
 * with a given image, with the purpose of drawing conclusions on how this shape
 * fits or misses the shapes in the image.
 *
 * It is typically used in morphological operations, such as dilation, erosion,
 * opening, and closing, as well as the hit-or-miss transform.
 */
using StructuringElement = Array<Pixel>;

/**
 * Generate a disk structuring element.
 */
struct Disk : public StructuringElement {
    explicit Disk(double r) {
        CHECK(r >= 0.0 && r <= INT_MAX);

        double inner_r = r / std::sqrt(2.0);
        double r2 = r * r;
        int radius = std::ceil(r);
        for (int x = -radius; x <= radius; ++x) {
            for (int y = -radius; y <= radius; ++y) {
                if ((std::abs(x) <= inner_r && std::abs(y) <= inner_r) ||
                    static_cast<double>(x) * x +
                    static_cast<double>(y) * y <= r2) {
                    this->emplace_back(x, y);
                }
            }
        }
    }
};

/**
 * Generate a square structuring element.
 *
 * Every pixel along the perimeter has a chessboard distance no greater than
 * radius (radius=floor(w/2)) pixels.
 */
struct Square : public StructuringElement {
    explicit Square(int w) {
        CHECK(w > 0);

        int l = -(w - 1) / 2;
        int r = w / 2;
        for (int x = l; x <= r; ++x)
            for (int y = l; y <= r; ++y)
                this->emplace_back(x, y);
    }
};

/**
 * Generate a diamond-shaped structuring element.
 *
 * A pixel is part of the neighborhood if the city block/Manhattan distance
 * between it and the center of the neighborhood is no greater than radius.
 */
struct Diamond : public StructuringElement {
    explicit Diamond(int r) {
        CHECK(r > 0);

        for (int x = -r; x <= r; ++x) {
            for (int y = -r; y <= r; ++y) {
                if (std::abs(x) + std::abs(y) <= r) {
                    this->emplace_back(x, y);
                }
            }
        }
    }
};

/**
 * Get the boundary pixels of an binary image.
 * The content of image is defined by 'content_pixel'.
 */
template <typename StructuringElement>
void GetBoundary(const Image& image, int content_pixel,
                 const StructuringElement& neighbor,
                 Array<Pixel>* boundary) {
    CHECK(image.n_channels() == 1);
    CHECK(boundary);

    boundary->clear();
    const Image::Byte* data = image.data();

    int w = image.width(), h = image.height();
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            if (data[y * w + x] == content_pixel) {
                for (const Pixel& p : neighbor) {
                    if (p.x == 0 && p.y == 0) continue;
                    int x1 = x + p.x, y1 = y + p.y;
                    if (x1 < 0 || x1 >= w || y1 < 0 || y1 >= h ||
                        data[y1 * w + x1] != content_pixel)
                        boundary->emplace_back(x, y);
                }
            }
        }
    }
}
inline void GetBoundary(const Image& image, int content_pixel,
                        Array<Pixel>* boundary) {
    GetBoundary(image, content_pixel, Square(3), boundary);
}

/**
 * Compute morphological dilation of an image using square kernel.
 *
 * Morphological dilation sets a pixel at (x,y) to the maximum over all pixels
 * in the neighborhood centered at (x,y):
 *
 *   dst(x, y) =        max          src(x + x', y + y')
 *              (x',y') \in N(x, y)
 *
 * Here, we assume that the given image is a binary image, the pixel value is
 * either 0 or 255.
 */
inline void BinaryDilate(const Image& image, int radius, Image* result) {
    CHECK(image.n_channels() == 1);
    CHECK(radius >= 0);
    CHECK(result);

    *result = image;
    if (radius == 0) return;

    ManhattanDistanceTransform dt(image, 255);
    const Array<double>& dis = dt.distance_map();
    Image::Byte* data = result->data();
    for (int i = 0; i < dis.size(); ++i) {
        data[i] = (dis[i] <= radius) ? 255 : 0;
    }
}

/**
 * Compute morphological erosion of an image using square kernel.
 *
 * Morphological dilation sets a pixel at (x,y) to the maximum over all pixels
 * in the neighborhood centered at (x,y):
 *
 *   dst(x, y) =        max          src(x + x', y + y')
 *              (x',y') \in N(x, y)
 *
 * Here, we assume that the given image is a binary image, the pixel value is
 * either 0 or 255.
 */
inline void BinaryErode(const Image& image, int radius, Image* result) {
    CHECK(image.n_channels() == 1);
    CHECK(radius >= 0);
    CHECK(result);

    *result = image;
    if (radius == 0) return;

    ManhattanDistanceTransform dt(image, 0);
    const Array<double>& dis = dt.distance_map();
    Image::Byte* data = result->data();
    for (int i = 0; i < dis.size(); ++i) {
        data[i] = (dis[i] <= radius) ? 0 : 255;
    }
}

/**
 * Mask corresponding to a flood fill.
 * Starting at a specific seed_point, connected points equal or within tolerance
 * of the seed value are found.
 *
 * Parameters:
 *  image      - the image for operation.
 *  seed_point - the point in image used as the starting point for the flood
 *               fill.
 *  neighbors  - A structuring element used to determine the neighborhood of
 *               each evaluated pixel.
 *  tolerance  - A comparison will be done at every point and if within
 *               tolerance of the initial value will also be filled (inclusive).
 *  pixels     - the output flood filled pixels.
 */
template <typename StructuringElement>
void Flood(const Image& image, const Pixel& seed_point,
           const StructuringElement& neighbors,
           int tolerance,
           Array<Pixel>* pixels) {
    CHECK(pixels);
    CHECK(tolerance >= 0);

    int w = image.width(), h = image.height();
    CHECK(seed_point.x >= 0 && seed_point.x < w);
    CHECK(seed_point.y >= 0 && seed_point.y < h);

    ArrayND<bool> visited(w, h);
    visited.fill(false);

    visited(seed_point.x, seed_point.y) = true;
    pixels->clear();
    pixels->push_back(seed_point);
    int front = 0;
    while (front < pixels->size()) {
        Pixel p = (*pixels)[front++];
        int v = image(p.y, p.x);

        for (const Pixel& q : neighbors) {
            int x = q.x + p.x;
            int y = q.y + p.y;
            if (x >= 0 && x < w && y >= 0 && y < h) {
                if (!visited(x, y) && std::abs(image(y, x) - v) <= tolerance) {
                    visited(x, y) = true;
                    pixels->emplace_back(x, y);
                }
            }
        }
    }
}
template <typename StructuringElement>
void Flood(const Image& image, const Pixel& seed_point,
           const StructuringElement& neighbors,
           Array<Pixel>* pixels) {
    Flood(image, seed_point, neighbors, 0, pixels);
}

/**
 * Compute a label image so that all connected regions are assigned the same
 * integer value.
 *
 * return the number of labels.
 */
template <typename StructuringElement>
int GetLabelImage(const Image& image,
                  const StructuringElement& neighbors,
                  ArrayND<int>* labels) {
    CHECK(labels);

    int n_labels = 0;
    int w = image.width(), h = image.height();
    labels->reshape(h, w);

    ArrayND<bool> is_visited(h, w);
    is_visited.fill(false);

    Array<Pixel> pixels;
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            if (is_visited(i, j)) continue;
            
            Flood(image, Pixel(j, i), neighbors, &pixels);
            for (auto pixel : pixels) {
                is_visited(pixel.y, pixel.x) = true;
                (*labels)(pixel.y, pixel.x) = n_labels;
            }
            n_labels++;
        }
    }
    return n_labels;
}

} // namespace morphology
} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_MORPHOLOGY_H_
