//
// Copyright 2020-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_IMAGE_DISTANCE_TRANSFORM_H_
#define CODELIBRARY_IMAGE_DISTANCE_TRANSFORM_H_

#include <cfloat>
#include <cmath>

#include "codelibrary/base/clamp.h"
#include "codelibrary/image/image.h"

namespace cl {
namespace image {

/**
 * One-dimensional squared Euclidean distance transform.
 *
 * Df(p) =   min   ((p - q)^2 + f(q))
 *         q \in G
 */
struct SquaredEuclideanDistanceTransform1D {
    /**
     * Return the infinity distance.
     */
    double infinity() const {
        return Square(INT_MAX);
    }

    /**
     * Transform 1D array.
     */
    void operator ()(const Array<double>& f, int n, Array<double>* res) const {
        Array<double> z(n + 1);
        Array<int> v(n);

        v[0] = 0;
        z[0] = -DBL_MAX;
        z[1] = DBL_MAX;

        // Compute lower envelope.
        for (int q = 1, k = 0; q <= n - 1; ++q) {
            double s = Intersection(v[k], q, f[v[k]], f[q]);
            while (s <= z[k]) {
                --k;
                s = Intersection(v[k], q, f[v[k]], f[q]);
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = DBL_MAX;
        }

        // Fill in values of distance transform.
        res->resize(n);
        for (int q = 0, k = 0; q <= n - 1; ++q) {
            while (z[k + 1] < q)
                ++k;
            (*res)[q] = Square(q - v[k]) + f[v[k]];
        }
    }

protected:
    /**
     * Return x * x.
     */
    static double Square(int x) {
        return static_cast<double>(x) * x;
    }

    /**
     * Compute the intersection between two parabolas.
     */
    static double Intersection(int i, int u, double f_i, double f_u) {
        return (Square(u) - Square(i) + f_u - f_i) / (2.0 * (u - i));
    }
};

/**
 * One-dimensional Euclidean distance transform.
 *
 * Slower than squared Euclidean.
 */
struct EuclideanDistanceTransform1D
        : public SquaredEuclideanDistanceTransform1D {
    /**
     * Transform 1D array.
     */
    void operator ()(const Array<double>& f, int n, Array<double>* res) const {
        Array<double> z(n + 1);
        Array<int> v(n);

        v[0] = 0;
        z[0] = -DBL_MAX;
        z[1] = DBL_MAX;

        // Compute lower envelope.
        for (int q = 1, k = 0; q <= n - 1; ++q) {
            double s = Intersection(v[k], q, f[v[k]], f[q]);
            while (s <= z[k]) {
                --k;
                s = Intersection(v[k], q, f[v[k]], f[q]);
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = DBL_MAX;
        }

        // Fill in values of distance transform.
        res->resize(n);
        for (int q = 0, k = 0; q <= n - 1; ++q) {
            while (z[k + 1] < q)
                ++k;
            (*res)[q] = std::sqrt(Square(q - v[k]) + f[v[k]] * f[v[k]]);
        }
    }

protected:
    /**
     * Compute the intersection between two parabolas.
     */
    static double Intersection(int i, int u, double f_i, double f_u) {
        return (Square(u) - Square(i) + f_u * f_u - f_i * f_i) /
               (2.0 * (u - i));
    }
};

/**
 * One-dimensional Manhattan (L1) distance transform.
 */
struct ManhattanDistanceTransform1D {
    /**
     * Return the infinity distance.
     */
    double infinity() const {
        return INT_MAX;
    }

    /**
     * Transform 1D array.
     */
    void operator ()(const Array<double>& f, int n, Array<double>* res) const {
        *res = f;
        for (int q = 1; q < n; ++q) {
            (*res)[q] = std::min((*res)[q], (*res)[q - 1] + 1);
        }
        for (int q = n - 2; q >= 0; --q) {
            (*res)[q] = std::min((*res)[q], (*res)[q + 1] + 1);
        }
    }
};

/**
 * One-dimensional Chebyshev distance (L_inf) (chessboard distance) transform.
 */
struct ChebyshevDistanceTransform1D {
    /**
     * Return the infinity distance.
     */
    double infinity() const {
        return Square(INT_MAX);
    }

    /**
     * Transform 1D array.
     */
    void operator ()(const Array<double>& f, int n, Array<double>* res) const {
        Array<double> z(n + 1);
        Array<int> v(n);

        v[0] = 0;
        z[0] = -DBL_MAX;
        z[1] = DBL_MAX;

        // Compute lower envelope.
        for (int q = 1, k = 0; q <= n - 1; ++q) {
            double s = Intersection(v[k], q, f[v[k]], f[q]);
            while (s <= z[k]) {
                --k;
                s = Intersection(v[k], q, f[v[k]], f[q]);
            }
            ++k;
            v[k] = q;
            z[k] = s;
            z[k + 1] = DBL_MAX;
        }

        // Fill in values of distance transform.
        res->resize(n);
        for (int q = 0, k = 0; q <= n - 1; ++q) {
            while (z[k + 1] < q)
                ++k;
            (*res)[q] = std::max(std::fabs(q - v[k]), f[v[k]]);
        }
    }

private:
    /**
     * Return x * x.
     */
    static double Square(int x) {
        return static_cast<double>(x) * x;
    }

    /**
     * Compute the intersection between two parabolas.
     */
    static double Intersection(int i, int u, double f_i, double f_u) {
        if (f_i <= f_u)
            return std::max(i + f_u, static_cast<double>((i + u) / 2));
        else
            return std::min(u - f_i, static_cast<double>((i + u) / 2));
    }
};

/**
 * Let G be a regular grid (image) and f:G->R a sampled function on the grid.
 * The distance transform of f is defined as:
 *
 *     Df(p) =   min   (d(p, q) + f(q)).
 *             q \in G
 *
 * Here, d(p, q) is some measure of the distance (not necessarily a metric)
 * between p and q.
 *
 * Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). Distance transforms of
 * sampled functions. Theory of computing, 8(1), 415-428.
 */
template <class DistanceFunc>
class DistanceTransform {
public:
    /**
     * Construct a distance transform with sampled function f (defined by
     * array).
     */
    DistanceTransform(const Image& image, const Array<double>& f)
        : height_(image.height()),
          width_(image.width()),
          distance_map_(f) {
        CHECK(image.n_channels() == 1);
        CHECK(image.size() == f.size());

        Transform();
    }

    /**
     * Construct a distance transform with the specific pixel value of boundary.
     *
     * Here, the sampled function f is defined as:
     *
     *              0     , if p is a boundary pixel.
     *   f(p) = {
     *           infinity , otherwise.
     */
    explicit DistanceTransform(const Image& image, int boundary_pixel = 255)
        : height_(image.height()),
          width_(image.width()) {
        CHECK(image.n_channels() == 1);

        distance_map_.resize(image.size());
        for (int i = 0; i < image.size(); ++i) {
            if (image.data()[i] == boundary_pixel) {
                distance_map_[i] = 0.0;
            } else {
                distance_map_[i] = distance_func_.infinity();
            }
        }

        Transform();
    }

    /**
     * Convert distance map to image.
     */
    void ToImage(Image* image) const {
        CHECK(image);

        image->Reset(height_, width_);
        if (distance_map_.empty()) return;

        double min = *std::min_element(distance_map_.begin(),
                                       distance_map_.end());
        double max = *std::max_element(distance_map_.begin(),
                                       distance_map_.end());
        if (min == max) return;

        Image::Byte* data = image->data();
        for (int i = 0; i < distance_map_.size(); ++i) {
            double t = (distance_map_[i] - min) / (max - min);
            data[i] = Clamp(static_cast<int>(t * 255), 0, 255);
        }
    }

    /**
     * Return the distance map from each pixel to its nearest boundary pixel.
     */
    const Array<double>& distance_map() const {
        return distance_map_;
    }

    /**
     * Return the height of the given image.
     */
    int height() const {
        return height_;
    }

    /**
     * Return the width of the given image.
     */
    int width() const {
        return width_;
    }

    /**
     * Check if this distance transform is empty.
     */
    bool empty() const {
        return height_ == 0 || width_ == 0;
    }

private:
    /**
     * Compute distance map.
     */
    void Transform() {
        int w = width_;
        int h = height_;

        int n = std::max(w, h);
        Array<double> f(n), d;

        // Transform along columns.
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                f[y] = distance_map_[y * w + x];
            }

            distance_func_(f, h, &d);
            for (int y = 0; y < h; ++y) {
                distance_map_[y * w + x] = d[y];
            }
        }

        // Transform along rows.
        for (int y = 0; y < h; ++y) {
            for (int x = 0, k = y * w; x < w; ++x, ++k) {
                f[x] = distance_map_[k];
            }
            distance_func_(f, w, &d);
            for (int x = 0, k = y * w; x < w; ++x, ++k) {
                distance_map_[k] = d[x];
            }
        }
    }

    int height_ = 0;
    int width_ = 0;
    DistanceFunc distance_func_;
    Array<double> distance_map_;
};

using EuclideanDistanceTransform =
      DistanceTransform<EuclideanDistanceTransform1D>;
using SquaredEuclideanDistanceTransform =
      DistanceTransform<SquaredEuclideanDistanceTransform1D>;
using ManhattanDistanceTransform =
      DistanceTransform<ManhattanDistanceTransform1D>;
using ChebyshevDistanceTransform =
      DistanceTransform<ChebyshevDistanceTransform1D>;

} // namespace image
} // namespace cl

#endif // CODELIBRARY_IMAGE_DISTANCE_TRANSFORM_H_

