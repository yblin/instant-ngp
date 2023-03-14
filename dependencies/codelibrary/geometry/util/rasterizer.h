//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_RASTERIZER_H_
#define CODELIBRARY_GEOMETRY_UTIL_RASTERIZER_H_

#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/image/image.h"

namespace cl {
namespace geometry {

/**
 * Hardware independent rasterizer for 2D polygons.
 */
class Rasterizer {
    /**
     * Active edge for scan lines.
     */
    struct Edge {
        double x;
        double inverse_slope;
        RPoint2D low, high;
    };

    class ActiveEdgeList {
    public:
        template <typename T>
        explicit ActiveEdgeList(const MultiPolygon2D<T>& poly) {
            GetEdges(poly);
        }

        /**
         * Insert new active edges whose low_y <= y.
         */
        void Insert(double y) {
            // Insert new edges.
            while (front_ < edges_.size()) {
                if (edges_[front_].low.y <= y) {
                    active_edges_.push_back(edges_[front_++]);
                } else {
                    break;
                }
            }
        }

        /**
         * Erase unactive edges whose high_y >= y.
         */
        void Erase(double y) {
            // Remove the edges below y from active_edges.
            int k = 0;
            for (int j = 0; j < active_edges_.size(); ++j) {
                if (active_edges_[j].high.y > y)
                    active_edges_[k++] = active_edges_[j];
            }
            active_edges_.resize(k);
        }

        /**
         * Update X coordinates of active edges.
         */
        void Update(double y) {
            // Update X coordinates of active edges.
            for (Edge& edge : active_edges_) {
                edge.x = edge.low.x + (y - edge.low.y) * edge.inverse_slope;
            }

            // Re-sort active edges according to X coordinate.
            std::sort(active_edges_.begin(), active_edges_.end(),
                      [](const Edge& e1, const Edge& e2) {
                          return e1.x < e2.x;
                      });
        }

        const Array<Edge>& active_edges() const {
            return active_edges_;
        }

    private:
        /**
         * Get all edges from polygon.
         */
        template <typename T>
        void GetEdges(const MultiPolygon2D<T> polygon) {
            using Point = Point2D<T>;
            for (const auto& b : polygon.boundaries()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    Segment2D<T> e = b.polygon.edge(i);
                    if (e.lower_point().y == e.upper_point().y) continue;

                    Point p1 = e.lower_point();
                    Point p2 = e.upper_point();

                    Edge edge;
                    edge.low = p1.y < p2.y ? RPoint2D(p1.x, p1.y)
                                           : RPoint2D(p2.x, p2.y);
                    edge.high = p1.y < p2.y ? RPoint2D(p2.x, p2.y)
                                            : RPoint2D(p1.x, p1.y);
                    edge.inverse_slope = (edge.high.x - edge.low.x) /
                                         (edge.high.y - edge.low.y);
                    edges_.push_back(edge);
                }
            }

            // Sort edges according to the highest Y value.
            std::sort(edges_.begin(), edges_.end(), [](const Edge& e1,
                                                       const Edge& e2) {
                return e1.low.y < e2.low.y;
            });
        }

        int front_ = 0;
        Array<Edge> edges_;
        Array<Edge> active_edges_;
    };

public:
    /**
     * Horizental scan lines.
     */
    struct ScanLine {
        int left, right;
        int y;
    };

    Rasterizer(int h, int w)
        : height_(h), width_(w) {
        CHECK(height_ > 0);
        CHECK(width_ > 0);
        CHECK(INT_MAX / height_ >= width_);
    }

    /**
     * Rasterize the polygon to the horizental scan lines.
     * It only support 1 samples.
     */
    template <typename T>
    void Rasterize(const MultiPolygon2D<T>& polygon,
                   Array<ScanLine>* scan_lines) {
        CHECK(scan_lines);

        scan_lines->clear();

        Box2D<T> box(0, width_, 0, height_);
        if (!geometry::Intersect(box, polygon.bounding_box(), &box)) return;

        // Sweep line algorithm. Convert polygon into scanlines.
        ActiveEdgeList active_edge_list(polygon);
        int y_start = std::max(std::floor(box.y_min()), 0.0);
        int y_end = std::min(std::ceil(box.y_max()), height_ - 1.0);

        for (int y0 = y_start; y0 <= y_end; ++y0) {
            // Do not modify the following sequence.
            active_edge_list.Insert(y0 + 0.5);
            active_edge_list.Erase(y0 + 0.5);
            active_edge_list.Update(y0 + 0.5);

            // Fill the scan lines.
            const Array<Edge>& active_edges = active_edge_list.active_edges();
            ScanLine scan_line;
            scan_line.y = y0;
            for (int i = 1; i < active_edges.size(); i += 2) {
                double x0 = active_edges[i - 1].x;
                double x1 = active_edges[i].x;
                if (x1 < 0 || x0 > width_) return;

                int left = std::ceil(x0);
                left = std::max(0, left);
                int right = std::floor(x1);
                right = std::min(right, width_ - 1);

                if (left > right) {
                    if (x1 - x0 > 0.5) --left;
                } else {
                    if (x0 >= 0.0 && left - x0 > 0.5) --left;
                    if (x1 - right < 0.5) --right;
                }
                scan_line.left = left;
                scan_line.right = right;
                scan_lines->push_back(scan_line);
            }
        }
    }

    /**
     * Rasterize the polygon. For each pixel, it will generate 'n_samples' scan
     * lines to compute the approximate coverage between polygon and the pixel.
     */
    template <typename T>
    void Rasterize(const MultiPolygon2D<T>& polygon, int n_samples,
                   Image* image) const {
        assert(n_samples > 0);
        assert(image);

        image->Reset(height_, width_);
        image->Fill(0);

        Box2D<T> box(0, width_, 0, height_);
        if (!geometry::Intersect(box, polygon.bounding_box(), &box)) return;

        // Sweep line algorithm. Convert polygon into scanlines.
        ActiveEdgeList active_edge_list(polygon);
        int y_start = std::max(std::floor(box.y_min()), 0.0);
        int y_end = std::min(std::ceil(box.y_max()), height_ - 1.0);

        for (int y0 = y_start; y0 <= y_end; ++y0) {
            Array<double> scanline(width_, 0.0);

            for (int i = 0; i < n_samples; ++i) {
                double y = y0 + (1.0 + i) / (n_samples + 1);

                // Do not modify the following sequence.
                active_edge_list.Insert(y);
                active_edge_list.Erase(y);
                active_edge_list.Update(y);

                // Fill the scans.
                const auto& active_edges = active_edge_list.active_edges();
                for (int i = 1; i < active_edges.size(); i += 2) {
                    FillScanline(active_edges[i - 1].x, active_edges[i].x,
                                 n_samples, scanline.data());
                }
            }

            uint8_t* data = image->data() + (height_ - y0 - 1) * width_;
            for (int i = 0; i < width_; ++i) {
                data[i] = Clamp(static_cast<int>(scanline[i]), 0, 255);
            }
        }
    }

    template <typename T>
    void Rasterize(const Polygon2D<T>& polygon, int n_samples,
                   Image* image) const {
        Rasterize(MultiPolygon2D<T>(polygon), n_samples, image);
    }

    int width() const {
        return width_;
    }

    int height() const {
        return height_;
    }

private:
    /**
     * Fill the pixels covered by range [x0, x1].
     */
    void FillScanline(double x0, double x1, int n_samples,
                      double* scanline) const {
        if (x1 < 0 || x0 > width_) return;

        double weight = 255.0 / n_samples;

        int left = std::ceil(x0);
        left = std::max(0, left);
        int right = std::floor(x1);
        right = std::min(right, width_ - 1);

        for (int i = left; i < right; ++i) {
            scanline[i] += weight;
        }

        if (left > right) {
            // Only one pixel is covered.
            scanline[left - 1] += (x1 - x0) * weight;
        } else {
            if (x0 >= 0) scanline[left - 1] += (left - x0) * weight;
            if (x1 <= width_) scanline[right] += (x1 - right) * weight;
        }
    }

    int height_, width_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_RASTERIZER_H_
