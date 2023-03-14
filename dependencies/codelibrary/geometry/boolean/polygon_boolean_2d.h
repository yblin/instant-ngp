//
// Copyright 2011-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_

#include <queue>

#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/even_odd_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D polygon boolean operations.
 *
 * This class depends on Arrangment2D.
 */
template <typename T>
class PolygonBoolean2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point = Point2D<T>;
    using Mesh = HalfedgeList<Point>;
    using Vertex = typename Mesh::Vertex;
    using Halfedge = typename Mesh::Halfedge;

    // Color flag for mesh halfedges.
    enum Color {
        BLACK  = 0x00,
        RED    = 0x01,
        BLUE   = 0x02,
        PURPLE = 0x03
    };

public:
    PolygonBoolean2D(const MultiPolygon2D<T>& red_polygon,
                     const MultiPolygon2D<T>& blue_polygon) {
        Initialize(red_polygon, blue_polygon);
    }

    PolygonBoolean2D(const PolygonBoolean2D&) = delete;

    PolygonBoolean2D& operator=(const PolygonBoolean2D&) = delete;

    /**
     * Get the union of two polygons.
     */
    void Union(MultiPolygon2D<T>* result_polygon) {
        GetResult(BLACK, result_polygon);
    }

    /**
     * Get the intersection of two polygons.
     */
    void Intersection(MultiPolygon2D<T>* result_polygon) {
        GetResult(PURPLE, result_polygon);
    }

    /**
     * Get the difference of two polygons (Red polygon - Blue polygon).
     */
    void Difference(MultiPolygon2D<T>* result_polygon) {
        GetResult(RED, result_polygon);
    }

    /**
     * The given red polygon may be modified for robust computing.
     */
    const MultiPolygon2D<T>& modified_red_polygon() const {
        return modified_red_polygon_;
    }

    /**
     * The given blue polygon may be modified for robust computing.
     */
    const MultiPolygon2D<T>& modified_blue_polygon() const {
        return modified_blue_polygon_;
    }

protected:
    /**
     * Get the boolean results by given color.
     */
    void GetResult(int color, MultiPolygon2D<T>* result_polygon) const {
        CHECK(result_polygon);
        result_polygon->clear();

        auto flags = halfedge_list_.AddHalfedgeProperty(false);
        Array<Point2D<T>> points;
        for (const Halfedge* e : halfedge_list_) {
            if (flags[e]) continue;

            if (color_[e] == color) {
                points.clear();
                const Halfedge* e1 = e;
                do {
                    flags[e1] = true;
                    points.push_back(e1->source_point());
                    e1 = e1->next();
                } while (e1 != e);

                Polygon2D<T> polygon(points);
                if (polygon.empty()) continue;

                if (color == BLACK) {
                    result_polygon->Insert(polygon, polygon.IsClockwise());
                } else {
                    result_polygon->Insert(polygon, !polygon.IsClockwise());
                }
            }
        }
    }

    /**
     * Triangulation.
     */
    void Initialize(const MultiPolygon2D<T>& red_polygon,
                    const MultiPolygon2D<T>& blue_polygon) {
        Triangulation(red_polygon, blue_polygon);
        if (halfedge_list_.empty()) return;

        const auto& triangulation = arrangement_.triangulation();

        // Step 1. Deal the coincide edge and find the outside boundary of two
        // polygons.
        color_ = halfedge_list_.AddHalfedgeProperty("color", 0);
        for (Halfedge* e : halfedge_list_) {
            color_[e] = triangulation.color(e);
        }
        Halfedge* out_boundary = nullptr;
        for (Halfedge* e : halfedge_list_) {
            int t = color_[e] & color_[e->twin()];
            color_[e] -= t;
            color_[e->twin()] -= t;

            if (triangulation.is_outer(e) && !out_boundary)
                out_boundary = e;
        }
        if (!out_boundary) return;

        auto flag = halfedge_list_.AddHalfedgeProperty(false);
        std::queue<Halfedge*> q1;
        q1.push(out_boundary);
        flag[out_boundary] = true;

        while (!q1.empty()) {
            Halfedge* e = q1.front();
            q1.pop();
            color_[e] = BLACK;

            if (!flag[e->twin()]) {
                if (color_[e->twin()] == BLACK) {
                    flag[e->twin()] = true;
                    q1.push(e->twin());
                }
            }

            if (!flag[e->next()]) {
                flag[e->next()] = true;
                q1.push(e->next());
            }
        }

        // Step 2. Modify the color for each inner edge.
        std::queue<Halfedge*> q2;
        for (Halfedge* e : halfedge_list_) {
            if (!flag[e]) {
                q2.push(e);
            }
        }

        auto color = halfedge_list_.AddHalfedgeProperty(0);
        for (Halfedge* e : halfedge_list_) {
            color[e] = color_[e];
        }

        while (!q2.empty()) {
            Halfedge* e = q2.front();
            q2.pop();

            int c = color_[e->prev()] | color_[e->next()] | color_[e];
            color_[e] = c;

            if (!flag[e->twin()]) {
                int c1 = color_[e] - color[e];
                if (color_[e->twin()] != (color_[e->twin()] | c1)) {
                    color_[e->twin()] |= c1;
                    q2.push(e->twin());
                }
            }

            if (c != color_[e->next()]) {
                color_[e->next()] = c;
                q2.push(e->next());
            }

            if (c != color_[e->prev()]) {
                color_[e->prev()] = c;
                q2.push(e->prev());
            }
        }

        EraseExtraEdges();
    }

    /**
     * Initialize a triangulation of two polygons and compute the color of
     * edges.
     */
    void Triangulation(const MultiPolygon2D<T>& red_polygon,
                       const MultiPolygon2D<T>& blue_polygon) {
        if (red_polygon.empty() && blue_polygon.empty()) return;

        // Get the triangulation of two polygons.
        for (const auto& b : red_polygon.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.Insert(b.polygon.vertex(i),
                                        b.polygon.next_vertex(i),
                                        RED);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.Insert(b.polygon.next_vertex(i),
                                        b.polygon.vertex(i),
                                        RED);
                }
            }
        }
        for (const auto& b : blue_polygon.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.Insert(b.polygon.vertex(i),
                                        b.polygon.next_vertex(i),
                                        BLUE);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                   arrangement_.Insert(b.polygon.next_vertex(i),
                                       b.polygon.vertex(i),
                                       BLUE);
                }
            }
        }

        // The topological of input polygons maybe changed, so we need compute
        // the new polygons.
        Array<Segment2D<T>> edges1, edges2;
        for (const Halfedge* e : arrangement_.mesh()) {
            if ((arrangement_.color(e) & RED) == RED) {
               edges1.emplace_back(e->source_point(), e->target_point());
           }
           if ((arrangement_.color(e) & BLUE) == BLUE) {
               edges2.emplace_back(e->source_point(), e->target_point());
           }
        }

        EvenOddPolygon2D(edges1, &modified_red_polygon_);
        EvenOddPolygon2D(edges2, &modified_blue_polygon_);

        // Recompute the arrangement of new polygons.
        arrangement_.clear();

        // Get the triangulation of two polygons.
        for (const auto& b : modified_red_polygon_.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.InsertWithoutCross(
                          b.polygon.vertex(i), b.polygon.next_vertex(i), RED);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.InsertWithoutCross(
                          b.polygon.next_vertex(i), b.polygon.vertex(i), RED);
                }
            }
        }
        for (const auto& b : modified_blue_polygon_.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    arrangement_.InsertWithoutCross(
                          b.polygon.vertex(i), b.polygon.next_vertex(i), BLUE);
                }
            } else {
                for (int i = 0; i < b.polygon.size(); ++i) {
                   arrangement_.InsertWithoutCross(
                         b.polygon.next_vertex(i), b.polygon.vertex(i), BLUE);
                }
            }
        }

        arrangement_.mesh().Clone(&halfedge_list_);
    }

    /**
     * Erase the extra halfedges.
     */
    void EraseExtraEdges() {
        Array<Halfedge*> delete_edges;

        // The edge is useless if the color is the same with its twin's color.
        for (Halfedge* e : halfedge_list_) {
            if (e->id() % 2 == 0 && color_[e] == color_[e->twin()]) {
                delete_edges.push_back(e);
            }
        }
        for (Halfedge* e : delete_edges) {
            halfedge_list_.EraseEdge(e);
        }
    }

    // Halfedge list to represent the boolean result.
    Mesh halfedge_list_;

    // Color for each halfedge.
    typename Mesh::template HalfedgeProperty<int> color_;

    // For polygon edges arrangement.
    Arrangement2D<T> arrangement_;

    // Two input polygons will be modified to ensure the topology.
    MultiPolygon2D<T> modified_red_polygon_, modified_blue_polygon_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_BOOLEAN_2D_H_
