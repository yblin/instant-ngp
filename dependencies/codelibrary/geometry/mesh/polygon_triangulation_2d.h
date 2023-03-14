//
// Copyright 2012-2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_POLYGON_TRIANGULATION_2D_H_
#define CODELIBRARY_GEOMETRY_MESH_POLYGON_TRIANGULATION_2D_H_

#include <queue>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/mesh/delaunay_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D polygon triangulation.
 *
 * This class depends on Delaunay triangulation.
 */
template <typename T>
class PolygonTriangulation2D : Delaunay2D<T> {
    using Point = Point2D<T>;
    using Mesh = HalfedgeList<Point>;
    using Vertex = typename Mesh::Vertex;
    using Halfedge = typename Mesh::Halfedge;
    using Polygon = Polygon2D<T>;
    using MultiPolygon = MultiPolygon2D<T>;

    static const int OUTER_HALFEDGE = 1;

public:
    explicit PolygonTriangulation2D(const Polygon& polygon) {
        MultiPolygon poly(polygon);
        Initialize(poly);
    }

    explicit PolygonTriangulation2D(const MultiPolygon& polygon) {
        Initialize(polygon);
    }

    PolygonTriangulation2D(const PolygonTriangulation2D&) = delete;

    PolygonTriangulation2D& operator=(const PolygonTriangulation2D&) = delete;

    const Mesh& mesh() const { return this->mesh_; }

    /**
     * Is outer or inner halfedge.
     */
    bool is_outer(const Halfedge* e) const {
        return this->color(e) == OUTER_HALFEDGE;
    }

    /**
     * Return true if the edge is a constraint edge.
     */
    bool is_constraint(const Halfedge* e) const {
        return this->is_constraint_[e];
    }

protected:
    /**
     * Initialize the constrained Delaunay triangulation.
     */
    void Initialize(const MultiPolygon& polygon) {
        Array<Point> points;
        for (const auto& b : polygon.boundaries()) {
            for (const Point& p : b.polygon) {
                points.push_back(p);
            }
        }
        this->Reset(points);

        for (const auto& b : polygon.boundaries()) {
            if (b.is_outer != b.polygon.IsClockwise()) {
                for (int j = 0; j < b.polygon.size(); ++j) {
                    if (!this->InsertEdge(b.polygon.next_vertex(j),
                                          b.polygon.vertex(j),
                                          OUTER_HALFEDGE)) {
                        CHECK(false)
                                << "The input polygon contains cross edges.";
                    }
                }
            } else {
                for (int j = 0; j < b.polygon.size(); ++j) {
                    if (!this->InsertEdge(b.polygon.vertex(j),
                                          b.polygon.next_vertex(j),
                                          OUTER_HALFEDGE)) {
                        CHECK(false)
                                << "The input polygon contains cross edges.";
                    }
                }
            }
        }

        // BFS to found every halfedge that at the outside bound of the polygon
        // (color == 1).
        Array<Halfedge*> delete_edges;
        std::queue<Halfedge*> queue;

        auto is_outer_edge = this->mesh_.AddHalfedgeProperty(false);

        for (Halfedge* e : this->mesh_) {
            if (this->color(e) == OUTER_HALFEDGE &&
                this->color(e->twin()) != OUTER_HALFEDGE) {
                is_outer_edge[e] = true;
                queue.push(e);
            }
        }

        while (!queue.empty()) {
            Halfedge* e = queue.front();
            queue.pop();

            Halfedge* e_next = e->next();
            if (!is_outer_edge[e_next]) {
                is_outer_edge[e_next] = true;
                is_outer_edge[e_next->twin()] = true;
                queue.push(e_next);
                queue.push(e_next->twin());
                delete_edges.push_back(e_next);
            }
        }

        for (Halfedge* e : delete_edges) {
            this->mesh_.EraseEdge(e);
        }
    }
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_POLYGON_TRIANGULATION_2D_H_
