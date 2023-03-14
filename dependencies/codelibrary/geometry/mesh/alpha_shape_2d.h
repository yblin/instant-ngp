//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_ALPHA_SHAPE_2D_H_
#define CODELIBRARY_GEOMETRY_MESH_ALPHA_SHAPE_2D_H_

#include "codelibrary/geometry/center.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/mesh/delaunay_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Get 2D Alpha shape as the form of tri-mesh.
 *
 * An alpha-shape associated with a set of points is a generalization of the
 * concept of the convex hull, i.e., every convex hull is an alpha-shape but not
 * every alpha shape is a convex hull.
 *
 * Reference:
 *   Edelsbrunner H, Kirkpatrick D, Seidel R. On the shape of a set of points in
 *   the plane[J]. IEEE Transactions on Information Theory, 1983, 29(4):551-559.
 */
template <typename T>
void AlphaShape2D(const Delaunay2D<T>& delaunay, double alpha,
                  HalfedgeList<Point2D<T>>* mesh) {
    CHECK(alpha > 0.0);
    CHECK(mesh);

    using Point = Point2D<T>;
    using DCEL = HalfedgeList<Point>;
    using Halfedge = typename DCEL::Halfedge;

    mesh->clear();
    mesh->ClearVertexProperites();
    mesh->ClearHalfedgeProperites();
    if (delaunay.empty()) return;

    delaunay.mesh().Clone(mesh);

    // Delete the triangles whose circumscribed circle radius bigger than
    // the given alpha value.
    Array<Halfedge*> delete_edges;
    auto is_visited = mesh->AddHalfedgeProperty(false);
    auto is_outer = mesh->AddHalfedgeProperty("is_outer", false);
    for (Halfedge* e : *mesh) {
        is_outer[e] = delaunay.is_outer(e);
    }

    for (Halfedge* e : *mesh) {
        if (!is_outer[e]) {
            RPoint2D p1(e->source_point().x, e->source_point().y);
            RPoint2D p2(e->target_point().x, e->target_point().y);
            RPoint2D p3(e->next()->target_point().x,
                        e->next()->target_point().y);

            RPoint2D c = Circumcenter(p1, p2, p3);
            double radius = Distance(c, p1);
            if (radius > alpha) {
                Halfedge* edges[3] = { e, e->prev(), e->next() };

                for (Halfedge* e : edges) {
                    is_outer[e] = true;

                    if (!is_visited[e] && is_outer[e->twin()]) {
                        is_visited[e] = true;
                        is_visited[e->twin()] = true;
                        delete_edges.push_back(e);
                    }
                }
            }
        }
    }
    for (Halfedge* e : delete_edges) {
        mesh->EraseEdge(e);
    }
}

/**
 * Get 2D Alpha shape as the form of multi-polygon.
 */
template <typename T>
void AlphaShape2D(const Delaunay2D<T>& delaunay, double alpha,
                  MultiPolygon2D<T>* shape) {
    CHECK(alpha > 0.0);
    CHECK(shape);

    using Point = Point2D<T>;
    using DCEL = HalfedgeList<Point>;
    using Halfedge = typename DCEL::Halfedge;

    DCEL mesh;
    AlphaShape2D(delaunay, alpha, &mesh);

    auto is_visited = mesh.AddHalfedgeProperty(false);
    auto is_outer = mesh.template GetHalfedgeProperty<bool>("is_outer");

    // Add the polygon loop into shape.
    Array<Point> points;
    for (Halfedge* e : mesh) {
        if (is_visited[e]) continue;
        if (!is_outer[e]) continue;

        points.clear();
        for (const Halfedge* e1 : mesh.circular_list(e)) {
            points.push_back(e1->source_point());
            is_visited[e1] = true;
        }
        Polygon2D<T> polygon(points);
        if (!polygon.empty()) {
            shape->Insert(polygon, polygon.IsClockwise());
        }
    }
}

/**
 * Get 2D Alpha shape as the form of multi-polygon.
 */
template <typename T>
void AlphaShape2D(const Array<Point2D<T>>& points, double alpha,
                  MultiPolygon2D<T>* shape) {
    Delaunay2D<T> delaunay(points);
    AlphaShape2D(delaunay, alpha, shape);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_ALPHA_SHAPE_2D_H_
