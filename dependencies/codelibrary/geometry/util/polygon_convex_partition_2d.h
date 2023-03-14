//
// Copyright 2012-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_POLYGON_CONVEX_PARTITION_2D_H_
#define CODELIBRARY_GEOMETRY_UTIL_POLYGON_CONVEX_PARTITION_2D_H_

#include <queue>

#include "codelibrary/geometry/mesh/polygon_triangulation_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * Fast convex partition the given polygon and store the results in
 * HalfedgeList.
 *
 * The time complexity is O(N).
 */
template <typename T>
void PolygonConvexPartition2D(const PolygonTriangulation2D<T>& pt,
                              HalfedgeList<Point2D<T>>* partition) {
    static_assert(std::is_floating_point<T>::value, "");

    CHECK(partition);

    using HalfedgeList = HalfedgeList<Point2D<T>>;
    using Halfedge = typename HalfedgeList::Halfedge;

    // Erase unnecessary triangulation edges.
    std::queue<Halfedge*> queue;

    pt.mesh().Clone(partition);
    for (Halfedge* e : *partition) {
        if (e->id() % 2 == 0 && !pt.is_outer(e) && !pt.is_outer(e->twin())) {
            queue.push(e);
        }
    }
    while (!queue.empty()) {
        Halfedge* e = queue.front();
        queue.pop();

        Halfedge* e_twin      = e->twin();
        Halfedge* e_next      = e->next();
        Halfedge* e_twin_prev = e_twin->prev();
        Halfedge* e_prev      = e->prev();
        Halfedge* e_twin_next = e_twin->next();

        if (Orientation(e_twin_prev->source_point(),
                        e_twin_prev->target_point(),
                        e_next->target_point()) >= 0 &&
            Orientation(e_prev->source_point(),
                        e_prev->target_point(),
                        e_twin_next->target_point()) >= 0) {
            partition->EraseEdge(e);
        }
    }
}
template <typename T>
void PolygonConvexPartition2D(const MultiPolygon2D<T>& polygon,
                              HalfedgeList<Point2D<T>>* partition) {
    PolygonTriangulation2D<T> pt(polygon);
    PolygonConvexPartition2D(pt, partition);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_POLYGON_CONVEX_PARTITION_2D_H_
