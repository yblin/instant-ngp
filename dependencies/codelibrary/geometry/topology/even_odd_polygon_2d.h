//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_

#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/winding_number.h"

namespace cl {
namespace geometry {

/**
 * Convert 2D Arrangement to multi-polygon via even-odd-rule.
 */
template <typename T>
void EvenOddPolygon2D(const Arrangement2D<T>& arrangement,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);

    using Point    = Point2D<T>;
    using DCEL     = HalfedgeList<Point>;
    using Halfedge = typename DCEL::Halfedge;

    polygon->clear();
    if (arrangement.empty()) return;

    Array<int> winding;
    WindingNumber(arrangement, &winding);

    DCEL list;
    arrangement.ToHalfedgeList(&list);

    auto visited = list.AddHalfedgeProperty(false);
    Array<Point> points;
    for (Halfedge* e : list) {
        int id = arrangement.region_id(e);
        if (id == -1) continue;

        if (winding[id] % 2 == 1 && !visited[e]) {
            points.clear();
            for (Halfedge* e1 : list.circular_list(e)) {
                visited[e1] = true;
                points.push_back(e1->source_point());
            }
            Polygon2D<T> poly(points);
            if (!poly.empty()) {
                polygon->Insert(poly, !poly.IsClockwise());
            }
        }
    }
}

/**
 * Get even-odd polygon of a given vertices loop.
 */
template <typename T>
void EvenOddPolygon2D(const Array<Point2D<T>>& points,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);

    polygon->clear();
    if (points.empty()) return;

    Arrangement2D<T> arrangement;
    for (int i = 0; i + 1 < points.size(); ++i) {
        arrangement.Insert(points[i], points[i + 1]);
    }
    arrangement.Insert(points.back(), points.front());
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, polygon);
}

/**
 * Get even-odd polygon consisting of multiple line segments.
 */
template <typename T>
void EvenOddPolygon2D(const Array<Segment2D<T>>& segs,
                      MultiPolygon2D<T>* polygon) {
    CHECK(polygon);

    polygon->clear();
    if (segs.empty()) return;

    Arrangement2D<T> arrangement;
    for (const Segment2D<T>& seg : segs) {
        arrangement.Insert(seg.lower_point(), seg.upper_point());
    }
    arrangement.Arrange();
    EvenOddPolygon2D(arrangement, polygon);
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_EVEN_ODD_POLYGON_2D_H_
