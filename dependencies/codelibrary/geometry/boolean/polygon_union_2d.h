//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_
#define CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_

#include <queue>

#include "codelibrary/graph/graph.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"
#include "codelibrary/geometry/topology/halfedge_graph.h"
#include "codelibrary/geometry/util/snap_2d.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace geometry {

/**
 * 2D polygon set union with snapping. It differs from PolygonBoolean that the
 * later one only support merge two polygons at a time. But this one can merge a
 * set of polygons, which is much faster.
 *
 * A suitable snap threshold is important for maintaining the topology of
 * polygons. Otherwise in some cases the inner contour may exceed the outer
 * contour causing topology errors.
 *
 * This class depends on Arrangment2D and Snap2D.
 */
template <typename T>
class PolygonUnion2D {
    static_assert(std::is_floating_point<T>::value, "");

    using Point   = Point2D<T>;
    using Polygon = MultiPolygon2D<T>;
    using DCEL    = HalfedgeList<Point>;
    using Vertex  = typename DCEL::Vertex;
    using Halfedge = typename DCEL::Halfedge;
    using Region = typename Arrangement2D<T>::Region;
    using TriFace = typename Arrangement2D<T>::TriFace;
    using EdgePropertyInt  = typename DCEL::template HalfedgeProperty<int>;

    // We merge 30 polygons at once to speed up.
    static const int BLOCK_SIZE = 30;

public:
    PolygonUnion2D() = default;

    PolygonUnion2D(const PolygonUnion2D&) = delete;

    PolygonUnion2D& operator=(const PolygonUnion2D&) = delete;

    /**
     * Union of polygon set with snap threshold.
     *
     * A suitable snap threshold is important for maintaining the topology of
     * polygons. Otherwise in some case the inner contour may exceed the outer
     * contour causing topology errors.
     */
    void operator()(const Array<Polygon>& polygons, double threshold,
                    Polygon* result) const {
        CHECK(result);
        CHECK(threshold > 0.0);

        result->clear();
        if (polygons.empty()) return;

        Array<Array<Polygon>> clusters;
        this->GroupClusters(polygons, threshold, &clusters);

        for (const Array<Polygon>& cluster : clusters) {
            Polygon t;
            if (cluster.size() <= BLOCK_SIZE) {
                SubProcess(cluster, threshold, &t);
            } else {
                MainProcess(cluster, threshold, &t);
            }
            for (auto& b : t) {
                result->Insert(b.polygon, b.is_outer);
            }
        }
    }

    /**
     * Similar to the previous one but return a set of polygon with holes.
     */
    void operator()(const Array<Polygon>& polygons, double threshold,
                    Array<Polygon>* results) const {
        CHECK(results);

        results->clear();

        Polygon poly;
        this->operator()(polygons, threshold, &poly);

        for (const auto& b : poly.boundaries()) {
            if (b.is_outer) {
                results->emplace_back(b.polygon);
            } else {
                CHECK(!results->empty());
                results->back().Insert(b.polygon, false);
            }
        }
    }

protected:
    /**
     * Return if the arrangement region is a inner region.
     */
    static bool IsInner(const Arrangement2D<T>& arrangement,
                        const Region& region,
                        const EdgePropertyInt& color) {
        for (const TriFace* face : region) {
            Halfedge* e = face->halfedge;
            if (arrangement.is_constraint(e))
                return color[e] == 0;
            if (arrangement.is_constraint(e->next()))
                return color[e->next()] == 0;
            if (arrangement.is_constraint(e->prev()))
                return color[e->prev()] == 0;
        }
        return false;
    }

    /**
     * Main process. It unions the subset of polygons with snapping.
     */
    void MainProcess(const Array<Polygon>& polygons, double threshold,
                     Polygon* result) const {
        if (polygons.empty()) return;
        if (polygons.size() == 1) {
            *result = polygons.front();
            return;
        }

        Array<Polygon> results;
        for (int i = 0; i < polygons.size(); i += BLOCK_SIZE) {
            int end = std::min(i + BLOCK_SIZE, polygons.size());
            Array<Polygon> sub_polygons(polygons.begin() + i,
                                        polygons.begin() + end);
            Polygon t;
            SubProcess(sub_polygons, threshold, &t);
            if (!t.empty()) results.push_back(t);
        }

        // Recursive.
        MainProcess(results, threshold, result);
    }

    /**
     * Group the intersected polygons into the same cluster.
     */
    void GroupClusters(const Array<Polygon>& polygons, double threshold,
                       Array<Array<Polygon>>* clusters) const {
        CHECK(clusters);

        Array<Box2D<T>> boxes;
        for (int i = 0; i < polygons.size(); ++i) {
            const Box2D<T>& b = polygons[i].bounding_box();
            boxes.emplace_back(b.x_min() - threshold, b.x_max() + threshold,
                               b.y_min() - threshold, b.y_max() + threshold);
        }
        DisjointSet set(polygons.size());
        for (int i = 0; i < polygons.size(); ++i) {
            for (int j = i + 1; j < polygons.size(); ++j) {
                if (Intersect(boxes[i], boxes[j])) {
                    set.Union(i, j);
                }
            }
        }

        Array<Array<int>> cs;
        set.ToClusters(&cs);
        clusters->resize(cs.size());
        for (int i = 0; i < cs.size(); ++i) {
            for (int v : cs[i]) {
                (*clusters)[i].push_back(polygons[v]);
            }
        }
    }

    /**
     * Compute the arrangement of polygons with snapping.
     */
    void Arrangement(const Array<Polygon>& polygons, double threshold,
                     Arrangement2D<T>* arrangement) const {
        CHECK(polygons.size() <= BLOCK_SIZE);
        CHECK(arrangement);

        Array<Segment2D<T>> segs;
        for (const Polygon& poly : polygons) {
            for (const auto& b : poly.boundaries()) {
                for (int i = 0; i < b.polygon.size(); ++i) {
                    segs.push_back(b.polygon.edge(i));
                }
            }
        }

        Array<Point> points;
        Cross(segs, &points);
        for (const Polygon& poly : polygons) {
            for (const auto& b : poly.boundaries()) {
                points.insert(b.polygon.vertices());
            }
        }

        if (points.empty()) return;

        Snap2D<T> snap(threshold);
        snap.Reset(points);
        arrangement->set_threshold(threshold);
        snap.GetSnapPoints(&points);
        arrangement->Reset(points);

        Array<Point> polyline;
        int id = 0;
        for (const Polygon& poly : polygons) {
            for (const auto& b : poly.boundaries()) {
                for (int j = 0; j < b.polygon.size(); ++j) {
                    if (b.is_outer != b.polygon.IsClockwise()) {
                        snap.FindSnapSegment(b.polygon.vertex(j),
                                             b.polygon.next_vertex(j),
                                             &polyline);
                    } else {
                        snap.FindSnapSegment(b.polygon.next_vertex(j),
                                             b.polygon.vertex(j),
                                             &polyline);
                    }
                    if (polyline.size() < 2) continue;

                    for (int k = 0; k + 1 < polyline.size(); ++k) {
                        arrangement->InsertWithoutNewVertex(polyline[k],
                                                            polyline[k + 1],
                                                            1 << id);
                    }
                }
            }
            ++id;
        }
    }

    /**
     * Sub process. It unions the subset of polygons with snapping.
     */
    void SubProcess(const Array<Polygon>& polygons, double threshold,
                    Polygon* result) const {
        if (polygons.size() == 1) {
            *result = polygons[0];
            return;
        }

        Arrangement2D<T> arrangement;
        Arrangement(polygons, threshold, &arrangement);

        const DCEL& list = arrangement.mesh();
        if (list.empty()) return;

        // Step 1. Initialize colors of each halfedge of arrangement regions.
        arrangement.Arrange();
        const auto& regions = arrangement.regions();
        auto color = list.AddHalfedgeProperty(0);
        for (Halfedge* e : list) {
            color[e] = arrangement.color(e);
        }

        // Step 2. Propagate color between regions.
        Array<int> region_colors(regions.size(), 0);
        for (int i = 0; i < regions.size(); ++i) {
            const Region& region = regions[i];
            for (const TriFace* face : region) {
                Halfedge* e = face->halfedge;
                region_colors[i] |= (color[e] | color[e->next()] |
                                     color[e->prev()]);
            }
        }

        std::queue<int> q;
        for (int i = 0; i < regions.size(); ++i) {
            q.push(i);
        }
        Array<bool> in_queue(regions.size(), true);
        while (!q.empty()) {
            int cur = q.front();
            q.pop();
            in_queue[cur] = false;
            const Region& region = regions[cur];
            int c = region_colors[cur];

            for (TriFace* face : region) {
                Halfedge* edges[3];
                edges[0] = face->halfedge;
                edges[1] = edges[0]->next();
                edges[2] = edges[0]->prev();
                for (Halfedge* e : edges) {
                    if ((color[e] & c) != color[e]) continue;

                    int r1 = arrangement.region_id(e->twin());
                    if (r1 == -1) continue;

                    int new_color = (region_colors[r1] | (c - color[e]));
                    if (region_colors[r1] != new_color) {
                        region_colors[r1] = new_color;
                        if (!in_queue[r1]) {
                          q.push(r1);
                          in_queue[r1] = true;
                        }
                    }
                }
            }
        }

        // Step 3. Remove useless halfedges.
        for (Halfedge* e : list) {
            if (arrangement.is_constraint(e)) {
                int r1 = arrangement.region_id(e);
                if (r1 != -1) color[e] = region_colors[r1];
                int r2 = arrangement.region_id(e->twin());
                if (r2 != -1) color[e->twin()] = region_colors[r2];
                if (color[e] != 0 && color[e->twin()] != 0)
                    arrangement.UnsetConstraintEdge(e);
            }
        }

        // Step 4. Get result.
        arrangement.Arrange();
        Polygon poly;
        for (int i = 0; i < arrangement.regions().size(); ++i) {
            // Filter inner region.
            if (IsInner(arrangement, arrangement.region(i), color)) continue;

            arrangement.GetRegion(i, &poly);
            for (const auto& b : poly) {
                result->Insert(b.polygon, b.is_outer);
            }
        }
    }
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_BOOLEAN_POLYGON_UNION_2D_H_
