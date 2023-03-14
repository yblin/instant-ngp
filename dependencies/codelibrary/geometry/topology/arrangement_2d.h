//
// Copyright 2012-2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_ARRANGEMENT_2D_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_ARRANGEMENT_2D_H_

#include <unordered_map>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/mesh/delaunay_2d.h"
#include "codelibrary/geometry/segment_2d.h"
#include "codelibrary/geometry/topology/halfedge_graph.h"

namespace cl {
namespace geometry {

/**
 * Arrangement with snap rounding.
 *
 * An arrangement of line segments is a decomposition of the plane into
 * vertices, edges, and faces, where
 * 1. vertices are intersection points of line segments or endpoint of line
 *    segments;
 * 2. edges are maximal connected (open) portions of line segments that do not
 *    contain any vertices;
 * 3. faces are bounded by the edges that do not contain any vertices or cross
 *    with the edge.
 *
 * Here, we use HalfedgeList to store the topological structure of the planar
 * subdivision. The input of the arrangement is floating-point line segments and
 * the output is snapped to floating-point too. It also ensures that no distance
 * between two vertices greater than the given threshold.
 */
template <typename T>
class Arrangement2D {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using Point    = Point2D<T>;
    using Segment  = Segment2D<T>;
    using DCEL     = HalfedgeList<Point>;
    using Vertex   = typename DCEL::Vertex;
    using Halfedge = typename DCEL::Halfedge;
    using Mesh     = DCEL;

private:
    /**
     * Edge with color.
     */
    struct Edge {
        Vertex* source;
        Vertex* target;
        int color1 = 0;
        int color2 = 0;

        Edge() = default;

        Edge(Vertex* s, Vertex* t, int c1, int c2)
            : source(s), target(t), color1(c1), color2(c2) {}
    };

    /**
     * Triangle face of the triangulation.
     */
    struct BaseFace {
        // Incident halfedge of the face.
        Halfedge* halfedge;

        // The ID of region own this face.
        int region_id;
    };

    // Face list.
    using FaceList = IndexedList<BaseFace>;

public:
    // Triangle face of triangulation.
    using TriFace = typename FaceList::Node;

    // Region of line arrangement.
    using Region = Array<TriFace*>;

    /**
     * Construct an empty arrangement with snapping threshold.
     */
    explicit Arrangement2D(double threshold = 0.0)
        : threshold_(threshold) {
        CHECK(threshold_ >= 0.0);
    }

    Arrangement2D(const Arrangement2D&) = delete;

    Arrangement2D& operator= (const Arrangement2D&) = delete;

    /**
     * Reset arrangement to the delauany triangulation of the given vertices.
     *
     * Here, we do not snap the input points for speed issue. Users need to
     * provide the snapped point by themselves. Call Insert(), if you don't want
     * to compute the snapped vertices by yourself.
     */
    void Reset(const Array<Point>& points) {
        this->clear();
        triangulation_.Reset(points);
        incident_points_.Reset(points);
        modified_after_arrange_ = true;
    }

    /**
     * Insert an incident point and return the inserted vertex.
     */
    Vertex* Insert(const Point& p) {
        return InsertVertex(p);
    }

    /**
     * Insert (s, t) with color.
     */
    void Insert(const Point& s, const Point& t, int color1 = 0,
                int color2 = 0) {
        if (s == t) {
            Insert(s);
            return;
        }

        // Compute intersection.
        Segment seg(s, t);
        for (auto l : lines_) {
            Point p;
            if (Cross(l, seg, &p)) InsertVertex(p);
        }

        Vertex* v1 = InsertVertex(s);
        Vertex* v2 = InsertVertex(t);

        lines_.push_back(seg);
        InsertImplement(v1, v2, color1, color2);
    }

    /**
     * Insert a edge (s, t) with color. The user should ensure that the edge
     * does not cross with other edges.
     *
     * The time complexity is O(logN).
     */
    void InsertWithoutCross(const Point& s, const Point& t, int color1 = 0,
                            int color2 = 0) {
        if (s == t) {
            Insert(s);
            return;
        }

        Vertex* v1 = InsertVertex(s);
        Vertex* v2 = InsertVertex(t);

        lines_.emplace_back(s, t);
        bool inserted = triangulation_.InsertEdge(v1, v2, color1, color2);
        CHECK(inserted);
        modified_after_arrange_ = true;
    }

    /**
     * Insert an edge (s, t) without generating a new vertex.
     *
     * If the inserted edge cross with an existing edge, the intersection point
     * will be snapped to the nearest existing vertex.
     */
    void InsertWithoutNewVertex(const Point& s, const Point& t,
                                int color1 = 0, int color2 = 0) {
        if (s == t) return;

        Vertex* v1 = FindNearestVertex(s);
        Vertex* v2 = FindNearestVertex(t);
        if (v1 && v2) {
            lines_.emplace_back(v1->point(), v2->point());
            InsertImplement(v1, v2, color1, color2);
        }
    }

    /**
     * Split the given halfedge by point 'p'.
     */
    Vertex* Split(Halfedge* e, const Point& p) {
        Vertex* v = InsertVertex(p);
        triangulation_.UnsetConstraintEdge(e);

        int c1 = triangulation_.color(e);
        int c2 = triangulation_.color(e->twin());
        this->InsertImplement(e->source(), v, c1, c2);
        this->InsertImplement(v, e->target(), c1, c2);
        return v;
    }

    /**
     * Compute the halfedge list of the arrangement result.
     *
     * Only constraint edges are preserved.
     */
    void ToHalfedgeList(DCEL* halfedge_list) const {
        CHECK(halfedge_list);

        halfedge_list->ClearHalfedgeProperties();
        halfedge_list->ClearHalfedgeProperties();
        triangulation_.mesh().Clone(halfedge_list);

        // Copy the color property.
        auto color = halfedge_list->AddHalfedgeProperty("color", 0);
        for (auto e : *halfedge_list) {
            color[e] = triangulation_.color(
                        triangulation_.mesh().halfedge(e->id()));
        }

        // Delete all non-constraint edges.
        Array<Halfedge*> delete_edges;
        for (Halfedge* e : *halfedge_list) {
            if (e->id() % 2 == 0 && !triangulation_.is_constraint(e)) {
                delete_edges.push_back(e);
            }
        }
        for (Halfedge* e : delete_edges) {
            halfedge_list->EraseEdge(e);
        }
    }

    /**
     * Remove constraint edges whose both side halfedge constrain the same
     * region.
     */
    void RemoveCoincideEdges() {
        CHECK(!modified_after_arrange_) << "Arrange() should be call first.";

        for (Halfedge* e : triangulation_.mesh()) {
            if (!triangulation_.is_constraint(e)) continue;

            TriFace* f1 = face_[e];
            TriFace* f2 = face_[e->twin()];
            if ((!f1 && !f2) || 
                (f1 && f2 && f1->region_id == f2->region_id)) {
                triangulation_.UnsetConstraintEdge(e);
            }
        }

        // Need re-arrangement().
        Arrange();
    }

    /**
     * Unset a constraint edge (but not remove the original lines).
     */
    void UnsetConstraintEdge(Halfedge* e) {
        triangulation_.UnsetConstraintEdge(e);
    }

    /**
     * Set edge 'e' to constraint edge.
     */
    void SetConstraintEdge(Halfedge* e) {
        triangulation_.SetConstraintEdge(e);
    }

    /**
     * Remove an vertex 'v'. It also removes all constraint edges around 'v'.
     */
    void RemoveVertex(Vertex* v) {
        triangulation_.Erase(v);
    }

    /**
     * Check if edge(s, t) is cross with an existing constraint edge.
     */
    bool IsCross(Vertex* s, Vertex* t) const {
        CHECK(s && t);

        return triangulation_.IsCross(s, t);
    }

    /**
     * Get partition regions enclosed by line segments.
     *
     * Each region consist of serval connected inner triangles. Each triangle
     * is defined by one of its halfedge.
     */
    void Arrange() {
        modified_after_arrange_ = false;
        const Mesh& mesh = triangulation_.mesh();

        // Set all faces of triangulation.
        face_ = mesh.AddHalfedgeProperty((TriFace*)nullptr);
        faces_.clear();
        for (Halfedge* e : mesh) {
            if (!triangulation_.is_outer(e) && !face_[e]) {
                TriFace* f = faces_.Allocate();
                f->halfedge = e;
                f->region_id = -1;
                for (Halfedge* e1 : mesh.circular_list(e)) {
                    face_[e1] = f;
                }
            }
        }

        regions_.clear();
        auto is_visited = faces_.AddProperty(false);
        for (TriFace* f : faces_) {
            CHECK(!triangulation_.is_outer(f->halfedge));

            if (is_visited[f]) continue;

            Region region;
            int front = 0;
            region.push_back(f);
            is_visited[f] = true;

            bool is_inner = true;
            while (front < region.size()) {
                TriFace* cur = region[front++];

                Halfedge* edges[3];
                edges[0] = cur->halfedge;
                edges[1] = edges[0]->next();
                edges[2] = edges[0]->prev();

                for (Halfedge* e : edges) {
                    TriFace* f1 = face_[e->twin()];

                    if (!triangulation_.is_constraint(e) && !f1) {
                        is_inner = false;
                    }

                    if (f1 && !is_visited[f1] &&
                        !triangulation_.is_constraint(e)) {
                        is_visited[f1] = true;
                        region.push_back(f1);
                    }
                }
            }

            if (!is_inner) continue;

            for (TriFace* f : region) {
                f->region_id = regions_.size();
            }
            regions_.push_back(region);
        }
    }

    /**
     * Get the region that contains the given point.
     *
     * Make sure 'Arrange()' is called before.
     *
     * It returns -1 if no region is found.
     */
    int Locate(const Point& p) const {
        CHECK(!modified_after_arrange_) << "Arrange() should be call first.";

        Halfedge* e = triangulation_.Locate(p);
        if (!e) return -1;
        TriFace* f = face_[e];
        return f ? f->region_id : -1;
    }

    /**
     * Find the vertex of the given point.
     */
    Vertex* Find(const Point& p) const {
        return triangulation_.Find(p);
    }

    /**
     * Find the nearest vertex.
     */
    Vertex* FindNearestVertex(const Point& p) const {
        Vertex* v = incident_points_.ApproximateNearestVertex(p);
        return v ? triangulation_.Find(v->point()) : nullptr;
    }

    /**
     * Update the arrangement.
     *
     * It deletes the isolate vertices, updates the constraint lines to the
     * constraint edges, and redo the triangulation.
     *
     * Note that, this function may greatly increase the number of constraint
     * lines. Use it with care unless you know what it does.
     */
    void Updade() {
        Array<std::pair<Point, Point>> lines;
        Array<std::pair<int, int>> colors;

        for (auto e : triangulation_.mesh()) {
            if (triangulation_.is_constraint(e)) {
                lines.emplace_back(e->source_point(), e->target_point());
                colors.emplace_back(triangulation_.color(e),
                                    triangulation_.color(e->twin()));
            }
        }

        this->clear();
        for (auto p : lines) {
            this->Insert(p.first);
            this->Insert(p.second);
            lines_.emplace_back(p.first, p.second);
        }
        for (int i = 0; i < lines.size(); ++i) {
            triangulation_.InsertEdge(lines[i].first, lines[i].second,
                                      colors[i].first, colors[i].second);
        }
    }

    /**
     * Get the i-th region and convert it to polygon with holes.
     *
     * Note that coincide edges will be removed.
     *
     * The first component of polygon_with_holes is outer contour and then are
     * inner contours.
     */
    void GetRegion(int i, MultiPolygon2D<T>* polygon_with_holes) const {
        CHECK(i >= 0 && i < regions_.size());
        CHECK(polygon_with_holes);
        CHECK(!modified_after_arrange_) << "Arrange() should be call first.";

        polygon_with_holes->clear();

        Region region = regions_[i];
        HalfedgeGraph<Vertex, Halfedge> graph;
        for (auto f : region) {
            Halfedge* e = f->halfedge;
            if (this->is_constraint(e)) graph.Insert(e);
            if (this->is_constraint(e->next())) graph.Insert(e->next());
            if (this->is_constraint(e->prev())) graph.Insert(e->prev());
        }

        Array<Array<Edge*>> edge_lists;
        GetEulerCycles(graph, &edge_lists);

        Array<Polygon2D<T>> inner_polygons;
        for (const auto& edge_list : edge_lists) {
            Array<Point> cycle;
            for (auto e : edge_list) {
                cycle.push_back(e->source_point());
            }
            Polygon2D<T> poly(cycle);

            poly.Trim();
            if (!poly.empty()) {
                // Note the cycle orientation is reversed.
                if (poly.IsClockwise()) {
                    inner_polygons.push_back(poly);
                } else {
                    CHECK(polygon_with_holes->empty());
                    polygon_with_holes->Insert(poly, true);
                }
            }
        }
        for (const Polygon2D<T>& poly : inner_polygons) {
            polygon_with_holes->Insert(poly, false);
        }
    }

    /**
     * Get the i-th region in the form of edge list.
     */
    void GetRegion(int i, Array<Array<Halfedge*>>* edge_lists) const {
        CHECK(i >= 0 && i < regions_.size());
        CHECK(edge_lists);
        CHECK(!modified_after_arrange_) << "Arrange() should be call first.";

        const Region& region = regions_[i];
        HalfedgeGraph<Vertex, Halfedge> graph;
        for (auto f : region) {
            Halfedge* e = f->halfedge;
            if (this->is_constraint(e)) graph.Insert(e);
            if (this->is_constraint(e->next())) graph.Insert(e->next());
            if (this->is_constraint(e->prev())) graph.Insert(e->prev());
        }
        GetEulerCycles(graph, edge_lists);
    }

    /**
     * Get all regions. See GetRegion(i, polygon_with_holes).
     */
    void GetRegions(Array<MultiPolygon2D<T>>* regions) const {
        CHECK(regions);
        CHECK(!modified_after_arrange_) << "Arrange() should be call first.";

        regions->resize(regions_.size());
        for (int i = 0; i < regions_.size(); ++i) {
            GetRegion(i, &(*regions)[i]);
        }
    }

    /**
     * Clear the arrangement.
     */
    void clear() {
        triangulation_.clear();
        lines_.clear();
        incident_points_.clear();
        faces_.clear();
        regions_.clear();
    }

    /**
     * Return the constrained delaunay triangulation of line segments.
     */
    const Delaunay2D<T>& triangulation() const {
        return triangulation_;
    }

    /**
     * Return triangulation mesh.
     */
    const Mesh& mesh() const {
        return triangulation_.mesh();
    }

    /**
     * Return the original set of lines.
     */
    const Array<Segment>& lines() const {
        return lines_;
    }

    /**
     * Return the face of the halfedge in triangulation.
     *
     * Call Arrange() before use this.
     */
    TriFace* face(const Halfedge* e) const {
        return face_[e];
    }

    /**
     * Return the region ID of the halfedge. If no valid region, return -1.
     */
    int region_id(const Halfedge* e) const {
        return face_[e] ? face_[e]->region_id : -1;
    }

    /**
     * Return faces of triangulation. It is a superset of lines partition.
     *
     * Call Arrange() before use this.
     */
    const Array<TriFace*>& faces() const {
        return faces_.nodes();
    }

    /**
     * Return the color of the given halfedge.
     */
    int color(const Halfedge* e) const {
        return triangulation_.color(e);
    }

    /**
     * Set the color of given halfedge.
     */
    void set_color(Halfedge* e, int color) {
        triangulation_.set_color(e, color);
    }

    /**
     * Check if a halfedge is the constraint edge.
     */
    bool is_constraint(const Halfedge* e) const {
        return triangulation_.is_constraint(e);
    }

    /**
     * Set threshold for snap rounding.
     *
     * The value can be dynamic changed when insertion.
     */
    void set_threshold(double threshold) {
        CHECK(threshold >= 0.0);

        threshold_ = threshold;
    }

    /**
     * Partition regions of line arrangement.
     *
     * Call Arrangement() before use this.
     */
    const Array<Region>& regions() const {
        return regions_;
    }

    /**
     * Return i-th region.
     */
    const Region& region(int i) const {
        CHECK(i >= 0 && i < regions_.size());

        return regions_[i];
    }

    bool empty() const {
        return lines_.empty();
    }

protected:
    /**
     * Insert an incident point.
     */
    Vertex* InsertVertex(const Point& p) {
        modified_after_arrange_ = true;

        Vertex* v = FindNearestVertex(p);
        if (!v || Distance(v->point(), p) > threshold_) {
            v = triangulation_.Insert(p);
            incident_points_.Insert(p);
        }

        return v;
    }

    /**
     * Insert an edge(s, t) into Delaunay triangulation with split.
     *
     * All intersection is round to the (snapped) vertices.
     */
    void InsertImplement(Vertex* s, Vertex* t, int color1, int color2) {
        if (s == t) return;
        modified_after_arrange_ = true;

        std::queue<Edge> q;
        q.emplace(s, t, color1, color2);
        while (!q.empty()) {
            Edge p = q.front();
            q.pop();

            if (p.source == p.target) continue;

            Segment seg(p.source->point(), p.target->point());
            Halfedge* e = triangulation_.FindIntersectedHalfedge(p.source,
                                                                 p.target);
            do {
                if (Intersect(seg, e->target_point())) {
                    triangulation_.InsertEdge(p.source, e->target(), p.color1,
                                              p.color2);
                    q.emplace(e->target(), p.target, p.color1, p.color2);
                    break;
                }

                Point intersection;
                Segment seg1(e->source_point(), e->target_point());
                if (Cross(seg, seg1, &intersection)) {
                    if (triangulation_.is_constraint(e)) {
                        Vertex* v = FindNearestVertex(intersection);
                        CHECK(v);

                        triangulation_.UnsetConstraintEdge(e);
                        q.emplace(p.source, v, p.color1, p.color2);
                        q.emplace(v, p.target, p.color1, p.color2);

                        int c1 = triangulation_.color(e);
                        int c2 = triangulation_.color(e->twin());
                        q.emplace(e->source(), v, c1, c2);
                        q.emplace(v, e->target(), c1, c2);
                        break;
                    }
                    e = e->twin()->next();
                } else {
                    e = e->next();
                }
            } while (true);
        }
    }

    // Constrained Delaunay triangulation.
    Delaunay2D<T> triangulation_;

    // Original line segments.
    Array<Segment2D<T>> lines_;

    // Face property of each halfedge in triangulation.
    typename Mesh::template HalfedgeProperty<TriFace*> face_;

    // Triangle faces of triangulation.
    FaceList faces_;

    // Partition regions of line arrangement.
    Array<Region> regions_;

    // Delaunay triangulation to store the incident points that consist of
    // endpoints and intersection of lines.
    Delaunay2D<T> incident_points_;

    // Threshold for snap rounding.
    double threshold_;

    // Whether the topology has changed after calling Arrange().
    bool modified_after_arrange_ = true;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_ARRANGEMENT_2D_H_
