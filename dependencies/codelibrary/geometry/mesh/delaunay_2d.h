//
// Copyright 2011-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_DELAUNAY_2D_H_
#define CODELIBRARY_GEOMETRY_MESH_DELAUNAY_2D_H_

#include <cfloat>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "codelibrary/base/array.h"
#include "codelibrary/base/equal.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/geometry/intersect_2d.h"
#include "codelibrary/geometry/mesh/halfedge_list.h"
#include "codelibrary/geometry/predicate_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace geometry {

/**
 * 2D Delaunay triangulation.
 *
 * Delaunay triangulation of points, P, in a plane is a triangulation DT(P) such
 * that no point in P is inside the circumcircle of any triangle in DT(P).
 *
 * Delaunay triangulations maximize the minimum angle of all the angles of the
 * triangles in the triangulation; they tend to avoid skinny triangles.
 *
 * This class supports the following operations:
 *  1) Divide and conquer algorithm for static Delaunay traingulation.
 *  2) Insertion and flip algorithm for dynamic Delunay (slower than 1).
 *  3) Search the approximate nearest vertex of the given query point.
 *  4) Constrained Delaunay triangulation of line segments.
 *
 * The triangulation result is stored in the HalfedgeList.
 */
template <typename T>
class Delaunay2D {
public:
    using Point    = Point2D<T>;
    using Segment  = Segment2D<T>;
    using Mesh     = HalfedgeList<Point>;
    using Vertex   = typename Mesh::Vertex;
    using Halfedge = typename Mesh::Halfedge;
    using EdgePropertyBool = typename Mesh::template HalfedgeProperty<bool>;
    using EdgePropertyInt  = typename Mesh::template HalfedgeProperty<int>;

private:
    /**
     * Used to divide points in the horizontal and vertical directions.
     */
    struct CompareXY {
        bool operator() (Vertex* a, Vertex* b) const {
            return a->point().x == b->point().x ? a->point().y < b->point().y
                                                : a->point().x < b->point().x;
        }
    };
    struct CompareYX {
        bool operator() (Vertex* a, Vertex* b) const {
            return a->point().y == b->point().y ? a->point().x < b->point().x
                                                : a->point().y < b->point().y;
        }
    };

public:
    Delaunay2D() {
        Initialize();
    }

    /**
     * Construct Delaunay Triangulation from the points in [first, last).
     */
    Delaunay2D(const Array<Point>& points) {
        Initialize();
        Reset(points);
    }

    Delaunay2D(Delaunay2D&) = delete;

    Delaunay2D& operator=(const Delaunay2D&) = delete;

    /**
     * Reset Delaunay tiangulation.
     */
    void Reset(const Array<Point>& points) {
        clear();
        if (points.empty()) return;

        for (const Point& p : points)
            AddVertex(p);

        if (mesh_.n_vertices() < 2) return;

        // Divide the points, using the strategy similar to the KD tree.
        vertices_ = mesh_.vertices();
        Halfedge* e = Divide(0, mesh_.n_vertices());

        // Set the outer edges.
        for (auto e1 : mesh_.circular_list(e)) {
            is_outer_[e1] = true;
        }
    }

    /**
     * Insert a point and update the triangulation.
     *
     * The user can specify the starting edge to speed up the location.
     */
    Vertex* Insert(const Point& p, Halfedge* s = nullptr) {
        if (mesh_.n_vertices() < 2) {
            Vertex* v1 = mesh_.n_vertices() == 1 ? mesh_.vertex(0) : nullptr;
            Vertex* v2 = AddVertex(p);
            if (v1 && v1 != v2) {
                Halfedge* e = mesh_.AddEdge(v1, v2);
                is_outer_[e] = is_outer_[e->twin()] = true;
            }
            return v2;
        }

        Halfedge* e = Locate(p, s);

        // Vertex is already exists.
        if (e->source_point() == p) return e->source();

        // Vertex is on the edge.
        if (OnEdge(e, p)) return Split(e, p);

        // The new point is outside the current boundary.
        if (is_outer_[e]) return InsertOuterVertex(p, e);

        // The new point is inside a triangle.
        return InsertInnerVertex(p, e);
    }

    /**
     * Erase a vertex and update the triangulation.
     */
    void Erase(const Point& p) {
        auto iter = vertex_map_.find(p);
        if (iter == vertex_map_.end()) return;
        Erase(iter->second);
    }
    void Erase(Vertex* v) {
        CHECK(v);
        CHECK(vertex_map_.find(v->point()) != vertex_map_.end());

        vertex_map_.erase(v->point());
        if (mesh_.n_vertices() <= 2) {
            mesh_.EraseVertex(v);
            return;
        }

        Halfedge* out_e = nullptr;
        for (Halfedge* e : mesh_.halfedges_from(v)) {
            if (is_outer_[e]) {
              out_e = e;
              break;
            }
        }
        if (out_e) {
            if (mesh_.n_vertices() == 3) {
                is_outer_[out_e->next()->twin()] = true;
                mesh_.EraseVertex(v);
                return;
            }

            // If the vertex is on the outer edge.
            // We should maintain the outer convex hull boundaries.
            Halfedge* start_e = out_e->prev()->prev();
            Halfedge* end_e = out_e->next();
            mesh_.EraseVertex(v);

            Array<Halfedge*> stack;
            stack.push_back(start_e);
            Halfedge* prev_e = start_e;
            for (Halfedge* e = start_e->next(); e != end_e; e = e->next()) {
                while (Orientation(prev_e->source_point(),
                                   prev_e->target_point(),
                                   e->target_point()) > 0) {
                    Halfedge* e1 = mesh_.JoinEdge(e, prev_e);
                    this->AddFlipEdge(e1);
                    e = e1->twin();
                    stack.pop_back();
                    CHECK(!stack.empty());
                    prev_e = stack.back();
                }
                prev_e = e;
                stack.push_back(e);
            }
            for (Halfedge* e : stack) {
                is_outer_[e] = true;
            }

            this->FlipEdges();
            return;
        }

        // The point is inside the triangulation.
        Halfedge* e = v->halfedge()->next();
        mesh_.EraseVertex(v);
        while (e->next()->next()->next() != e) {
            if (Orientation(e->prev()->source_point(), e->source_point(),
                            e->target_point()) <= 0) {
                e = e->next();
                continue;
            }

            bool flag = false;
            for (Halfedge* e1 : mesh_.circular_list(e)) {
                if (e1 == e || e1 == e->prev() || e1 == e->prev()->prev())
                    continue;

                if (Inside(e->prev()->source_point(), e->source_point(),
                           e->target_point(), e1->target_point())) {
                    flag = true;
                    break;
                }
            }
            if (!flag) {
                Halfedge* e1 = mesh_.JoinEdge(e, e->prev());
                e = e1->twin();
                this->AddFlipEdge(e);
            }
            e = e->next();
        }
        this->FlipEdges();
    }

    /**
     * Insert a constraint edge(s, t).
     *
     * It must be ensured that the inserted edge does not cross with other
     * constraint edges. Overlap and touch are allowed. If user does not know
     * whether the inserted edge crosses with other edges, call IsCross() to
     * check.
     *
     * Return false if the inserted edge cross with other constraint edges.
     */
    bool InsertEdge(const Point& s, const Point& t, int color1 = 0,
                    int color2 = 0) {
        Vertex* v1 = Insert(s);
        Vertex* v2 = Insert(t);
        return InsertEdge(v1, v2, color1, color2);
    }
    bool InsertEdge(Vertex* v1, Vertex* v2, int color1 = 0, int color2 = 0) {
        CHECK(v1 && v2);
        if (v1 == v2) return true;

        Segment seg(v1->point(), v2->point());
        Halfedge* e = FindIntersectedHalfedge(v1, v2);
        bool first = true;
        Halfedge* first_edge = e->prev();

        while (v1 != v2) {
            if (Intersect(seg, e->target_point())) {
                if (!first) {
                    Halfedge* constraint_edge = mesh_.JoinEdge(e, first_edge);
                    ProcessConstraintHalfedge(constraint_edge);

                    is_constraint_[constraint_edge] = true;
                    is_constraint_[constraint_edge->twin()] = true;
                    color_[constraint_edge->twin()] |= color1;
                    color_[constraint_edge] |= color2;
                } else {
                    is_constraint_[e] = true;
                    is_constraint_[e->twin()] = true;
                    color_[e] |= color1;
                    color_[e->twin()] |= color2;
                }

                v1 = e->target();
                if (v1 == v2) break;

                seg = Segment(v1->point(), v2->point());
                e = FindIntersectedHalfedge(v1, v2);
                first = true;
                first_edge = e->prev();
                continue;
            }

            first = false;
            if (Cross(seg, Segment(e->source_point(), e->target_point()))) {
                if (is_constraint_[e]) {
                    LOG(WARNING) << "The inserted edge is cross with other "
                                    "constraint edge.";
                    return false;
                }

                Halfedge* tmp = e->twin()->next();
                mesh_.EraseEdge(e);
                e = tmp;
            } else {
                e = e->next();
            }
        }

        return true;
    }

    /**
     * Check if edge (s, t) is cross with an existing constraint edge.
     */
    bool IsCross(Vertex* s, Vertex* t) const {
        CHECK(s && t);
        
        if (s == t) return false;
        Segment seg(s->point(), t->point());
        
        Halfedge* e = FindIntersectedHalfedge(s, t);
        Halfedge* first_edge = e->prev();
        
        while (s != t) {
            if (Intersect(seg, e->target_point())) {
                s = e->target();
                if (s == t) break;
                 
                seg = Segment(s->point(), t->point());
                e = FindIntersectedHalfedge(s, t);
                continue;
            }

            if (Cross(seg, Segment(e->source_point(), e->target_point()))) {
                if (is_constraint_[e]) {
                    return true;
                }
                
                e = e->twin()->next();
            } else {
                e = e->next();
            }
        }
        return false;
    }

    /**
     * Remove a constraint edge and update the triangulation.
     */
    void RemoveConstraintEdge(Halfedge* e) {
        if (!is_constraint_[e]) return;

        this->UnsetConstraintEdge(e);

        AddFlipEdge(e);
        AddFlipEdge(e->next());
        AddFlipEdge(e->prev());
        AddFlipEdge(e->twin()->prev());
        AddFlipEdge(e->twin()->next());
        FlipEdges();
    }

    /**
     * Unset a constraint edge without updating the triangulation.
     *
     * It just unset 'is_constraint' flag but not flip edges.
     */
    void UnsetConstraintEdge(Halfedge* e) {
        CHECK(e);

        is_constraint_[e] = false;
        is_constraint_[e->twin()] = false;
    }

    /**
     * Set 'e' to the constraint edge.
     */
    void SetConstraintEdge(Halfedge* e) {
        CHECK(e);

        is_constraint_[e] = true;
        is_constraint_[e->twin()] = true;
    }

    /**
     * Search an approximate nearest vertex to the given point with the given
     * start vertex. If there is no constrainted edges, it returns the exact
     * nearest vertex.
     */
    Vertex* ApproximateNearestVertex(const Point& p, Vertex* start) const {
        CHECK(start);

        double dis = DBL_MAX;
        bool finish = false;
        do {
            finish = true;
            Vertex* v = start;
            for (auto e : mesh_.halfedges_from(v)) {
                double d = SquaredDistance(e->target_point(), p);
                if (d < dis && !Equal(d, dis)) {
                    dis = d;
                    v = e->target();
                    finish = false;
                    break;
                }
            }
            start = v;
        } while (!finish);

        return start;
    }

    /**
     * Search an approximate nearest vertex to the given point. If there is no
     * constrainted edges, it returns the exact nearest vertex.
     *
     * It can be used as the start vertex for location.
     *
     * The worst time complexity is O(n).
     */
    Vertex* ApproximateNearestVertex(const Point& p) const {
        if (empty()) return nullptr;

        Vertex* v = Find(p);
        if (v) return v;

        // Find a start vertex according to local principle.
        const Array<Vertex*>& vertices = mesh_.vertices();
        int latest = static_cast<int>(std::sqrt(vertices.size())) + 1;
        double dis = DBL_MAX;
        for (int i = vertices.size() - 1; i >= 0 && latest; --i, --latest) {
            Vertex* v1 = vertices[i];
            double d = SquaredDistance(v1->point(), p);
            if (d < dis) {
                dis = d;
                v = v1;
            }
        }
        CHECK(v);

        return ApproximateNearestVertex(p, v);
    }

    /**
     * Get the incident halfedge of the face that contains 'p'.
     *
     * It takes time O(n) in the worst case, but only O(sqrt(n)) on average if
     * the vertices are distributed uniformly at random.
     *
     * Users can specify the starting edge to speed up location.
     */
    Halfedge* Locate(const Point& p, Halfedge* s = nullptr) const {
        if (mesh_.n_vertices() < 2) return nullptr;

        // Check if the vertex already exists.
        auto i = vertex_map_.find(p);
        if (i != vertex_map_.end()) return i->second->halfedge();

        // Do not use HalfedgeProperty. It will allocate an array of size |V|,
        // which is too slow.
        std::unordered_set<int> hash;
        std::queue<Halfedge*> queue;

        if (!s) s = ApproximateNearestVertex(p)->halfedge();
        queue.push(s);
        hash.insert(s->id());

        while (!queue.empty()) {
            Halfedge* start = queue.front();
            queue.pop();
            Halfedge* e = start;

            do {
                if (is_outer_[e]) {
                    // Check if p is outside or on the boundary.
                    if (Orientation(e->source_point(), e->target_point(),
                                    p) >= 0)
                        return e;

                    if (is_outer_[e->twin()]) return start;
                    start = e->twin();
                    e = start->next();
                    if (hash.find(start->id()) != hash.end()) break;
                    hash.insert(start->id());
                } else {
                    int o = Orientation(e->source_point(), e->target_point(),
                                        p);

                    if (o > 0) {
                        e = e->next();
                    } else {
                        if (o == 0 && Inside(e, p)) return e;

                        start = e->twin();
                        if (hash.find(start->id()) != hash.end()) break;
                        hash.insert(start->id());

                        // We add e->next->twin into queue as the another start
                        // edge.
                        Halfedge* next = e->next()->twin();
                        if (hash.find(next->id()) == hash.end()) {
                            hash.insert(next->id());
                            queue.push(next);
                        }

                        if (is_outer_[start]) return start;
                        e = start->next();
                    }
                }
            } while (e != start);
            if (e == start) return e;
        }

        CHECK(false) << "Impossible!";
        return nullptr;
    }

    /**
     * Find the vertex of the given point in the triangulation.
     */
    Vertex* Find(const Point& p) const {
        auto iter = vertex_map_.find(p);
        return iter == vertex_map_.end() ? nullptr : iter->second;
    }

    /**
     * Find the triangle attached on 's' and intersected by edge (s, t).
     *
     * Return the intersected halfedge.
     */
    Halfedge* FindIntersectedHalfedge(Vertex* s, Vertex* t) const {
        CHECK(s && t);

        Segment2D<T> seg(s->point(), t->point());
        Halfedge* e = s->halfedge();
        Halfedge* e1 = e;
        do {
            if (Intersect(seg, e1->target_point())) {
                return e1;
            }
            Halfedge* next = e1->next();
            if (Cross(seg, Segment(next->source_point(),
                                   next->target_point()))) {
                return next;
            }
            e1 = e1->prev()->twin();
        } while (e1 != e);

        // Code should not go to here.
        CHECK(false) << "Unreachable code.";
        return nullptr;
    }

    /**
     * Clear the triangulation.
     */
    void clear() {
        mesh_.clear();
        vertex_map_.clear();
    }

    /**
     * Check if this Delaunay Triangulation is empty.
     */
    bool empty() const {
        return mesh_.n_vertices() == 0;
    }

    /**
     * Return the triangulation result, only halfedges here.
     */
    const Mesh& mesh() const {
        return mesh_;
    }

    /**
     * Return true if the given halfedge is an outside boundary edge.
     */
    bool is_outer(const Halfedge* e) const {
        return is_outer_[e];
    }

    /**
     * Return true if the edge is a constraint edge.
     */
    bool is_constraint(const Halfedge* e) const {
        return is_constraint_[e];
    }

    /**
     * Return the color of the given halfedge.
     */
    int color(const Halfedge* e) const {
        return color_[e];
    }

    /**
     * Return the color property for each halfedge of the mesh.
     */
    const EdgePropertyInt& color() const {
        return color_;
    }

    /**
     * Set the color of halfedge.
     */
    void set_color(Halfedge* e, int color) {
        color_[e] = color;
    }

protected:
    /**
     * Check if p is on the edge.
     */
    static bool OnEdge(Halfedge* e, const Point& p) {
        int o = Orientation(e->source_point(), e->target_point(), p);

        return o == 0 && Inside(e, p);
    }

    /**
     * Check if p is inside the box determined by p1 and p2.
     */
    static bool Inside(Halfedge* e, const Point& p) {
        return Inside(e->source_point(), e->target_point(), p);
    }
    static bool Inside(const Point& p1, const Point& p2, const Point& p) {
        return (p1 <= p && p <= p2) || (p2 <= p && p <= p1);
    }

    /**
     * Check if p is inside the triangle (a, b, c).
     */
    static bool Inside(const Point& a, const Point& b, const Point& c,
                       const Point& p) {
        return Orientation(p, a, b) >= 0 &&
               Orientation(p, b, c) >= 0 &&
               Orientation(p, c, a) >= 0;
    }

    /**
     * Initialization.
     */
    void Initialize() {
        is_flip_ = mesh_.AddHalfedgeProperty("is_flip", false);
        is_outer_ = mesh_.AddHalfedgeProperty("is_outer", false);
        is_constraint_ = mesh_.AddHalfedgeProperty("is_constraint", false);
        color_ = mesh_.AddHalfedgeProperty("color", 0);
    }

    /**
     * Add a new vertex.
     */
    Vertex* AddVertex(const Point& p) {
        auto i = vertex_map_.find(p);
        if (i != vertex_map_.end()) return i->second;

        Vertex* v = mesh_.AddVertex(p);
        vertex_map_[p] = v;
        return v;
    }

    /**
     * Make a new triangle or two edges with three vertices.
     */
    Halfedge* MakeTriangle(Vertex* v1, Vertex* v2, Vertex* v3) {
        int o = Orientation(v1->point(), v2->point(), v3->point());

        if (o == 0) {
            // We need to sort v1, v2, and v3.
            if (v2->point() < v1->point()) std::swap(v1, v2);
            if (v3->point() < v2->point()) {
                std::swap(v2, v3);
                if (v2->point() < v1->point()) std::swap(v1, v2);
            }

            Halfedge* e1 = mesh_.AddEdge(v1, v2);
            Halfedge* e2 = e1->twin();
            Halfedge* e3 = mesh_.AddEdge(v2, v3);
            Halfedge* e4 = e3->twin();

            mesh_.set_next(e1, e3);
            mesh_.set_next(e4, e2);
            return e1;
        } else {
            Halfedge *e1, *e2, *e3, *e4, *e5, *e6;
            e1 = mesh_.AddEdge(v1, v2);
            e2 = e1->twin();
            e3 = mesh_.AddEdge(v2, v3);
            e4 = e3->twin();
            e5 = mesh_.AddEdge(v3, v1);
            e6 = e5->twin();

            mesh_.AddTriangle(e1, e3, e5);
            mesh_.AddTriangle(e2, e6, e4);

            return o > 0 ? e2 : e1;
        }
    }

    /**
     * This method first divide the point set to left and right part,
     * and recursive get the left and right part of Delaunay triangulation;
     * and then call merge method two parts into one triangulation.
     *
     * Parameters:
     *   l - the left index of vertices.
     *   r - the right index of vertices.
     */
    Halfedge* Divide(int l, int r) {
        int n = r - l;
        CHECK(n >= 2);

        if (n == 2) {
            return mesh_.AddEdge(vertices_[l], vertices_[l + 1]);
        }

        if (n == 3) {
            return MakeTriangle(vertices_[l], vertices_[l + 1],
                                vertices_[l + 2]);
        }

        // Horizontal cut or vertical cut.
        int cut_dimension;
        int cut_index = MiddleSplit(l, r, &cut_dimension);

        Divide(l, cut_index);
        Divide(cut_index, r);

        return Merge(l, cut_index, r, cut_dimension);
    }

    /**
     * Middle split the vertices in [l, r).
     *
     * Parameters:
     *  l - the first index to split.
     *  r - the last index to split.
     *
     * Return the index in [l, r) for splitting.
     */
    int MiddleSplit(int l, int r, int* cut_dimension) {
        T max_span = 0;
        int d = -1;

        for (int i = 0; i < 2; ++i) {
            // Compute exact span on the found dimension.
            T min_elem = vertices_[l]->point()[i];
            T max_elem = vertices_[l]->point()[i];

            for (int j = l + 1; j < r; ++j) {
                T val = vertices_[j]->point()[i];
                min_elem = std::min(min_elem, val);
                max_elem = std::max(max_elem, val);
            }

            if (max_elem - min_elem >= max_span || d == -1) {
                max_span = max_elem - min_elem;
                d = i;
            }
        }
        *cut_dimension = d;

        int split = l + (r - l) / 2;
        if (*cut_dimension == 0) {
            std::nth_element(vertices_.begin() + l, vertices_.begin() + split,
                             vertices_.begin() + r, compare_xy_);
        } else {
            std::nth_element(vertices_.begin() + l, vertices_.begin() + split,
                             vertices_.begin() + r, compare_yx_);
        }

        return split;
    }

    /**
     * Determine the lower tangent of two triangulations.
     *
     * Parameters:
     *   [l, m) are the indices of the first triangulation.
     *   [m, r) are the indices of the right triangulation.
     *   cut_dimension = 0 means the cut plane is perpendicular on X axis;
     *   otherwise, the cut plane is perpendicular on Y axis.
     *
     * Return:
     *   the halfedge of the lower tangent (from right to left).
     */
    Halfedge* LowerTangent(int l, int m, int r, int cut_dimension) {
        // Find the rightmost point in [l, m) and leftmost point in [m, r).
        Vertex* vl = nullptr;
        Vertex* vr = nullptr;

        // The order of vertices may be changed, so we need to traverse to find
        // the vl and vr.
        if (cut_dimension == 0) {
            vl = *std::max_element(vertices_.begin() + l, vertices_.begin() + m,
                                   compare_xy_);
            vr = *std::min_element(vertices_.begin() + m, vertices_.begin() + r,
                                   compare_xy_);
        } else {
            vl = *std::max_element(vertices_.begin() + l, vertices_.begin() + m,
                                   compare_yx_);
            vr = *std::min_element(vertices_.begin() + m, vertices_.begin() + r,
                                   compare_yx_);
        }

        Halfedge* el = FindCWBoundary(vl);
        Halfedge* er = FindCCWBoundary(vr);

        // Zig-zag algorithm.
        while (true) {
            if (Orientation(el->source_point(), er->source_point(),
                            el->target_point()) < 0) {
                el = el->next();
            } else if (Orientation(el->source_point(), er->source_point(),
                                   er->target_point()) < 0) {
                er = er->twin()->prev()->twin();
            } else {
                break;
            }
        }
        return mesh_.JoinEdge(er->twin(), el);
    }

    /**
     * Find the boundary edge in clockwise whose source vertex is 'v'.
     *
     * It requires that 'v' is the boundary vertex.
     */
    Halfedge* FindCWBoundary(Vertex* v) const {
        Halfedge* e = v->halfedge();
        Halfedge* e1 = e->prev()->twin();
        do {
            if (Orientation(v->point(), e->target_point(),
                            e1->target_point()) <= 0) return e;
            e = e1;
            e1 = e->prev()->twin();
        } while (true);

        return nullptr;
    }

    /**
     * Find the boundary edge in counter-clockwise whose source vertex is 'v'.
     *
     * It requires that 'v' is the boundary vertex.
     */
    Halfedge* FindCCWBoundary(Vertex* v) const {
        Halfedge* e = v->halfedge();
        Halfedge* e1 = e->twin()->next();
        do {
            if (Orientation(v->point(), e->target_point(),
                            e1->target_point()) >= 0) return e;
            e = e1;
            e1 = e->twin()->next();
        } while (true);

        return nullptr;
    }

    /**
     * Merge two triangulations into one.
     *
     * Parameters:
     *   [l, m) are the indices of the first triangulation.
     *   [m, r) are the indices of the right triangulation.
     *   cut_dimension = 0 means the cut plane is perpendicular on X axis;
     *   otherwise, the cut plane is perpendicular on Y axis.
     *
     * Return:
     *   the halfedge of the lower tangent (from right to left).
     */
    Halfedge* Merge(int left, int mid, int right, int cut_dimension) {
        // Create the first cross edge by joining lower common tangent.
        Halfedge* tangent = LowerTangent(left, mid, right, cut_dimension);
        Halfedge* base = tangent->twin();
        Halfedge* l = base->prev()->twin();
        Halfedge* r = base->next();

        while (true) {
            int o1 = Orientation(base->target_point(), base->source_point(),
                                 l->target_point());
            int o2 = Orientation(base->source_point(), base->target_point(),
                                 r->target_point());
            if (o1 >= 0 && o2 <= 0) {
                // Found the upper common tangent.
                break;
            }

            if (o1 < 0) {
                Halfedge* t = l->prev()->twin();
                while (Orientation(base->source_point(), base->target_point(),
                                   t->target_point()) > 0 &&
                       InCircle(base->source_point(), base->target_point(),
                                l->target_point(), t->target_point()) > 0) {
                    mesh_.EraseEdge(l);
                    l = t;
                    t = l->prev()->twin();
                }
            }

            if (o2 > 0) {
                Halfedge* t = r->twin()->next();
                while (Orientation(base->source_point(), base->target_point(),
                                   t->target_point()) > 0 &&
                       InCircle(base->source_point(), base->target_point(),
                                r->target_point(), t->target_point()) > 0) {
                    mesh_.EraseEdge(r);
                    r = t;
                    t = r->twin()->next();
                }
            }

            if (o1 >= 0) {
                r = r->next();
                base = mesh_.JoinEdge(l->twin(), r);
                continue;
            }

            if (o2 <= 0) {
                l = l->twin()->prev()->twin();
                base = mesh_.JoinEdge(l->twin(), r);
                continue;
            }

            if (InCircle(base->source_point(), base->target_point(),
                         l->target_point(), r->target_point()) < 0) {
                l = l->twin()->prev()->twin();
            } else {
                r = r->next();
            }

            base = mesh_.JoinEdge(l->twin(), r);
        }

        return tangent;
    }

    /**
     * Split halfedge 'e' at point 'p'.
     *
     * Return the split vertex.
     */
    Vertex* Split(Halfedge* e, const Point& p) {
        if (p == e->source_point()) return e->source();
        if (p == e->target_point()) return e->target();

        Halfedge* e_prev = e->prev();
        Halfedge* e_twin_prev = e->twin()->prev();

        Vertex* v = AddVertex(p);
        bool is_constraint = is_constraint_[e];
        int c1 = color_[e];
        int c2 = color_[e->twin()];
        bool is_outer1 = is_outer_[e];
        bool is_outer2 = is_outer_[e->twin()];

        Halfedge* e1 = mesh_.SplitEdge(e, v);
        Halfedge* e2 = e1->next()->twin();
        if (is_constraint) {
            is_constraint_[e1] = true;
            is_constraint_[e1->next()] = true;
            is_constraint_[e2] = true;
            is_constraint_[e2->next()] = true;
            color_[e1] = c1;
            color_[e1->next()] = c1;
            color_[e2] = c2;
            color_[e2->next()] = c2;
        }

        if (is_outer1) is_outer_[e1] = is_outer_[e1->next()] = true;
        if (is_outer2) is_outer_[e2] = is_outer_[e2->next()] = true;

        if (!is_outer1) {
            Halfedge* e = mesh_.JoinEdge(e1, e_prev);
            AddFlipEdge(e->twin()->prev());
            AddFlipEdge(e->next());
        }

        if (!is_outer2) {
            Halfedge* e = mesh_.JoinEdge(e2, e_twin_prev);
            AddFlipEdge(e->twin()->prev());
            AddFlipEdge(e->next());
        }
        FlipEdges();

        return v;
    }

    /**
     * Find the immediately previous boundary edge, 'e', such that,
     * Orient(e, p) <= 0.
     */
    Halfedge* FindPreviousBoundaryEdge(Halfedge* e, const Point& p) const {
        Halfedge* e1 = e;
        do {
            Halfedge* e2 = e1->next();
            int o = Orientation(e1->source_point(), e1->target_point(), p);
            if (o == 0) {
                if (Inside(e1, p)) return e1;

                int o1 = Orientation(e2->source_point(), e2->target_point(), p);
                if (o1 > 0) return e1;

                if (o1 == 0 &&
                    Inside(e1->source_point(), p, e1->target_point()) &&
                    Inside(p, e2->target_point(), e2->source_point())) {
                    return e1;
                }
            } else if (o < 0) {
                int o1 = Orientation(e2->source_point(), e2->target_point(), p);
                if (o1 > 0) return e1;
            }

            e1 = e1->prev();
        } while (e1 != e);

        CHECK(false);
        return nullptr;
    }

    /**
     * Add the flip edge into the array.
     */
    void AddFlipEdge(Halfedge* e) {
        if (!is_outer_[e] && !is_outer_[e->twin()] && !is_constraint_[e]) {
            is_flip_[e] = true;
            is_flip_[e->twin()] = true;
            flip_edges_.push_back(e);
        }
    }

    /**
     * Flip an internal edge of the triangulation.
     */
    void Flip(Halfedge* e) {
        CHECK(!is_outer_[e] && !is_outer_[e->twin()]);

        Halfedge* prev = e->prev();
        Halfedge* next = e->twin()->next();

        mesh_.EraseEdge(e);
        mesh_.JoinEdge(next, prev);
    }

    /**
     * Flip all edges to make a Delaunay triangulation.
     */
    void FlipEdges() {
        if (flip_edges_.empty()) return;

        int front = 0;
        while (front < flip_edges_.size()) {
            Halfedge* e = flip_edges_[front++];
            is_flip_[e] = false;
            is_flip_[e->twin()] = false;
            if (is_outer_[e] || is_outer_[e->twin()]) continue;
            if (is_constraint_[e]) continue;

            Point p = e->twin()->next()->target_point();
            if (InCircle(e->source_point(), e->target_point(),
                         e->prev()->source_point(), p) > 0) {
                AddFlipEdge(e->twin()->next());
                AddFlipEdge(e->twin()->prev());
                AddFlipEdge(e->next());
                AddFlipEdge(e->prev());
                Flip(e);
            }
        }

        flip_edges_.clear();
    }

    /**
     * Insert a vertex that lies outside the triangulation.
     *
     * The parameter 'e' is the incident edge of 'p'.
     */
    Vertex* InsertOuterVertex(const Point& p, Halfedge* e) {
        Halfedge* first_e = FindPreviousBoundaryEdge(e, p);
        if (OnEdge(first_e, p)) return Split(first_e, p);

        Vertex* v = AddVertex(p);
        Halfedge* last_e = first_e->next();
        do {
            if (Orientation(last_e->source_point(), last_e->target_point(),
                            p) <= 0)
                break;
            is_outer_[last_e] = false;
            AddFlipEdge(last_e);
            last_e = last_e->next();
        } while (last_e != first_e->next());

        Array<Halfedge*> edges;
        Halfedge* e2 = first_e;
        do {
            Halfedge* e3 = mesh_.AddEdge(e2->target(), v);
            Halfedge* e4 = e2->next();
            edges.push_back(e3);
            mesh_.set_next(e3->twin(), e4);
            mesh_.set_next(e2, e3);
            e2 = e4;
        } while (e2 != last_e);

        for (int i = 0; i < edges.size(); ++i) {
            Halfedge* prev_e = i - 1 >= 0 ? edges[i - 1] : edges.back();
            Halfedge* e = edges[i];
            mesh_.set_next(e, prev_e->twin());
        }

        is_outer_[first_e->next()] = true;
        is_outer_[last_e->prev()] = true;

        FlipEdges();
        return v;
    }

    /**
     * Insert a vertex that lies inside the triangulation.
     *
     * The parameter 'e' is the edge of the face containing 'p'.
     */
    Vertex* InsertInnerVertex(const Point& p, Halfedge* e) {
        Vertex* v = AddVertex(p);

        Array<Halfedge*> edges;
        for (Halfedge* e1 : mesh_.circular_list(e)) {
            edges.push_back(mesh_.AddEdge(e1->target(), v));
            AddFlipEdge(e1);
        }

        for (int i = 0; i < edges.size(); ++i) {
            int j = i == 0 ? edges.size() - 1 : i - 1;
            Halfedge* e1 = edges[i];
            Halfedge* e2 = edges[j]->twin();
            Halfedge* tmp = e->next();
            mesh_.set_next(e1, e2);
            mesh_.set_next(e2, e);
            mesh_.set_next(e, e1);
            e = tmp;
        }

        FlipEdges();

        return v;
    }

    /**
     * Modify the triangulation by the given constraint Halfedge.
     */
    void ProcessConstraintHalfedge(Halfedge* e) {
        std::queue<Halfedge*> q;
        q.push(e);
        q.push(e->twin());
        while (!q.empty()) {
            Halfedge* cur = q.front();
            q.pop();

            // Store the halfedges between cur->source() to cur->target();
            Array<Halfedge*> halfedges;
            Halfedge* tmp = cur->next();
            while (tmp != cur) {
                halfedges.push_back(tmp);
                tmp = tmp->next();
            }

            if (halfedges.size() <= 2) continue;

            int best = 1;
            for (int i = 2; i < halfedges.size(); ++i) {
                if (InCircle(cur->source_point(),
                             cur->target_point(),
                             halfedges[best]->source_point(),
                             halfedges[i]->source_point()) > 0) {
                    best = i;
                }
            }
            if (best == 1) {
                Halfedge* tmp = mesh_.JoinEdge(halfedges[best]->prev(), cur);
                q.push(tmp->twin());
            } else if (best == halfedges.size() - 1) {
                Halfedge* tmp = mesh_.JoinEdge(cur, halfedges[best]);
                q.push(tmp->twin());
            } else {
                Halfedge* tmp1 = mesh_.JoinEdge(halfedges[best]->prev(), cur);
                Halfedge* tmp2 = mesh_.JoinEdge(cur, tmp1);
                q.push(tmp1->twin());
                q.push(tmp2->twin());
            }
        }
    }

    // The halfedge list mesh to store the triangulation results.
    Mesh mesh_;

    // Map from point to vertex.
    std::unordered_map<Point, Vertex*> vertex_map_;

    // Temporary used.
    Array<Vertex*> vertices_;

    // Used to divide points in the horizontal and vertical directions.
    CompareXY compare_xy_;
    CompareYX compare_yx_;

    // The edges that need to be flip.
    Array<Halfedge*> flip_edges_;

    // Used to mark the edges needed to be flipped.
    // This is an internal temporary variable.
    EdgePropertyBool is_flip_;

    // Check if the edge is the outer edge.
    EdgePropertyBool is_outer_;

    // Check if the halfedge is an inserted constraint halfedge.
    EdgePropertyBool is_constraint_;

    // Color property is useful for many applications (such as polygon boolean).
    EdgePropertyInt color_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_DELAUNAY_2D_H_
