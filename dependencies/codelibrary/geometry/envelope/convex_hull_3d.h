//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_3D_H_
#define CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_3D_H_

#include <cfloat>
#include <queue>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/mesh/halfedge_list.h"
#include "codelibrary/geometry/predicate_2d.h"
#include "codelibrary/geometry/predicate_3d.h"

namespace cl {
namespace geometry {

/**
 * 3D convex hull.
 *
 * This class adopts quickhull algorithm of Barber et al.
 *
 * Reference:
 *   Barber C B, Dobkin D P, Huhdanpaa H. The quickhull algorithm for convex
 *   hulls [J]. ACM Transactions on Mathematical Software, 1996, 22(4): 469-483.
 */
template <typename T>
class ConvexHull3D {
    using Point = Point3D<T>;
    using Mesh = HalfedgeList<Point>;
    using Vertex = typename Mesh::Vertex;
    using Halfedge = typename Mesh::Halfedge;

    /**
     * Face of convex hull.
     */
    struct BaseFace {
        Halfedge* halfedge; // Incident halfedge of this face.
    };

    using FaceList = IndexedList<BaseFace>;

public:
    using Face = typename FaceList::Node;

    ConvexHull3D() {
        face_ = mesh_.AddHalfedgeProperty("face", (Face*)nullptr);
        outside_sets_ = faces_.AddProperty("outside_sets", Array<int>());
        farthest_point_ = faces_.AddProperty("farthest_point", -1);
    }

    explicit ConvexHull3D(const Array<Point3D<T>>& points) {
        face_ = mesh_.AddHalfedgeProperty("face", (Face*)nullptr);
        outside_sets_ = faces_.AddProperty("outside_sets", Array<int>());
        farthest_point_ = faces_.AddProperty("farthest_point", -1);

        Reset(points);
    }

    /**
     * Reset 3D convex hull with the given points.
     */
    void Reset(const Array<Point3D<T>>& points) {
        this->clear();
        points_ = points;
        Build();
    }

    void clear() {
        points_.clear();
        mesh_.clear();
        faces_.clear();
    }

    bool empty() const {
        return faces_.empty();
    }

    const Mesh& mesh() const {
        return mesh_;
    }

    const Array<Face*>& faces() const {
        return faces_.nodes();
    }

private:
    /**
     * Building convex hull.
     */
    void Build() {
        // Initialize a initial tetrahedron.
        if (!InitializeTetrahedron()) return;

        // Initialize outside sets for faces.
        Array<int> indices(points_.size());
        for (int i = 0; i < points_.size(); ++i) {
            indices[i] = i;
        }

        Array<Face*> faces(faces_.begin(), faces_.end());
        ComputeOutsideSets(faces, indices);

        while (!available_faces_.empty()) {
            Face* cur_face = available_faces_.front();
            available_faces_.pop();

            // Update available faces.
            if (!faces_.IsAvailable(cur_face) ||
                outside_sets_[cur_face].empty()) {
                continue;
            }

            // Find visible faces from point 'p'.
            int farthest = farthest_point_[cur_face];
            const Point& p = points_[farthest];
            Array<Face*> visible_faces;
            FindVisibleFaces(cur_face, p, &visible_faces);

            Array<int> outside_set;
            for (Face* face : visible_faces) {
                for (int v : outside_sets_[face]) {
                    outside_set.push_back(v);
                }
            }

            // Remove the visible faces.
            Array<Halfedge*> delete_edges;
            for (Face* face : visible_faces) {
                Halfedge* e = face->halfedge;

                EraseFace(face);

                if (!face_[e->twin()])
                    delete_edges.push_back(e);
                if (!face_[e->next()->twin()])
                    delete_edges.push_back(e->next());
                if (!face_[e->prev()->twin()])
                    delete_edges.push_back(e->prev());
            }

            // Delete useless edges.
            for (Halfedge* e : delete_edges) {
                mesh_.EraseEdge(e);
            }

            // Compute boundary halfedges.
            Halfedge* start = nullptr;
            for (Halfedge* e : mesh_) {
                if (!face_[e]) {
                    start = e;
                    break;
                }
            }
            CHECK(start);

            Array<Halfedge*> boundary;
            for (Halfedge* e : mesh_.circular_list(start)) {
                boundary.push_back(e);
            }

            // Add new faces.
            Vertex* v = mesh_.AddVertex(p);
            Array<Halfedge*> edges(boundary.size() * 2);
            for (int i = 0; i < boundary.size(); ++i) {
                Halfedge* e = mesh_.AddEdge(boundary[i]->source(), v);
                edges[i << 1] = e;
                edges[(i << 1) + 1] = e->twin();
            }

            Array<Face*> new_faces;
            for (int i = 0; i < boundary.size(); ++i) {
                int next = i + 1 < boundary.size() ? i + 1 : 0;
                Face* face = InsertTriangle(boundary[i], edges[next << 1],
                                            edges[(i << 1) + 1]);
                new_faces.push_back(face);
            }

            // Assigned points to outside sets of new faces.
            ComputeOutsideSets(new_faces, outside_set);
        }

        // Remove useless vertices.
        for (Vertex* v : mesh_.vertices()) {
            if (v->is_isolated()) {
                mesh_.EraseVertex(v);
            }
        }
    }

    /**
     * Initialize a tetrahedron.
     *
     * Return false if all points are on the same plane.
     */
    bool InitializeTetrahedron() {
        // The size of points must at least 4.
        if (points_.size() < 4) return false;

        int a = 0, b = -1, c = -1;
        for (int i = 1; i < points_.size(); ++i) {
            if (points_[a].x != points_[i].x || points_[a].y != points_[i].y) {
                b = i;
                break;
            }
        }

        // All the points are the same in X and Y coordinates.
        if (b == -1) return false;

        for (int i = 0; i < points_.size(); ++i) {
            if (i == a || i == b) continue;

            Point2D<T> p1(points_[a].x, points_[a].y);
            Point2D<T> p2(points_[b].x, points_[b].y);
            Point2D<T> p3(points_[i].x, points_[i].y);

            int o = Orientation(p1, p2, p3);
            if (o != 0) {
                c = i;
                break;
            }
        }

        // All the points are coplanar.
        if (c == -1) return false;

        // Find the farthest point away from the plane defined by a, b, c.
        double max_dis = 0.0;
        int d = -1;
        for (int i = 0; i < points_.size(); ++i) {
            if (i == a || i == b || i == c) continue;

            int o = Orientation(points_[a], points_[b], points_[c], points_[i]);
            if (o != 0) {
                double dis = SignedDistance(points_[a], points_[b], points_[c],
                                            points_[i]);
                if (std::fabs(dis) > std::fabs(max_dis)) {
                    max_dis = dis;
                    d = i;
                }
            }
        }

        // All the points are coplanar.
        if (d == -1) return false;

        Vertex* vertices[4];
        vertices[0] = mesh_.AddVertex(points_[a]);
        vertices[1] = mesh_.AddVertex(points_[b]);
        vertices[2] = mesh_.AddVertex(points_[c]);
        vertices[3] = mesh_.AddVertex(points_[d]);
        Halfedge* edges[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = i + 1; j < 4; ++j) {
                edges[i][j] = mesh_.AddEdge(vertices[i], vertices[j]);
                edges[j][i] = edges[i][j]->twin();
            }
        }
        if (max_dis > 0) {
            InsertTriangle(edges[2][1], edges[1][0], edges[0][2]);
            InsertTriangle(edges[0][1], edges[1][3], edges[3][0]);
            InsertTriangle(edges[1][2], edges[2][3], edges[3][1]);
            InsertTriangle(edges[2][0], edges[0][3], edges[3][2]);
        }  else if (max_dis < 0) {
            InsertTriangle(edges[0][1], edges[1][2], edges[2][0]);
            InsertTriangle(edges[2][1], edges[1][3], edges[3][2]);
            InsertTriangle(edges[1][0], edges[0][3], edges[3][1]);
            InsertTriangle(edges[0][2], edges[2][3], edges[3][0]);
        } else {
            // The maximum distance from point to the plane is zero.
            return false;
        }

        return true;
    }

    /**
     * Compute the outside sets for the given faces.
     */
    void ComputeOutsideSets(const Array<Face*>& faces,
                            const Array<int>& sequences) {
        Array<int> indices(sequences);

        for (Face* face : faces) {
            int farthest = -1;
            double dis_max = -DBL_MAX;
            Array<int> outside_set;
            for (int j = 0; j < indices.size(); ) {
                const Point& p = points_[indices[j]];

                int o = FaceOrientation(p, face);
                if (o > 0) {
                    double dis = SignedDistance(p, face);
                    outside_set.push_back(indices[j]);
                    if (dis > dis_max) {
                        dis_max = dis;
                        farthest = indices[j];
                    }
                    indices[j] = indices.back();
                    indices.pop_back();
                } else {
                    ++j;
                }
            }

            if (!outside_set.empty()) {
                outside_sets_[face] = outside_set;
                farthest_point_[face] = farthest;
                available_faces_.push(face);
            }
        }
    }

    /**
     * Find the visible faces from point 'p'.
     */
    void FindVisibleFaces(Face* face, const Point& p,
                          Array<Face*>* visible_faces) const {
        // BFS to found the new boundary of current convex hull.
        std::queue<Face*> queue;
        queue.push(face);
        auto is_visited = faces_.AddProperty(false);
        is_visited[face] = true;

        while (!queue.empty()) {
            Face* face = queue.front();
            queue.pop();
            visible_faces->push_back(face);

            Face* faces[3];
            Halfedge* e = face->halfedge;
            faces[0] = face_[e->twin()];
            faces[1] = face_[e->next()->twin()];
            faces[2] = face_[e->prev()->twin()];

            for (Face* face : faces) {
                if (face && !is_visited[face]) {
                    if (FaceOrientation(p, face) > 0) {
                        is_visited[face] = true;
                        queue.push(face);
                    }
                }
            }
        }
    }

    /**
     * Return the orientation from the given point to the face.
     */
    int FaceOrientation(const Point& p, Face* face) const {
        Halfedge* e = face->halfedge;
        return Orientation(e->source_point(), e->target_point(),
                           e->next()->target_point(), p);
    }

    /**
     * Return the signed distance from the given point to the face.
     */
    double SignedDistance(const Point& p, Face* face) const {
        Halfedge* e = face->halfedge;
        return SignedDistance(e->source_point(), e->target_point(),
                              e->next()->target_point(), p);
    }

    /**
     * Compute the signed distance from p4 to the plane determined by p1, p2,
     * and p3.
     */
    double SignedDistance(const Point& p1, const Point& p2,
                          const Point& p3, const Point& p4) const {
        double dis = OrientationDeterminant<double>(p1.x, p1.y, p1.z,
                                                    p2.x, p2.y, p2.z,
                                                    p3.x, p3.y, p3.z,
                                                    p4.x, p4.y, p4.z);
        return dis;
    }

    /**
     * Insert a triangle into mesh.
     */
    Face* InsertTriangle(Halfedge* e1, Halfedge* e2, Halfedge* e3) {
        mesh_.AddTriangle(e1, e2, e3);

        Face* face = faces_.Allocate();
        face->halfedge = e1;
        face_[e1] = face;
        face_[e2] = face;
        face_[e3] = face;
        return face;
    }

    void EraseFace(Face* face) {
        for (Halfedge* e1 : mesh_.circular_list(face->halfedge)) {
            face_[e1] = nullptr;
        }
        faces_.Deallocate(face);
    }

    // Input points.
    Array<Point> points_;

    // Mesh for convex hull.
    Mesh mesh_;

    // Assign a face to each halfedge.
    typename Mesh::template HalfedgeProperty<Face*> face_;

    // The points are assigned to each face as its outside set.
    typename FaceList::template Property<Array<int>> outside_sets_;

    // Farthest point away from the face.
    typename FaceList::template Property<int> farthest_point_;

    // Current available faces for building convex hull.
    std::queue<Face*> available_faces_;

    // Face list.
    FaceList faces_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_3D_H_
