//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
#define CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_

#include "codelibrary/geometry/box_3d.h"
#include "codelibrary/geometry/triangle_3d.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/util/list/circular_list_view.h"
#include "codelibrary/util/list/indexed_list.h"

namespace cl {
namespace geometry {

/**
 * SurfaceMesh differs from HalfedgeList<Point3D> in that SurfaceMesh can hold
 * any general 3D polygonal mesh, but HalfedgeList supports up to two faces per
 * edge.
 */
template <typename Point>
class SurfaceMesh {
    using T = typename Point::value_type;
    static_assert(std::is_floating_point<T>::value, "");

public:
    class BaseVertex;
    class BaseHalfedge;
    class BaseFace;

    using VertexList   = IndexedList<BaseVertex>;
    using HalfedgeList = IndexedList<BaseHalfedge>;
    using FaceList     = IndexedList<BaseFace>;

    using Vertex   = typename VertexList::Node;
    using Halfedge = typename HalfedgeList::Node;
    using Face     = typename FaceList::Node;

    /// Base Vertex of SurfaceMesh.
    class BaseVertex {
        friend class SurfaceMesh;

    public:
        BaseVertex() = default;

        /**
         * Return true if this vertex has no incident halfedges.
         */
        bool is_isolated() const {
            return halfedges_.empty();
        }

        /**
         * Return the position of this vertex.
         */
        const Point& point() const {
            return point_;
        }

        /**
         * Return the incident halfedges of this vertex.
         */
        const Array<Halfedge*>& halfedges() const {
            return halfedges_;
        }

    private:
        // The position of this vertex.
        Point point_;

        // Incident halfedges (which point outward from the vertex).
        Array<Halfedge*> halfedges_;
    };

    /// Base Halfedge of SurfaceMesh.
    class BaseHalfedge {
        friend class SurfaceMesh;

    public:
        BaseHalfedge() = default;

        Vertex* source()            const { return source_;         }
        Vertex* target()            const { return target_;         }
        Face* face()                const { return face_;           }
        Halfedge* next()            const { return next_;           }
        Halfedge* prev()            const { return prev_;           }
        const Point& source_point() const { return source_->point_; }
        const Point& target_point() const { return target_->point_; }

    private:
        // The pointer to the source vertex.
        Vertex* source_ = nullptr;

        // The pointer to the target vertex.
        Vertex* target_ = nullptr;

        // The pointer to the incident face.
        Face* face_ = nullptr;

        // The pointer to next halfedge.
        Halfedge* next_ = nullptr;

        // The pointer to previous halfedge.
        Halfedge* prev_ = nullptr;
    };

    /// Base face of SurfaceMesh.
    class BaseFace {
        friend class SurfaceMesh;

    public:
        BaseFace() = default;

        Halfedge* halfedge() const { return halfedge_; }

        /**
         * Return triangle of this face.
         *
         * TODO: Currently only triangular faces are considered.
         */
        Triangle3D<T> GetTriangle() {
            CHECK(halfedge_ && halfedge_->next());

            return Triangle3D<T>(halfedge_->source_point(),
                                 halfedge_->target_point(),
                                 halfedge_->next()->target_point());
        }

    private:
        // The pointer to the incident halfedge.
        Halfedge* halfedge_ = nullptr;
    };

    // Iterators.
    using VertexIterator        = typename VertexList::Iterator;
    using VertexConstIterator   = typename VertexList::ConstIterator;
    using HalfedgeIterator      = typename HalfedgeList::Iterator;
    using HalfedgeConstIterator = typename HalfedgeList::ConstIterator;
    using FaceIterator          = typename FaceList::Iterator;
    using FaceConstIterator     = typename FaceList::ConstIterator;
    using Iterator              = typename Array<Halfedge*>::const_iterator;

    // Properties.
    template <class T>
    using VertexProperty   = typename VertexList::template Property<T>;
    template <class T>
    using HalfedgeProperty = typename HalfedgeList::template Property<T>;
    template <class T>
    using FaceProperty     = typename FaceList::template Property<T>;

    SurfaceMesh() = default;

    SurfaceMesh(const SurfaceMesh& mesh) = delete;

    SurfaceMesh& operator =(const SurfaceMesh& mesh) = delete;

    /**
     * Clear the data.
     */
    void clear() {
        vertices_.clear();
        halfedges_.clear();
        faces_.clear();
    }

    /**
     * Check if the surface mesh is empty.
     */
    bool empty() const {
        return n_vertices() == 0;
    }

    /**
     * Return the number of vertices.
     */
    int n_vertices() const {
        return vertices_.n_available();
    }

    /**
     * Return the number of halfedges.
     */
    int n_halfedges() const {
        return halfedges_.n_available();
    }

    /**
     * Return the number of faces.
     */
    int n_faces() const {
        return faces_.n_available();
    }

    /**
     * Return the number of allocated vertices.
     */
    int n_allocated_vertices() const {
        return vertices_.n_allocated();
    }

    /**
     * Return the number of allocated halfedges.
     */
    int n_allocated_halfedges() const {
        return halfedges_.n_allocated();
    }

    /**
     * Return the number of allocated faces.
     */
    int n_allocated_faces() const {
        return faces_.n_allocated();
    }

    /**
     * Clone this SurfaceMesh.
     *
     * Note that we do NOT clone the properties but just resize them.
     */
    void Clone(SurfaceMesh* mesh) const {
        CHECK(mesh);

        if (this == mesh) return;

        mesh->clear();
        vertices_.Clone(&mesh->vertices_);
        halfedges_.Clone(&mesh->halfedges_);
        faces_.Clone(&mesh->faces_);

        auto v1 = vertices_.begin();
        auto v2 = mesh->vertices_.begin();
        for (; v1 != vertices_.end(); ++v1, ++v2) {
            auto e1 = v1->halfedges_.begin();
            auto e2 = v2->halfedges_.begin();
            for (; e1 != v1->halfedges_.end(); ++e1, ++e2) {
                *e2 = mesh->halfedges_[e1->id()];
            }
        }

        auto e1 = halfedges_.begin();
        auto e2 = mesh->halfedges_.begin();
        for (; e1 != halfedges_.end(); ++e1, ++e2) {
            e2->vertex_ = mesh->vertices_[e1->vertex_->id()];
            if (e2->next_)
                e2->next_ = mesh->halfedges_[e1->next_->id()];
            if (e2->prev_)
                e2->prev_ = mesh->halfedges_[e1->prev_->id()];
            if (e2->face_)
                e2->face_ = mesh->faces_[e1->face_->id()];
        }

        auto f1 = faces_.begin();
        auto f2 = mesh->faces_.begin();
        for (; f1 != faces_.end(); ++f1, ++f2) {
            if (f1->halfedge_) {
                f2->halfedge_ = mesh->halfedges_[v1->halfedge_->id()];
            }
        }
    }

    /**
     * Add a new isolated vertex.
     *
     * Return the pointer to the new vertex.
     */
    Vertex* AddVertex(const Point& p) {
        Vertex* v = vertices_.Allocate();
        v->point_ = p;
        return v;
    }

    /**
     * Add a new face.
     */
    void AddFace(const Array<Vertex*>& vertices) {
        CHECK(vertices.size() >= 3);

        Face* face = faces_.Allocate();
        Array<Halfedge*> halfedges(vertices.size());
        for (int i = 0; i < vertices.size(); ++i) {
            Halfedge* e = halfedges_.Allocate();
            e->source_ = vertices[i];
            e->target_ = (i + 1 == vertices.size()) ? vertices[0]
                                                    : vertices[i + 1];
            e->face_ = face;
            vertices[i]->halfedges_.push_back(e);
            halfedges[i] = e;
        }
        for (int i = 0; i < halfedges.size(); ++i) {
            halfedges[i]->next_ = i + 1 == vertices.size() ? halfedges[0]
                                                           : halfedges[i + 1];
            halfedges[i]->prev_ = i - 1 >= 0 ? halfedges[i - 1]
                                             : halfedges.back();
        }

        face->halfedge_ = halfedges.front();
    }

    // Access functions.
    const Array<Vertex*>& vertices()    const { return vertices_.nodes();  }
    const Array<Halfedge*>& halfedges() const { return halfedges_.nodes(); }
    const Array<Face*>& faces()         const { return faces_.nodes();     }

    Vertex* vertex(int id)                 { return vertices_[id];  }
    const Vertex* vertex(int id)     const { return vertices_[id];  }
    Halfedge* halfedge(int id)             { return halfedges_[id]; }
    const Halfedge* halfedge(int id) const { return halfedges_[id]; }
    Face* face(int id)                     { return faces_[id];     }
    const Face* face(int id)         const { return faces_[id];     }

    /**
     * Add a vertex property.
     */
    template <typename T>
    VertexProperty<T> AddVertexProperty(const std::string& name,
                                        const T& initial_value = T()) {
        return vertices_.AddProperty(name, initial_value);
    }

    /**
     * Add a const vertex property.
     */
    template <typename T>
    VertexProperty<T> AddVertexProperty(const T& initial_value = T()) const {
        return vertices_.AddProperty(initial_value);
    }

    /**
     * Get a vertex property.
     */
    template <typename T>
    VertexProperty<T> GetVertexProperty(const std::string& name) const {
        return vertices_.template GetProperty<T>(name);
    }

    /**
     * Add a halfedge property.
     */
    template <typename T>
    HalfedgeProperty<T> AddHalfedgeProperty(const std::string& name,
                                            const T& initial_value = T()) {
        return halfedges_.AddProperty(name, initial_value);
    }

    /**
     * Add a const halfedge property.
     */
    template <typename T>
    HalfedgeProperty<T> AddHalfedgeProperty(const T& initial_v = T()) const {
        return halfedges_.AddProperty(initial_v);
    }

    /**
     * Get a halfedge property.
     */
    template <typename T>
    HalfedgeProperty<T> GetHalfedgeProperty(const std::string& name) const {
        return halfedges_.template GetProperty<T>(name);
    }

    /**
     * Add a face property.
     */
    template <typename T>
    FaceProperty<T> AddFaceProperty(const std::string& name,
                                    const T& initial_value = T()) {
        return faces_.AddProperty(name, initial_value);
    }

    /**
     * Add a const face property.
     */
    template <typename T>
    FaceProperty<T> AddFaceProperty(const T& initial_v = T()) const {
        return faces_.AddProperty(initial_v);
    }

    /**
     * Get a face property.
     */
    template <typename T>
    FaceProperty<T> GetFaceProperty(const std::string& name) const {
        return faces_.template GetProperty<T>(name);
    }

    /**
     * Erase a vertex property with the given name.
     */
    void EraseVertexProperty(const std::string& name) {
        vertices_.EraseProperty(name);
    }

    /**
     * Erase a halfedge property with the given name.
     */
    void EraseHalfedgeProperty(const std::string& name) {
        halfedges_.EraseProperty(name);
    }

    /**
     * Erase a face property with the given name.
     */
    void EraseFaceProperty(const std::string& name) {
        faces_.EraseProperty(name);
    }

    /**
     * Clear all vertex properties.
     */
    void ClearVertexProperties() {
        vertices_.ClearAllProperties();
    }

    /**
     * Clear all halfedge properties.
     */
    void ClearHalfedgeProperties() {
        halfedges_.ClearAllProperties();
    }

    /**
     * Clear all face properties.
     */
    void ClearFaceProperties() {
        faces_.ClearAllProperties();
    }

    /**
     * Return true if the given vertex is available.
     */
    bool IsAvailable(Vertex* v) const {
        return vertices_.IsAvailable(v);
    }

    /**
     * Return true if the given halfedge is available.
     */
    bool IsAvailable(Halfedge* e) const {
        return halfedges_.IsAvailable(e);
    }

    /**
     * Return true if the given halfedge is available.
     */
    bool IsAvailable(Face* e) const {
        return faces_.IsAvailable(e);
    }

    /**
     * Get the circular list start from 'e'.
     *
     * Circular list is used to traverse the halfedges starting from 'e'
     * according to 'next()'.
     */
    CircularListView<Halfedge> circular_list(Halfedge* e) const {
        return CircularListView<Halfedge>(e);
    }

    /**
     * Get normal vector of a face.
     */
    Vector3D<T> GetFaceNormal(const Face* face) const {
        CHECK(face);

        Halfedge* e1 = face->halfedge_;
        Halfedge* e2 = e1->prev();
        return Normalize(CrossProduct(e2->target_point() - e2->source_point(),
                                      e1->target_point() - e1->source_point()));
    }

    /**
     * Compute a normal vertex map for each vertex.
     */
    VertexProperty<Vector3D<T>> GetVertexNormals() const {
        auto normals = vertices_.AddProperty(Vector3D<T>(0, 0, 0));
        for (auto f : faces_) {
            for (auto e : circular_list(f->halfedge())) {
                normals[e->source()] += GetFaceNormal(f);
            }
        }
        return normals;
    }

    /**
     * Return the bounding box of the mesh.
     */
    Box3D<T> GetBoundingBox() const {
        if (vertices_.empty()) return Box3D<T>();

        T x_min = std::numeric_limits<T>::max();
        T x_max = std::numeric_limits<T>::lowest();
        T y_min = std::numeric_limits<T>::max();
        T y_max = std::numeric_limits<T>::lowest();
        T z_min = std::numeric_limits<T>::max();
        T z_max = std::numeric_limits<T>::lowest();
        for (auto v : vertices_) {
            x_min = std::min(x_min, v->point().x);
            x_max = std::max(x_max, v->point().x);
            y_min = std::min(y_min, v->point().y);
            y_max = std::max(y_max, v->point().y);
            z_min = std::min(z_min, v->point().z);
            z_max = std::max(z_max, v->point().z);
        }
        return Box3D<T>(x_min, x_max, y_min, y_max, z_min, z_max);
    }

protected:
    VertexList vertices_;
    HalfedgeList halfedges_;
    FaceList faces_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
