//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
#define CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_

#include <unordered_map>

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
    class BaseEdge;
    class BaseFace;

    using VertexList = IndexedList<BaseVertex>;
    using EdgeList   = IndexedList<BaseEdge>;
    using FaceList   = IndexedList<BaseFace>;

    using Vertex = typename VertexList::Node;
    using Edge   = typename EdgeList::Node;
    using Face   = typename FaceList::Node;

    /// Base Vertex of SurfaceMesh.
    class BaseVertex {
        friend class SurfaceMesh;

    public:
        BaseVertex() = default;

        /**
         * Return true if this vertex has no incident edges.
         */
        bool is_isolated() const {
            return edges_.empty();
        }

        /**
         * Return the position of this vertex.
         */
        const Point& point() const {
            return point_;
        }

        /**
         * Return the incident edges of this vertex.
         */
        const Array<Edge*>& edges() const {
            return edges_;
        }

    private:
        // The position of this vertex.
        Point point_;

        // Incident edges (which point outward from the vertex).
        Array<Edge*> edges_;
    };

    /// Base edge of SurfaceMesh.
    class BaseEdge {
        friend class SurfaceMesh;

    public:
        BaseEdge() = default;

        Vertex* source()            const { return source_;         }
        Vertex* target()            const { return target_;         }
        Face* face()                const { return face_;           }
        Edge* next()                const { return next_;           }
        Edge* prev()                const { return prev_;           }
        const Point& source_point() const { return source_->point_; }
        const Point& target_point() const { return target_->point_; }

    private:
        // The pointer to the source vertex.
        Vertex* source_ = nullptr;

        // The pointer to the target vertex.
        Vertex* target_ = nullptr;

        // The pointer to the incident face.
        Face* face_ = nullptr;

        // The pointer to next edge.
        Edge* next_ = nullptr;

        // The pointer to previous edge.
        Edge* prev_ = nullptr;
    };

    /// Base face of SurfaceMesh.
    class BaseFace {
        friend class SurfaceMesh;

    public:
        BaseFace() = default;

        Edge* edge() const { return edge_; }

        /**
         * Return triangle of this face.
         *
         * TODO: Currently only triangular faces are considered.
         */
        Triangle3D<T> GetTriangle() {
            CHECK(edge_ && edge_->next());

            return Triangle3D<T>(edge_->source_point(),
                                 edge_->target_point(),
                                 edge_->next()->target_point());
        }

    private:
        // The pointer to the incident edge.
        Edge* edge_ = nullptr;
    };

    // Iterators.
    using VertexIterator      = typename VertexList::Iterator;
    using VertexConstIterator = typename VertexList::ConstIterator;
    using EdgeIterator        = typename EdgeList::Iterator;
    using EdgeConstIterator   = typename EdgeList::ConstIterator;
    using FaceIterator        = typename FaceList::Iterator;
    using FaceConstIterator   = typename FaceList::ConstIterator;
    using Iterator            = typename Array<Edge*>::const_iterator;

    // Properties.
    template <class T>
    using VertexProperty = typename VertexList::template Property<T>;
    template <class T>
    using EdgeProperty   = typename EdgeList::template Property<T>;
    template <class T>
    using FaceProperty   = typename FaceList::template Property<T>;

    SurfaceMesh() = default;

    SurfaceMesh(const SurfaceMesh& mesh) = delete;

    SurfaceMesh& operator =(const SurfaceMesh& mesh) = delete;

    /**
     * Clear the data.
     */
    void clear() {
        vertices_.clear();
        edges_.clear();
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
     * Return the number of edges.
     */
    int n_edges() const {
        return edges_.n_available();
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
     * Return the number of allocated edges.
     */
    int n_allocated_edges() const {
        return edges_.n_allocated();
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
        edges_.Clone(&mesh->edges_);
        faces_.Clone(&mesh->faces_);

        auto v1 = vertices_.begin();
        auto v2 = mesh->vertices_.begin();
        for (; v1 != vertices_.end(); ++v1, ++v2) {
            auto e1 = v1->edges_.begin();
            auto e2 = v2->edges_.begin();
            for (; e1 != v1->edges_.end(); ++e1, ++e2) {
                *e2 = mesh->edges_[e1->id()];
            }
        }

        auto e1 = edges_.begin();
        auto e2 = mesh->edges_.begin();
        for (; e1 != edges_.end(); ++e1, ++e2) {
            e2->vertex_ = mesh->vertices_[e1->vertex_->id()];
            if (e2->next_)
                e2->next_ = mesh->edges_[e1->next_->id()];
            if (e2->prev_)
                e2->prev_ = mesh->edges_[e1->prev_->id()];
            if (e2->face_)
                e2->face_ = mesh->faces_[e1->face_->id()];
        }

        auto f1 = faces_.begin();
        auto f2 = mesh->faces_.begin();
        for (; f1 != faces_.end(); ++f1, ++f2) {
            if (f1->edge_) {
                f2->edge_ = mesh->edges_[v1->edge_->id()];
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
        Array<Edge*> edges(vertices.size());
        for (int i = 0; i < vertices.size(); ++i) {
            Edge* e = edges_.Allocate();
            e->source_ = vertices[i];
            e->target_ = (i + 1 == vertices.size()) ? vertices[0]
                                                    : vertices[i + 1];
            e->face_ = face;
            vertices[i]->edges_.push_back(e);
            edges[i] = e;
        }
        for (int i = 0; i < edges.size(); ++i) {
            edges[i]->next_ = i + 1 == vertices.size() ? edges[0]
                                                       : edges[i + 1];
            edges[i]->prev_ = i - 1 >= 0 ? edges[i - 1]
                                         : edges.back();
        }

        face->edge_ = edges.front();
    }

    // Access functions.
    const Array<Vertex*>& vertices() const { return vertices_.nodes(); }
    const Array<Edge*>& edges()      const { return edges_.nodes();    }
    const Array<Face*>& faces()      const { return faces_.nodes();    }

    Vertex* vertex(int id)             { return vertices_[id];  }
    const Vertex* vertex(int id) const { return vertices_[id];  }
    Edge* edge(int id)                 { return edges_[id];     }
    const Edge* edge(int id)     const { return edges_[id];     }
    Face* face(int id)                 { return faces_[id];     }
    const Face* face(int id)     const { return faces_[id];     }

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
     * Add a edge property.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const std::string& name,
                                    const T& initial_value = T()) {
        return edges_.AddProperty(name, initial_value);
    }

    /**
     * Add a const edge property.
     */
    template <typename T>
    EdgeProperty<T> AddEdgeProperty(const T& initial_v = T()) const {
        return edges_.AddProperty(initial_v);
    }

    /**
     * Get a edge property.
     */
    template <typename T>
    EdgeProperty<T> GetEdgeProperty(const std::string& name) const {
        return edges_.template GetProperty<T>(name);
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
     * Erase a edge property with the given name.
     */
    void EraseEdgeProperty(const std::string& name) {
        edges_.EraseProperty(name);
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
     * Clear all edge properties.
     */
    void ClearEdgeProperties() {
        edges_.ClearAllProperties();
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
     * Return true if the given edge is available.
     */
    bool IsAvailable(Edge* e) const {
        return edges_.IsAvailable(e);
    }

    /**
     * Return true if the given edge is available.
     */
    bool IsAvailable(Face* e) const {
        return faces_.IsAvailable(e);
    }

    /**
     * Get the circular list start from 'e'.
     *
     * Circular list is used to traverse the edges starting from 'e' according
     * to 'next()'.
     */
    CircularListView<Edge> circular_list(Edge* e) const {
        return CircularListView<Edge>(e);
    }

    /**
     * Get normal vector of a face.
     */
    Vector3D<T> GetFaceNormal(const Face* face) const {
        CHECK(face);

        Edge* e1 = face->edge_;
        Edge* e2 = e1->prev();
        return Normalize(CrossProduct(e2->target_point() - e2->source_point(),
                                      e1->target_point() - e1->source_point()));
    }

    /**
     * Compute a normal vertex map for each vertex.
     */
    VertexProperty<Vector3D<T>> GetVertexNormals() const {
        auto normals = vertices_.AddProperty(Vector3D<T>(0, 0, 0));
        for (auto f : faces_) {
            for (auto e : circular_list(f->edge())) {
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

    /**
     * Convert SurfaceMesh to compressed mesh for easy transfer and rendering.
     *
     * The compressed mesh consists of a set of vertices and a set of face
     * indices.
     */
    void ToCompressMesh(Array<Point>* vertices,
                        Array<Array<int>>* faces) const {
        CHECK(vertices);
        CHECK(faces);

        vertices->clear();
        faces->clear();

        int id = 0;
        std::unordered_map<Point, int> hash;
        for (auto v : vertices_) {
            auto pair = hash.insert({v->point(), id});
            if (pair.second) {
                ++id;
                vertices->push_back(v->point());
            }
        }

        Array<int> indices;
        for (auto f : faces_) {
            indices.clear();
            for (auto e : this->circular_list(f->edge())) {
                indices.push_back(hash.find(e->source_point())->second);
            }
            faces->push_back(indices);
        }
    }

protected:
    VertexList vertices_;
    EdgeList edges_;
    FaceList faces_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_SURFACE_MESH_H_
