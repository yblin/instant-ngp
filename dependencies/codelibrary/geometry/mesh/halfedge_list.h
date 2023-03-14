//
// Copyright 2016-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_HALFEDGE_LIST_H_
#define CODELIBRARY_GEOMETRY_MESH_HALFEDGE_LIST_H_

#include "codelibrary/util/list/circular_list_view.h"
#include "codelibrary/util/list/indexed_list.h"

namespace cl {
namespace geometry {

/**
 * Halfedge list data structure.
 *
 * A halfedge list data structure also known as the doubly connected edge list
 * (DCEL), it is an edge-centered data structure capable of maintaining
 * incidence information of vertices, edges and faces, for example for planar
 * maps, polyhedral, or other orientable, two-dimensional surfaces embedded in
 * arbitrary dimension.
 *
 * In this implementation, the face is not explicitly defined. Because the
 * creation and deletion of faces may take linear time.
 *
 * Each halfedge stores following informations.
 * 1. A pointer to the source vertex.
 * 2. A pointer to its twin edge.
 * 3. A pointer to the next edge on the boundary of the incident face.
 * 4. A pointer to the previous edge on the boundary of the incident face.
 * 5. Note that, the halfedges along the boundary of a hole are called border
 *    halfedges and have no incident face.
 */
template <typename Point>
class HalfedgeList {
    /**
     * Traverse the outgoing halfedges around 'v'.
     */
    template <class Node>
    class OutgoingEdgeList {
    public:
        class Iterator {
        public:
            explicit Iterator(Node* node)
                : head_(node), node_(node) {}

            bool operator == (const Iterator& rhs) const {
                return head_ == rhs.head_ && node_ == rhs.node_;
            }

            bool operator != (const Iterator& rhs) const {
                return !(*this == rhs);
            }

            Node* operator*()  const { return node_; }
            Node* operator->() const { return node_; }

            Iterator& operator++() {
                CHECK(node_);
                node_ = node_->twin();
                CHECK(node_);
                node_ = node_->next();
                CHECK(node_);

                if (node_ == head_) {
                    head_ = nullptr;
                    node_ = nullptr;
                }
                return *this;
            }

        protected:
            Node* head_ = nullptr;
            Node* node_ = nullptr;
        };

        explicit OutgoingEdgeList(Node* head)
            : head_(head) {}

        Iterator begin() const {
            return Iterator(head_);
        }

        Iterator end() const {
            return Iterator(nullptr);
        }

    protected:
        Node* head_ = nullptr;
    };

public:
    class BaseVertex;
    class BaseHalfedge;

    using VertexList = IndexedList<BaseVertex>;
    using EdgeList   = IndexedList<BaseHalfedge>;

    using Vertex   = typename VertexList::Node;
    using Halfedge = typename EdgeList::Node;

    // Base Vertex of HalfedgeList.
    class BaseVertex {
        friend class HalfedgeList;

    public:
        BaseVertex() = default;

        bool is_isolated()   const { return halfedge_ == nullptr; }
        const Point& point() const { return point_;               }
        Halfedge* halfedge() const { return halfedge_;            }

    private:
        Point point_;                  // The position of this vertex.
        Halfedge* halfedge_ = nullptr; // The pointer to incident halfedge.
    };

    // Base Halfedge of HalfedgeList.
    class BaseHalfedge {
        friend class HalfedgeList;

    public:
        BaseHalfedge() = default;

        Vertex* source()            const { return vertex_;                }
        Vertex* target()            const { return twin_->vertex_;         }
        Halfedge* twin()            const { return twin_;                  }
        Halfedge* next()            const { return next_;                  }
        Halfedge* prev()            const { return prev_;                  }
        const Point& source_point() const { return vertex_->point_;        }
        const Point& target_point() const { return twin_->vertex_->point_; }

    private:
        Vertex* vertex_ = nullptr; // The pointer to source vertex.
        Halfedge* twin_ = nullptr; // The pointer to twin halfedge.
        Halfedge* next_ = nullptr; // The pointer to next halfedge.
        Halfedge* prev_ = nullptr; // The pointer to previous halfedge.
    };

    // Iterators.
    using VertexIterator        = typename VertexList::Iterator;
    using VertexConstIterator   = typename VertexList::ConstIterator;
    using HalfedgeIterator      = typename EdgeList::Iterator;
    using HalfedgeConstIterator = typename EdgeList::ConstIterator;
    using Iterator              = typename Array<Halfedge*>::const_iterator;

    // Properties.
    template <class T>
    using VertexProperty   = typename VertexList::template Property<T>;
    template <class T>
    using HalfedgeProperty = typename EdgeList::template Property<T>;

    HalfedgeList() = default;

    HalfedgeList(const HalfedgeList& list) = delete;

    HalfedgeList& operator =(const HalfedgeList& list) = delete;

    /**
     * Clear the data.
     */
    void clear() {
        vertices_.clear();
        halfedges_.clear();
    }

    /**
     * Check if the halfedge list is empty.
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
     * Add a new vertex with position p.
     *
     * Return the pointer to the new vertex.
     */
    Vertex* AddVertex(const Point& p) {
        Vertex* v = CreateVertex();
        v->point_ = p;
        return v;
    }

    /**
     * Add a new pair of unattached halfedges.
     * This function only insert a pair of halfedges into HalfedgeList, but
     * don't join these halfedges to the other existed halfedges.
     *
     * Parameters:
     *  source - the pointer to the source vertex.
     *  target - the pointer to the target vertex.
     *
     * Return the halfedge from source to target.
     */
    Halfedge* AddEdge(Vertex* source, Vertex* target) {
        Halfedge* e1 = CreateEdge();
        Halfedge* e2 = e1->twin_;

        e1->vertex_ = source;
        e2->vertex_ = target;

        if (source->is_isolated()) {
            source->halfedge_ = e1;
        }
        if (target->is_isolated()) {
            target->halfedge_ = e2;
        }

        return e1;
    }

    /**
     * Find the halfedge from v1 to v2.
     */
    Halfedge* FindHalfedge(Vertex* v1, Vertex* v2) const {
        CHECK(v1 != v2);

        Halfedge* e = v1->halfedge_;
        if (e == nullptr) return nullptr;

        Halfedge* tmp = e;
        do {
            if (tmp->target() == v2) {
                return tmp;
            }
            tmp = tmp->twin_->next_;
        } while (tmp != e);

        return nullptr;
    }

    /**
     * Create a new pair of halfedges to join the a's target to b's source.
     *
     * Note that, the halfedges a and b have to belong to the same face.
     *
     * Parameters:
     *  a is the halfedge which target point will be joined.
     *  b is the halfedge which source point will be joined.
     *
     * Return the halfedge that from a's target to b's source.
     */
    Halfedge* JoinEdge(Halfedge* a, Halfedge* b) {
        CHECK(a->target() != b->source());

        Halfedge *e1, *e2;
        e1 = AddEdge(a->target(), b->source());
        e2 = e1->twin_;

        set_next(e2, a->next_);
        set_next(a, e1);
        set_next(b->prev_, e2);
        set_next(e1, b);

        return e1;
    }

    /**
     * Erase an existing pair of halfedge.
     */
    void EraseEdge(Halfedge* e) {
        CHECK(e);

        Halfedge* e1 = e->twin_;
        Halfedge* e_prev = e->prev_;
        Halfedge* e1_next = e1->next_;
        set_next(e_prev, e1_next);

        Halfedge* e1_prev = e1->prev_;
        Halfedge* e_next = e->next_;
        set_next(e1_prev, e_next);

        if (e->source()->halfedge_ == e) {
            e->source()->halfedge_ = (e1_next == e ? nullptr : e1_next);
        }
        if (e1->source()->halfedge_ == e1) {
            e1->source()->halfedge_ = (e_next == e1 ? nullptr : e_next);
        }

        halfedges_.Deallocate(e);
        halfedges_.Deallocate(e1);
    }

    /**
     * Erase an existing vertex.
     */
    void EraseVertex(Vertex* v) {
        CHECK(v);

        Array<Halfedge*> edges;
        for (Halfedge* e : halfedges_from(v)) {
            edges.push_back(e);
        }
        for (Halfedge* e : edges) {
            EraseEdge(e);
        }
        vertices_.Deallocate(v);
    }

    /**
     * Split the edge 'e' by vertex 'v', required 'v' is isolated.
     *
     * Return one of the split halfedge (from e->source to v).
     */
    Halfedge* SplitEdge(Halfedge* e, Vertex* v) {
        CHECK(v->is_isolated());

        Halfedge* e_next = e->next_;
        Halfedge* e_prev = e->prev_;
        Halfedge* e_twin = e->twin_;
        Halfedge* e_twin_prev = e_twin->prev_;
        Halfedge* e_twin_next = e_twin->next_;

        Halfedge *e1, *e2, *e3, *e4;
        e1 = AddEdge(e->source(), v);
        e2 = e1->twin_;
        e3 = AddEdge(v, e->target());
        e4 = e3->twin_;

        if (e_next == e_twin) {
            set_next(e3, e4);
        } else {
            set_next(e3, e_next);
        }

        if (e_prev == e_twin) {
            set_next(e2, e1);
        } else {
            set_next(e_prev, e1);
        }

        set_next(e1, e3);
        set_next(e4, e2);

        if (e_twin_next != e) set_next(e2, e_twin_next);
        if (e_twin_prev != e) set_next(e_twin_prev, e4);

        if (e->source()->halfedge_ == e)
            e->source()->halfedge_ = e1;
        if (e_twin->source()->halfedge_ == e_twin) {
            e_twin->source()->halfedge_ = e4;
        }

        halfedges_.Deallocate(e);
        halfedges_.Deallocate(e_twin);

        return e1;
    }

    /**
     * Clone this HalfedgeList.
     *
     * Note that we do NOT clone the properties but just resize them.
     */
    void Clone(HalfedgeList* list) const {
        CHECK(list);

        if (this == list) return;

        list->clear();
        vertices_.Clone(&list->vertices_);
        halfedges_.Clone(&list->halfedges_);

        VertexConstIterator v1 = vertices_.begin();
        VertexIterator v2 = list->vertices_.begin();
        for (; v1 != vertices_.end(); ++v1, ++v2) {
            if (v1->halfedge_) {
                v2->halfedge_ = list->halfedges_[v1->halfedge_->id()];
            }
        }

        HalfedgeConstIterator e1 = halfedges_.begin();
        HalfedgeIterator e2 = list->halfedges_.begin();
        for (; e1 != halfedges_.end(); ++e1, ++e2) {
            e2->vertex_ = list->vertices_[e1->vertex_->id()];
            e2->next_   = list->halfedges_[e1->next_->id()];
            e2->prev_   = list->halfedges_[e1->prev_->id()];
            e2->twin_   = list->halfedges_[e1->twin_->id()];
        }
    }

    /**
     * set e1->next = e2, and e2->prev = e1.
     */
    void set_next(Halfedge* e1, Halfedge* e2) {
        CHECK(e1->target() == e2->source());

        e1->next_ = e2;
        e2->prev_ = e1;
    }

    /**
     * set e1->next = e2, and e2->prev = e1.
     */
    void set_prev(Halfedge* e1, Halfedge* e2) {
        CHECK(e2->target() == e1->source());

        e1->prev_ = e2;
        e2->next_ = e1;
    }

    /**
     * Add a triangle by three halfedges (e1->e2->e3).
     */
    void AddTriangle(Halfedge* e1, Halfedge* e2, Halfedge* e3) {
        set_next(e1, e2);
        set_next(e2, e3);
        set_next(e3, e1);
    }

    // Access functions.
    const Array<Vertex*>& vertices()    const { return vertices_.nodes();  }
    const Array<Halfedge*>& halfedges() const { return halfedges_.nodes(); }

    Vertex* vertex(int id)                 { return vertices_[id];  }
    const Vertex* vertex(int id)     const { return vertices_[id];  }
    Halfedge* halfedge(int id)             { return halfedges_[id]; }
    const Halfedge* halfedge(int id) const { return halfedges_[id]; }

    Iterator begin() const { return halfedges_.nodes().begin(); }
    Iterator end()   const { return halfedges_.nodes().end();   }

    /**
     * Get all outgoing halfedges starting from 'v'.
     */
    OutgoingEdgeList<Halfedge> halfedges_from(Vertex* v) const {
        CHECK(v);

        return OutgoingEdgeList<Halfedge>(v->halfedge());
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

protected:
    /**
     * Create a new vertex, but does not change the other information in the
     * HalfedgeList.
     */
    Vertex* CreateVertex() {
        Vertex* v = vertices_.Allocate();
        v->halfedge_ = nullptr;
        return v;
    }

    /**
     * Create a new pairs of halfedges, but does not change the other
     * information in the HalfedgeList.
     */
    Halfedge* CreateEdge() {
        Halfedge* e1 = halfedges_.Allocate();
        Halfedge* e2 = halfedges_.Allocate();

        e1->twin_ = e2;
        e2->twin_ = e1;
        set_next(e1, e2);
        set_prev(e1, e2);

        return e1;
    }

    VertexList vertices_;
    EdgeList   halfedges_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_HALFEDGE_LIST_H_
