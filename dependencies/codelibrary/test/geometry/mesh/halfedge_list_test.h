//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_MESH_HALFEDGE_LIST_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_MESH_HALFEDGE_LIST_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/mesh/halfedge_list.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {
namespace test {

/**
 * Halfedge list test.
 */
class HalfedgeListTest : public Test {
protected:
    using Point    = RPoint2D;
    using List     = geometry::HalfedgeList<Point>;
    using Vertex   = List::Vertex;
    using Halfedge = List::Halfedge;
};

TEST_F(HalfedgeListTest, AddVertex) {
    Point p(1.0, 2.0);
    List list;
    Vertex* v1 = list.AddVertex(p);
    ASSERT_EQ(list.n_vertices(), 1);
    Vertex* v2 = list.AddVertex(p);
    ASSERT_EQ(list.n_vertices(), 2);

    ASSERT_EQ(list.vertices()[0], v1);
    ASSERT_EQ(list.vertices()[1], v2);
}

TEST_F(HalfedgeListTest, Clone) {
    Array<Point> ps = {{1.0, 2.0}, {3.0, 2.0}, {1.0, 1.0}};

    List list;
    Vertex* v1 = list.AddVertex(ps[0]);
    Vertex* v2 = list.AddVertex(ps[1]);
    Vertex* v3 = list.AddVertex(ps[2]);

    Halfedge *e1;
    e1 = list.AddEdge(v1, v2);
    e1 = list.AddEdge(v2, v3);
    e1 = list.AddEdge(v3, v1);

    list.EraseEdge(e1);
    list.AddEdge(v1, v3);

    List list1;
    list.Clone(&list1);

    ASSERT_EQ(list1.n_vertices(), list.n_vertices());
    ASSERT_EQ(list1.n_halfedges(), list.n_halfedges());
    ASSERT_EQ(list1.n_allocated_halfedges(), list.n_allocated_halfedges());
    ASSERT_EQ(list1.n_allocated_vertices(), list.n_allocated_vertices());

    for (int i = 0; i < list.vertices().size(); ++i) {
        Vertex* v1 = list.vertices()[i];
        Vertex* v2 = list1.vertices()[i];
        ASSERT_EQ(v1->id(), v2->id());
        ASSERT_EQ(v1->halfedge() == nullptr ? -1 : v1->halfedge()->id(),
                 v2->halfedge() == nullptr ? -1 : v2->halfedge()->id());
    }

    for (int i = 0; i < list.halfedges().size(); ++i) {
        Halfedge* e1 = list.halfedges()[i];
        Halfedge* e2 = list1.halfedges()[i];
        ASSERT_EQ(e1->id(), e2->id());
        ASSERT_EQ(e1->next() == nullptr ? -1 : e1->next()->id(),
                 e2->next() == nullptr ? -1 : e2->next()->id());
        ASSERT_EQ(e1->prev() == nullptr ? -1 : e1->prev()->id(),
                 e2->prev() == nullptr ? -1 : e2->prev()->id());
        ASSERT_EQ(e1->twin() == nullptr ? -1 : e1->twin()->id(),
                 e2->twin() == nullptr ? -1 : e2->twin()->id());
        ASSERT_EQ(e1->source()->id(), e2->source()->id());
        ASSERT_EQ(e1->target()->id(), e2->target()->id());
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_MESH_HALFEDGE_LIST_TEST_H_
