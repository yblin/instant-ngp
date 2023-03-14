//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GRAPH_GRAPH_TEST_H_
#define CODELIBRARY_TEST_GRAPH_GRAPH_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/graph/graph.h"

namespace cl {
namespace test {

TEST(GraphTest, Resize) {
    Graph graph;
    ASSERT_EQ(graph.n_vertices(), 0);

    graph.Resize(5);
    ASSERT_EQ(graph.n_vertices(), 5);
}

TEST(GraphTest, Insert) {
    Graph graph(5);

    Graph::Edge* e0 = graph.InsertOneWayEdge(1, 2);
    ASSERT_EQ(e0->source(), 1);
    ASSERT_EQ(e0->target(), 2);
    Graph::Edge* t1 = *graph.edges_from(1).begin();
    ASSERT_EQ(e0, t1);

    Graph::Edge* e1 = graph.InsertOneWayEdge(1, 3);
    ASSERT_EQ(e1->source(), 1);
    ASSERT_EQ(e1->target(), 3);
    ASSERT_EQ(graph.edges_from(1).size(), 2);

    Graph::Edge* e2 = graph.InsertTwoWayEdge(2, 4);
    ASSERT_EQ(e2->source(), 2);
    ASSERT_EQ(e2->target(), 4);

    Graph::Edge* e3 = e2->twin();
    ASSERT_EQ(e3->source(), 4);
    ASSERT_EQ(e3->target(), 2);
}

TEST(GraphTest, Find) {
    Graph graph(10);
    graph.InsertTwoWayEdge(3, 0);

    CHECK(graph.FindEdge(3, 0));
}

TEST(GraphTest, Erase) {
    Graph graph(4);
    Graph::Edge* e1 = graph.InsertOneWayEdge(1, 2);
    Graph::Edge* e2 = graph.InsertOneWayEdge(1, 3);

    graph.EraseTwoWayEdge(e2);

    Graph::Edge* e = *graph.edges_from(1).begin();
    ASSERT_EQ(e->source(), 1);
    ASSERT_EQ(e->target(), 2);

    graph.EraseOneWayEdge(e1);
    ASSERT_EQ(graph.n_edges(), 0);
}

TEST(GraphTest, Clone) {
    Graph graph1(4);
    graph1.InsertOneWayEdge(1, 2);
    Graph::Edge* e2 = graph1.InsertOneWayEdge(1, 3);
    graph1.InsertTwoWayEdge(2, 3);
    graph1.EraseTwoWayEdge(e2);

    Graph graph2;
    graph1.Clone(&graph2);

    ASSERT_EQ(graph1.n_vertices(), graph2.n_vertices());
    ASSERT_EQ(graph1.n_edges(), graph2.n_edges());

    for (int i = 0; i < graph1.n_vertices(); ++i) {
        Graph::EdgeList::Iterator i1 = graph1.edges_from(i).begin();
        Graph::EdgeList::Iterator i2 = graph2.edges_from(i).begin();
        for (; i1 != graph1.edges_from(i).end(); ++i1, ++i2) {
            ASSERT_EQ(i1->id(), i2->id());
            ASSERT_EQ(i1->source(), i2->source());
            ASSERT_EQ(i1->target(), i2->target());
        }
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GRAPH_GRAPH_TEST_H_
