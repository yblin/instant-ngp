//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GRAPH_KRUSKAL_MINIMUM_SPANNING_TREE_TEST_H_
#define CODELIBRARY_TEST_GRAPH_KRUSKAL_MINIMUM_SPANNING_TREE_TEST_H_

#include <random>

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/distance_2d.h"
#include "codelibrary/graph/kruskal_min_spanning_tree.h"

namespace cl {
namespace test {

TEST(KruskalMinimumSpanningTreeTest, Test) {
    using Edge = Graph::Edge;

    Graph graph(5);
    Edge* e1 = graph.InsertTwoWayEdge(1, 2);
    Edge* e2 = graph.InsertTwoWayEdge(1, 3);
    Edge* e3 = graph.InsertTwoWayEdge(2, 3);
    Edge* e4 = graph.InsertTwoWayEdge(4, 3);

    Graph::EdgeProperty<int> weight = graph.AddEdgeProperty<int>();
    weight[e1] = weight[e1->twin()] = 4;
    weight[e2] = weight[e2->twin()] = 9;
    weight[e3] = weight[e3->twin()] = 2;
    weight[e4] = weight[e4->twin()] = 1;

    Array<const Edge*> mst_edges;
    graph::KruskalMinSpanningTree(graph, weight, &mst_edges);

    ASSERT_EQ(mst_edges.size(), 3);
    ASSERT_EQ(mst_edges[0]->source(), 3);
    ASSERT_EQ(mst_edges[0]->target(), 4);
    ASSERT_EQ(mst_edges[1]->source(), 2);
    ASSERT_EQ(mst_edges[1]->target(), 3);
    ASSERT_EQ(mst_edges[2]->source(), 1);
    ASSERT_EQ(mst_edges[2]->target(), 2);
}

/**
 * Test the performance of KruskalMinimumSpanningTree.
 */
TEST(KruskalMinimumSpanningTreeTest, Performance) {
    const int n = 500;

    Array<RPoint2D> points(n);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::mt19937 random;

    for (int i = 0; i < n; ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }

    using Edge = Graph::Edge;

    Graph graph(n);
    Graph::EdgeProperty<double> weight =
            graph.AddEdgeProperty<double>("weight");
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double w = Distance(points[i], points[j]);
            Edge* e = graph.InsertTwoWayEdge(i, j);
            weight[e] = weight[e->twin()] = w;
        }
    }

    graph::KruskalMinSpanningTree(graph, weight);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GRAPH_KRUSKAL_MINIMUM_SPANNING_TREE_TEST_H_
