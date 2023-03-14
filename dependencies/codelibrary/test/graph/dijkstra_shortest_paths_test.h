//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GRAPH_DIJKSTRA_SHORTEST_PATHS_TEST_H_
#define CODELIBRARY_TEST_GRAPH_DIJKSTRA_SHORTEST_PATHS_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/graph/dijkstra_shortest_paths.h"

namespace cl {
namespace test {

TEST(DijkstraShortestPathsTest, Test) {
    using Edge = Graph::Edge;

    Graph graph(5);
    Edge* e1 = graph.InsertTwoWayEdge(0, 1);
    Edge* e2 = graph.InsertTwoWayEdge(0, 2);
    Edge* e3 = graph.InsertTwoWayEdge(1, 2);
    Edge* e4 = graph.InsertTwoWayEdge(2, 3);
    Edge* e5 = graph.InsertTwoWayEdge(1, 3);

    Graph::EdgeProperty<int> weight = graph.AddEdgeProperty<int>();
    weight[e1] = weight[e1->twin()] = 3;
    weight[e2] = weight[e2->twin()] = 1;
    weight[e3] = weight[e3->twin()] = 1;
    weight[e4] = weight[e4->twin()] = 2;
    weight[e5] = weight[e5->twin()] = 1;

    Array<int> predecessors;
    Array<int> distances;
    graph::DijkstraShortestPaths(graph, weight, 0, &predecessors, &distances);

    ASSERT_EQ(distances[0], 0);
    ASSERT_EQ(distances[1], 2);
    ASSERT_EQ(distances[2], 1);
    ASSERT_EQ(distances[3], 3);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GRAPH_DIJKSTRA_SHORTEST_PATHS_TEST_H_
