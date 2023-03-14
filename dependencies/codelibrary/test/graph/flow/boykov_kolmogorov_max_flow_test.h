//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_TEST_H_
#define CODELIBRARY_TEST_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/graph/flow/boykov_kolmogorov_max_flow.h"

namespace cl {
namespace test {

TEST(BoykovKolmogorovMaxFlow, Test) {
    using Edge = Graph::Edge;
    graph::BoykovKolmogorovMaxFlow<int> max_flow;

    Graph graph(5);
    Edge* e1 = graph.InsertTwoWayEdge(1, 2);
    Edge* e2 = graph.InsertTwoWayEdge(1, 4);
    Edge* e3 = graph.InsertTwoWayEdge(2, 4);
    Edge* e4 = graph.InsertTwoWayEdge(2, 3);
    Edge* e5 = graph.InsertTwoWayEdge(3, 4);

    Graph::EdgeProperty<int> capacity = graph.AddEdgeProperty<int>("capacity");
    capacity[e1] = 40;
    capacity[e2] = 20;
    capacity[e3] = 20;
    capacity[e4] = 30;
    capacity[e5] = 10;

    Graph::EdgeProperty<int> flow = graph.AddEdgeProperty<int>("flow");
    ASSERT_EQ(max_flow(graph, capacity, 1, 4, &flow), 50);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_TEST_H_
