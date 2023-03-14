//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_DAG_SHORTEST_PATHS_H_
#define CODELIBRARY_GRAPH_DAG_SHORTEST_PATHS_H_

#include <queue>

#include "codelibrary/graph/graph.h"
#include "codelibrary/graph/topological_sort.h"

namespace cl {
namespace graph {

/**
 * Shortest paths for directed acyclic graph (DAG).
 * 
 * This algorithm solves the single-source shortest-paths problem on a weighted,
 * directed acyclic graph (DAG). This algorithm is more efficient for DAG's than
 * either the Dijkstra or Bellman-Ford algorithm.
 *
 * The time complexity is O(E + V).
 *
 * Parameters:
 *  graph        - a directed acyclic graph.
 *  weight       - the weight for graph's edges.
 *  source       - the source vertex.
 *  predecessors - predecessor[i] records the predecessor of the i-th vertex
 *                 in the input graph, or -1 if the predecessor does not exist.
 *  distances    - distances[i] records the shortest distance from source to
 *                 the i-th vertex.
 */
template <typename T>
void DagShortestPaths(const Graph& graph,
                      const Graph::EdgeProperty<T>& weight,
                      int source,
                      Array<int>* predecessors,
                      Array<T>* distances) {
    CHECK(predecessors && distances);

    int n = graph.n_vertices();
    CHECK(0 <= source && source < n);

    predecessors->assign(n, -1);
    distances->assign(n, 0);

    using Edge = Graph::Edge;

    Array<int> seq;
    if (!TopologicalSort(graph, &seq)) {
        CHECK(false && "The input graph is not a DAG.");
    }

    for (int cur : seq) {
        for (const Edge* e : graph.edges_from(cur)) {
            if (e->target() == source) continue;

            if ((*predecessors)[e->target()] == -1 ||
                (*distances)[e->target()] > (*distances)[cur] + weight[e]) {
                (*distances)[e->target()] = (*distances)[cur] + weight[e];
                (*predecessors)[e->target()] = cur;
            }
        }
    }
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_DAG_SHORTEST_PATHS_H_
