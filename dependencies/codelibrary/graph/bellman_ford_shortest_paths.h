//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_BELLMAN_FORD_SHORTEST_PATHS_H_
#define CODELIBRARY_GRAPH_BELLMAN_FORD_SHORTEST_PATHS_H_

#include <queue>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * The Bellman Ford algorithm computes single-source shortest paths in a
 * weighted digraph.
 *
 * For graphs with only non-negative edge weights, the faster Dijkstra's
 * algorithm also solves the problem. Thus, Bellman Ford is used primarily for
 * graphs with negative edge weights.
 *
 * The time complexity is O(V * E).
 *
 * Parameters:
 *  graph        - a directed or undirected graph.
 *  weight       - the weight for graph's edges.
 *  source       - the source vertex.
 *  predecessors - predecessor[i] records the predecessor of the i-th vertex
 *                 in the input graph, or -1 if the predecessor does not exist.
 *  distances    - distances[i] records the shortest distance from source to
 *                 the i-th vertex.
 *
 * Return: false if there is a negative cost circuit in the graph.
 */
template <typename T>
bool BellmanFordShortestPaths(const Graph& graph,
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

    std::queue<int> q;
    Array<bool> in_q(n, false);
    q.push(source);
    q.push(n);
    in_q[source] = true;

    int step = 0;

    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        in_q[cur] = false;

        if (cur == n) {
            if (step++ > n) break;
            q.push(n);
            continue;
        }

        // Relax edges.
        for (const Edge* e : graph.edges_from(cur)) {
            if (e->target() == source || e->source() == e->target()) continue;

            const T& w = weight[e];
            if ((*predecessors)[e->target()] == -1 ||
                (*distances)[cur] + w < (*distances)[e->target()]) {
                (*distances)[e->target()] = (*distances)[cur] + w;
                (*predecessors)[e->target()] = cur;
                if (!in_q[e->target()]) {
                    in_q[e->target()] = true;
                    q.push(e->target());
                }
            }
        }
    }

    // Check for negative-weight cycles.
    for (int i = 0; i < n; ++i) {
        for (const Edge* e : graph.edges_from(i)) {
            if ((*predecessors)[e->target()] == -1) continue;
            if ((*distances)[e->source()] + weight[e] <
                (*distances)[e->target()]) {
                return false;
            }
        }
    }

    return true;
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_BELLMAN_FORD_SHORTEST_PATHS_H_
