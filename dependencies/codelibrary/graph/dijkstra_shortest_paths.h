//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_DIJKSTRA_SHORTEST_PATHS_H_
#define CODELIBRARY_GRAPH_DIJKSTRA_SHORTEST_PATHS_H_

#include <functional>
#include <queue>
#include <utility>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * Dijkstra algorithm solves the single-source shortest-paths problem on a
 * weighted, directed or undirected graph for the case where all edge weights
 * are nonnegative.
 *
 * Use the Bellman-Ford algorithm for the case when some edge weights are
 * negative.
 *
 * The worst time complexity is O((E + V) * log(V)).
 *
 * Parameters:
 *  graph        - a directed or undirected graph.
 *  weight       - the weight for graph's edges.
 *  source       - the source vertex.
 *  predecessors - predecessor[i] records the predecessor of the i-th vertex
 *                 in the input graph, or -1 if the predecessor does not exist.
 *  distances    - distances[i] records the shortest distance from source to
 *                 the i-th vertex.
 */
template <typename T>
void DijkstraShortestPaths(const Graph& graph,
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
    using Pair = std::pair<T, int>;

    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> q;
    q.push(Pair(0, source));

    Array<bool> is_finish(n, false);

    while (!q.empty()) {
        Pair cur = q.top();
        q.pop();
        if (is_finish[cur.second]) continue;
        is_finish[cur.second] = true;

        for (const Edge* e : graph.edges_from(cur.second)) {
            if (e->target() == source || e->source() == e->target()) continue;

            const T& w = weight[e];
            if ((*predecessors)[e->target()] == -1 ||
                (*distances)[e->source()] + w < (*distances)[e->target()]) {
                (*distances)[e->target()] = (*distances)[e->source()] + w;
                (*predecessors)[e->target()] = e->source();
                q.push(Pair((*distances)[e->target()], e->target()));
            }
        }
    }
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_DIJKSTRA_SHORTEST_PATHS_H_
