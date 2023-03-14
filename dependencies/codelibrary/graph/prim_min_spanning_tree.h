//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_PRIM_MIN_SPANNING_TREE_H_
#define CODELIBRARY_GRAPH_PRIM_MIN_SPANNING_TREE_H_

#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * Prim algorithm to get minimum spanning tree.
 *
 * The time complexity of this function is O((E + V) Log(V)).
 *
 * Parameters:
 *   graph     - the bidirectional graph.
 *   weight    - the weight for graph's edges.
 *   mst_edges - the output minimum spanning tree edges.
 * 
 * Return:
 *   the sum of cost of the minimum spanning tree's edges.
 */
template <typename T>
T PrimMinSpanningTree(const Graph& graph,
                      const Graph::EdgeProperty<T>& weight,
                      Array<const Graph::Edge*>* mst_edges = nullptr) {
    CHECK(graph.IsBidirectional());

    using Edge = Graph::Edge;

    if (mst_edges) mst_edges->clear();

    int n = graph.n_vertices();

    int source = -1;
    const Edge* e_source = nullptr;
    for (int i = 0; i < n; ++i) {
        for (const Edge* e : graph.edges_from(i)) {
            CHECK(weight[e] == weight[e->twin()]) <<
                "The twin edges must have the same weight";

            source = i;
        }
    }
    if (source == -1) return 0;

    using Pair = std::pair<T, const Edge*>;
    std::priority_queue<Pair, Array<Pair>, std::greater<Pair>> q;
    q.push(Pair(0, e_source));

    Array<T> distances(n, std::numeric_limits<T>::max());
    Array<bool> is_finish(n, false);

    is_finish[source] = true;
    T sum = 0;

    while (!q.empty()) {
        Pair cur = q.top();
        q.pop();
        const Edge* e1 = cur.second;
        if (e1) {
            if (is_finish[e1->target()]) continue;
            is_finish[e1->target()] = true;
            if (mst_edges) mst_edges->push_back(e1);
            sum += weight[e1];
        }

        int target = e1 ? e1->target() : source;
        for (const Edge* e : graph.edges_from(target)) {
            if (e->target() == source || e->source() == e->target()) continue;

            const T& w = weight[e];
            if (!is_finish[e->target()] && w < distances[e->target()]) {
                distances[e->target()] = w;
                q.push(Pair(w, e));
            }
        }
    }

    return sum;
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_PRIM_MIN_SPANNING_TREE_H_
