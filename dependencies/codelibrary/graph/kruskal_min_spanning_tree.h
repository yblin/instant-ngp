//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_KRUSKAL_MIN_SPANNING_TREE_H_
#define CODELIBRARY_GRAPH_KRUSKAL_MIN_SPANNING_TREE_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/graph/graph.h"
#include "codelibrary/util/set/disjoint_set.h"

namespace cl {
namespace graph {

/**
 * Kruskal algorithm to get minimum spanning tree.
 *
 * The time complexity of this function is O(E Log(E)).
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
T KruskalMinSpanningTree(const Graph& graph,
                         const Graph::EdgeProperty<T>& weight,
                         Array<const Graph::Edge*>* mst_edges = nullptr) {
    CHECK(graph.IsBidirectional());

    using Edge = Graph::Edge;

    if (mst_edges) mst_edges->clear();

    int n = graph.n_vertices();
    if (mst_edges) mst_edges->reserve(n - 1);

    Array<const Edge*> edges;
    edges.reserve(graph.n_edges());
    for (int i = 0; i < n; ++i) {
        for (const Edge* e : graph.edges_from(i)) {
            CHECK(weight[e] == weight[e->twin()]) <<
                "The twin edges must have the same weight";
            edges.push_back(e);
        }
    }

    // Find the corresponding weights for each edge.
    Array<T> cost;
    cost.reserve(edges.size());
    for (const Edge* e : edges) {
        cost.push_back(weight[e]);
    }

    // Get the sequence according to the weight.
    Array<int> seq;
    IndexSort(cost.begin(), cost.end(), &seq);
    DisjointSet disjoint_set(n);

    T sum = 0;
    // Sorting the edges from the smallest distance to the largest.
    for (int s : seq) {
        const Edge* e = edges[s];
        if (disjoint_set.Find(e->source()) != disjoint_set.Find(e->target())) {
            disjoint_set.Union(e->source(), e->target());
            if (mst_edges) mst_edges->push_back(e);
            sum += cost[s];
        }
    }

    return sum;
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_KRUSKAL_MIN_SPANNING_TREE_H_
