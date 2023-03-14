//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_TOPOLOGICAL_SORT_H_
#define CODELIBRARY_GRAPH_TOPOLOGICAL_SORT_H_

#include <queue>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * A topological sort of a directed graph is a linear ordering of its vertices
 * such that for every directed edge uv from vertex u to vertex v, u comes
 * before v in the ordering.
 *
 * Note that, topological sort only valid for directed acyclic graph (DAG).
 * Therefore, this function will return false if the graph is not a DAG.
 */
inline bool TopologicalSort(const Graph& graph, Array<int>* sequence) {
    CHECK(sequence);

    sequence->clear();

    int n = graph.n_vertices();
    Array<int> indegree(n, 0);
    for (int i = 0; i < n; ++i) {
        for (const Graph::Edge* e : graph.edges_from(i)) {
            ++indegree[e->target()];
        }
    }

    std::queue<int> q;
    for (int i = 0; i < n; ++i) {
        if (indegree[i] == 0) q.push(i);
    }

    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        sequence->push_back(cur);
        for (const Graph::Edge* e : graph.edges_from(cur)) {
            if (--indegree[e->target()] == 0) q.push(e->target());
        }
    }

    return sequence->size() == n;
}

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_TOPOLOGICAL_SORT_H_
