//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GRAPH_FLOW_IMPROVED_SAP_MAX_FLOW_H_
#define CODELIBRARY_GRAPH_FLOW_IMPROVED_SAP_MAX_FLOW_H_

#include <algorithm>
#include <queue>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * Improved shortest augmenting path algorithm to calculate the max flow.
 * Time complexity: O(V^2 * E), very fast in practice.
 */
template <typename T>
class ImprovedSAPMaxFlow {
    using Edge = Graph::Edge;
    using ConstIterator = Graph::EdgeList::ConstIterator;

public:
    /**
     * Parameters:
     *  graph    - the input graph.
     *  capacity - the capacity for each edge.
     *  source   - the source vertex of the flow network graph.
     *  target   - the target (sink) vertex of the flow network graph.
     *  flow     - the output flow for each edge.
     *
     * Return:
     *  the maximal flow of the given graph.
     */
    T operator() (const Graph& graph,
                  const Graph::EdgeProperty<T>& capacity,
                  int source,
                  int target,
                  Graph::EdgeProperty<T>* flow) {
        // Check if the input parameters are valid.
        CHECK(graph.IsBidirectional());
        CHECK(flow);

        int n = graph.n_vertices();

        CHECK(0 <= source && source < n);
        CHECK(0 <= target && target < n);

        if (source == target) return T(0);

        for (int i = 0; i < n; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                CHECK(capacity[e] >= T(0)) << "Capacity must be nonnegative.";
                (*flow)[e] = T(0);
            }
        }

        // Get the BFS level of reverse residual graph.
        Array<int> level(n, n);
        std::queue<int> q;
        q.push(target);
        level[target] = 0;

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            for (const Edge* e : graph.edges_from(cur)) {
                if (level[e->target()] != n || capacity[e->twin()] == 0)
                    continue;

                level[e->target()] = level[e->source()] + 1;
                q.push(e->target());
            }
        }

        // Count the number of each level.
        Array<int> n_level(n + 1, 0);
        for (int i = 0; i < n; ++i) {
            ++n_level[level[i]];
        }

        Array<const Edge*> path(n);
        Array<ConstIterator> cur(n);
        for (int i = 0; i < n; ++i) {
            cur[i] = graph.edges_from(i).begin();
        }

        T total_flow = 0;
        int u = source; // Current vertex.
        while (level[source] < n) {
            if (u == target) {
                // Found an augmenting path.
                T increment = capacity[path[target]] - (*flow)[path[target]];
                for (int i = target; i != source; i = path[i]->source()) {
                    const Edge* e = path[i];
                    increment = std::min(increment, capacity[e] - (*flow)[e]);
                }

                for (int i = target; i != source; i = path[i]->source()) {
                    (*flow)[path[i]] += increment;
                    (*flow)[path[i]->twin()] -= increment;
                }
                total_flow += increment;
                u = source;
            }

            for (; cur[u] != graph.edges_from(u).end(); ++cur[u]) {
                const Edge* e = *cur[u];
                int v = e->target();

                if (capacity[e] > (*flow)[e] && level[u] == level[v] + 1) {
                    u = v;
                    path[v] = e;
                    break;
                }
            }

            if (cur[u] == graph.edges_from(u).end()) {
                // If no admissible arc, relabel this vertex.
                if (--n_level[level[u]] == 0) break;

                // Backtrack.
                cur[u] = graph.edges_from(u).begin();

                int min_level = n - 1;
                for (const Edge* e : graph.edges_from(u)) {
                    if (capacity[e] > (*flow)[e]) {
                        min_level = std::min(min_level, level[e->target()]);
                    }
                }

                level[u] = min_level + 1;
                ++n_level[level[u]];

                if (u != source) {
                    // Backtrack.
                    u = path[u]->source();
                }
            }
        }

        return total_flow;
    }
};

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_FLOW_IMPROVED_SAP_MAX_FLOW_H_
