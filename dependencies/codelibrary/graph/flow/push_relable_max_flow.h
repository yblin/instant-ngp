//
// Copyright 2000 University of Notre Dame.
// Authors: Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef CODELIBRARY_GRAPH_FLOW_PUSH_RELABLE_MAX_FLOW_H_
#define CODELIBRARY_GRAPH_FLOW_PUSH_RELABLE_MAX_FLOW_H_

#include <algorithm>
#include <limits>
#include <list>
#include <queue>
#include <utility>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * Highest-label version of the push-relabel method with the global relabeling
 * and gap relabeling heuristics.
 *
 * Time complexity: O(V^3).
 *
 * This code is adapted from Boost library.
 *
 * Reference:
 *  Cherkassy B V, Goldberg A V. On Implementing Push-Relabel Method for the
 *  Maximum Flow Problem[C]. International IPCO Conference on Integer
 *  Programming and Combinatorial Optimization. Springer-Verlag, 1995:157-171.
 */
template <typename T>
class PushRelableMaxFlow {
    using Edge = Graph::Edge;
    using ConstIterator = Graph::EdgeList::ConstIterator;

    enum Color {
        WHITE, GRAY, BLACK
    };

    /**
     * A "layer" is a group of vertices with the same distance. The vertices in
     * each layer are categorized as active or inactive.  An active vertex has
     * positive excess flow and its distance is less than n (it is not blocked).
     */
    struct Layer {
        std::list<int> active_vertices;
        std::list<int> inactive_vertices;
    };

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

        n_vertices_ = graph.n_vertices();
        n_edges_ = graph.n_edges() / 2;
        source_ = source;
        target_ = target;

        CHECK(0 <= source_ && source_ < n_vertices_);
        CHECK(0 <= target_ && target_ < n_vertices_);

        if (source_ == target_) return T(0);

        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                CHECK(capacity[e] >= T(0)) <<
                    "The capacity of graph's edge must be greater than 0.";
            }
        }

        Initialize(graph, capacity, flow);
        T total_flow = MaximumPreflow(graph);
        ConvertPreflowToFlow(graph, capacity, flow);
        return total_flow;
    }

private:
    /**
     * Initialization.
     */
    void Initialize(const Graph& graph,
                    const Graph::EdgeProperty<T>& capacity,
                    Graph::EdgeProperty<T>* flow) {
        residual_capacity_ = graph.AddEdgeProperty<T>();
        excess_flow_.resize(n_vertices_);
        current_.resize(n_vertices_);
        layers_.resize(n_vertices_);
        distance_.resize(n_vertices_);
        color_.resize(n_vertices_);
        layer_list_ptr_.resize(n_vertices_);

        // Initialize flow to zero which means initializing the residual
        // capacity to equal the capacity.
        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                (*flow)[e] = T(0);
                residual_capacity_[e] = capacity[e];
            }
        }

        for (int i = 0; i < n_vertices_; ++i) {
            excess_flow_[i] = 0;
            current_[i] = std::make_pair(graph.edges_from(i).begin(),
                                         graph.edges_from(i).end());
        }

        bool overflow_detected = false;
        T test_excess = 0;
        for (const Edge* e : graph.edges_from(source_)) {
            if (e->target() != source_) {
                test_excess += residual_capacity_[e];
            }
        }
        if (test_excess > (std::numeric_limits<T>::max())) {
            overflow_detected = true;
        }

        if (overflow_detected) {
            excess_flow_[source_] = std::numeric_limits<T>::max();
        } else {
            excess_flow_[source_] = 0;
            for (const Edge* e : graph.edges_from(source_)) {
                if (e->target() != source_) {
                    T delta = residual_capacity_[e];
                    residual_capacity_[e] -= delta;
                    residual_capacity_[e->twin()] += delta;
                    excess_flow_[e->target()] += delta;
                }
            }
        }
        max_distance_ = n_vertices_ - 1;
        max_active_ = 0;
        min_active_ = n_vertices_;

        for (int u = 0; u < n_vertices_; ++u) {
            if (u == target_) {
                distance_[u] = 0;
                continue;
            }

            if (u == source_ && !overflow_detected) {
                distance_[u] = n_vertices_;
            } else {
                distance_[u] = 1;
            }

            if (excess_flow_[u] > 0) {
                AddToActiveList(u, &layers_[1]);
            } else if (distance_[u] < n_vertices_) {
                AddToInactiveList(u, &layers_[1]);
            }
        }
    }

    /**
     * This is the core part of the algorithm, "phase one".
     */
    T MaximumPreflow(const Graph& graph) {
        work_since_last_update_ = 0;

        while (max_active_ >= min_active_) { // "main" loop.
            Layer& layer = layers_[max_active_];
            std::list<int>::iterator u_iter = layer.active_vertices.begin();

            if (u_iter == layer.active_vertices.end()) {
                --max_active_;
            } else {
                int u = *u_iter;
                RemoveFromActiveList(u);

                Discharge(graph, u);

                if (work_since_last_update_ / 2 > 6 * n_vertices_ + n_edges_) {
                    GlobalDistanceUpdate(graph);
                    work_since_last_update_ = 0;
                }
            }
        } // while (max_active >= min_active)

        return excess_flow_[target_];
    }

    /**
     * Remove excess flow, the "second phase".
     * This does a DFS on the reverse flow graph of nodes with excess flow.
     * If a cycle is found, cancel it.
     * Return the nodes with excess flow in topological order.
     */
    void ConvertPreflowToFlow(const Graph& graph,
                              const Graph::EdgeProperty<T>& capacity,
                              Graph::EdgeProperty<T>* flow) {
        Array<int> parent(n_vertices_);
        Array<int> topo_next(n_vertices_);

        int tos, bos;
        bool bos_nullptr = true;

        // Handle self-loops.
        for (int u = 0; u < n_vertices_; ++u) {
            for (const Edge* e : graph.edges_from(u)) {
                if (e->target() == u) {
                    residual_capacity_[e] = capacity[e];
                }
            }
        }

        // Initialize.
        for (int u = 0; u < n_vertices_; ++u) {
            color_[u] = WHITE;
            parent[u] = u;
            current_[u] = std::make_pair(graph.edges_from(u).begin(),
                                         graph.edges_from(u).end());
        }

        int restart;

        // Eliminate flow cycles and topologically order the vertices.
        for (int i = 0; i < n_vertices_; ++i) {
            int u = i;
            if (color_[u] == WHITE && excess_flow_[u] > 0 &&
                u != source_ && u != target_) {
                int r = u;
                color_[r] = GRAY;
                while (true) {
                        for (; current_[u].first != current_[u].second;
                         ++current_[u].first) {
                        const Edge* e = *current_[u].first;
                        if (capacity[e] == 0 && IsResidualEdge(e)) {
                            int v = e->target();
                            if (color_[v] == WHITE) {
                                color_[v] = GRAY;
                                parent[v] = u;
                                u = v;
                                break;
                            }

                            if (color_[v] == GRAY) {
                                // Find minimum flow on the cycle.
                                T delta = residual_capacity_[e];
                                while (true) {
                                    const Edge* e1 = *current_[v].first;
                                    delta = std::min(delta,
                                                     residual_capacity_[e1]);
                                    if (v == u) break;

                                    v = e1->target();
                                }

                                // Remove delta flow units.
                                v = u;
                                while (true) {
                                    e = *current_[v].first;
                                    residual_capacity_[e] -= delta;
                                    residual_capacity_[e->twin()] += delta;
                                    v = e->target();
                                    if (v == u)
                                        break;
                                }

                                // Back-out of DFS to the first saturated edge.
                                restart = u;
                                for (v = current_[u].first->target(); v != u;
                                     v = e->target()) {
                                    e = *current_[v].first;
                                    if (color_[v] == WHITE || IsSaturated(e)) {
                                        color_[e->target()] = WHITE;
                                        if (color_[v] != WHITE)
                                            restart = v;
                                    }
                                }

                                if (restart != u) {
                                    u = restart;
                                    ++current_[u].first;
                                    break;
                                }
                            } // else if (color[v] == GRAY)
                        } // if (capacity[a]) == 0 ...
                    } // for

                    if (current_[u].first == current_[u].second) {
                        // Scan of i is complete.
                        color_[u] = BLACK;
                        if (u != source_) {
                            if (bos_nullptr) {
                                bos = u;
                                bos_nullptr = false;
                                tos = u;
                            } else {
                                topo_next[u] = tos;
                                tos = u;
                            }
                        }
                        if (u != r) {
                            u = parent[u];
                            ++current_[u].first;
                        } else {
                            break;
                        }
                    }
                } // while (true)
            } // if (color[u] == WHITE && excess_flow[u] > 0 & ...)
        } // for all vertices in graph

        // Return excess flows.
        // Note that the sink is not on the stack.
        if (!bos_nullptr) {
            for (int u = tos; u != bos; u = topo_next[u]) {
                ConstIterator ai = graph.edges_from(u).begin();
                ConstIterator a_end = graph.edges_from(u).end();
                while (excess_flow_[u] > 0 && ai != a_end) {
                    if (capacity[*ai] == 0 && IsResidualEdge(*ai))
                        PushFlow(*ai);
                       ++ai;
                }
            }

            // Do the bottom.
            int u = bos;
            ConstIterator ai = graph.edges_from(u).begin();
            ConstIterator a_end = graph.edges_from(u).end();
            while (excess_flow_[u] > 0 && ai != a_end) {
                if (capacity[*ai] == 0 && IsResidualEdge(*ai))
                    PushFlow(*ai);
                ++ai;
            }
        }

        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                (*flow)[e] = capacity[e] - residual_capacity_[e];
            }
        }
    }

    /**
     * This is a breadth-first search over the reverse of the residual graph.
     */
    void GlobalDistanceUpdate(const Graph& graph) {
        for (int u = 0; u < n_vertices_; ++u) {
            color_[u] = WHITE;
            distance_[u] = n_vertices_;
        }
        color_[target_] = GRAY;
        distance_[target_] = 0;

        for (int l = 0; l <= max_distance_; ++l) {
            layers_[l].active_vertices.clear();
            layers_[l].inactive_vertices.clear();
        }

        max_distance_ = max_active_ = 0;
        min_active_ = n_vertices_;

        std::queue<int> q;
        q.push(target_);
        while (!q.empty()) {
            int u = q.front();
            q.pop();
            int d_v = distance_[u] + 1;

            for (const Edge* e : graph.edges_from(u)) {
                int v = e->target();
                if (color_[v] == WHITE && IsResidualEdge(e->twin())) {
                    distance_[v] = d_v;
                    color_[v] = GRAY;
                    current_[v] = std::make_pair(graph.edges_from(v).begin(),
                                                 graph.edges_from(v).end());
                    max_distance_ = std::max(max_distance_, d_v);

                    if (excess_flow_[v] > 0) {
                        AddToActiveList(v, &layers_[d_v]);
                    } else {
                        AddToInactiveList(v, &layers_[d_v]);
                    }

                    q.push(v);
                }
            }
        }
    }

    void Discharge(const Graph& graph, int u) {
        CHECK(excess_flow_[u] > 0);

        while (true) {
            ConstIterator e = current_[u].first;
            for (; e != current_[u].second; ++e) {
                if (IsResidualEdge(*e)) {
                    int v = e->target();
                    if (IsAdmissible(u, v)) {
                        if (v != target_ && excess_flow_[v] == 0) {
                            RemoveFromInactiveList(v);
                            AddToActiveList(v, &layers_[distance_[v]]);
                        }
                        PushFlow(*e);
                        if (excess_flow_[u] == 0) break;
                    }
                }
            } // For edges of u starting from current.

            Layer& layer = layers_[distance_[u]];
            int du = distance_[u];

            if (e == current_[u].second) { // i must be relabeled.
                RelabelDistance(graph, u);
                if (layer.active_vertices.empty() &&
                    layer.inactive_vertices.empty())
                    Gap(du);
                if (distance_[u] == n_vertices_) break;
            } else {              // i is no longer active
                current_[u].first = e;
                AddToInactiveList(u, &layer);
                break;
            }
        } // while (true)
    }

    /**
     * Cleanup beyond the gap.
     */
    void Gap(int empty_distance) {
        // Distance of layer before the current layer.
        int r = empty_distance - 1;

        // Set the distance for the vertices beyond the gap to "infinity".
        for (int l = empty_distance + 1; l < max_distance_; ++l) {
            for (int u : layers_[l].inactive_vertices) {
                distance_[u] = n_vertices_;
            }
            layers_[l].inactive_vertices.clear();
        }
        max_distance_ = r;
        max_active_ = r;
    }

    /**
     * The main purpose of this routine is to set distance[v] to the smallest
     * value allowed by the valid labeling constraints, which are:
     *   distance[t] = 0
     *   distance[u] <= distance[v] + 1   for every residual edge (u,v)
     */
    int RelabelDistance(const Graph& graph, int u) {
        work_since_last_update_ += 12;
        int min_distance = n_vertices_;
        distance_[u] = min_distance;

        // Examine the residual out-edges of vertex i, choosing the edge whose
        // target vertex has the minimal distance.
        ConstIterator min_edge_iter;
        for (ConstIterator i = graph.edges_from(u).begin();
             i != graph.edges_from(u).end(); ++i) {
            ++work_since_last_update_;
            const Edge* e = *i;
            int v = e->target();
            if (IsResidualEdge(e) && distance_[v] < min_distance) {
                min_distance = distance_[v];
                min_edge_iter = i;
            }
        }
        ++min_distance;
        if (min_distance < n_vertices_) {
            distance_[u] = min_distance; // This is the main action.
            current_[u].first = min_edge_iter;
            max_distance_ = std::max(min_distance, max_distance_);
        }
        return min_distance;
    }

    /**
     * Push the excess flow from from vertex u to v.
     */
    void PushFlow(const Edge* u_v) {
        int u = u_v->source();
        int v = u_v->target();

        T flow_delta = std::min(excess_flow_[u], residual_capacity_[u_v]);
        residual_capacity_[u_v] -= flow_delta;
        residual_capacity_[u_v->twin()] += flow_delta;

        excess_flow_[u] -= flow_delta;
        excess_flow_[v] += flow_delta;
    }


    //=======================================================================
    // Some helper predicates

    bool IsAdmissible(int u, int v) const {
        return distance_[u] == distance_[v] + 1;
    }

    bool IsResidualEdge(const Edge* e) const {
        return residual_capacity_[e] > 0;
    }

    bool IsSaturated(const Edge* e) const {
        return residual_capacity_[e] == 0;
    }

    // ========================================================================
    // Layer List Management Functions.

    void AddToActiveList(int u, Layer* layer) {
        layer->active_vertices.push_front(u);
        max_active_ = std::max(max_active_, distance_[u]);
        min_active_ = std::min(min_active_, distance_[u]);
        layer_list_ptr_[u] = layer->active_vertices.begin();
    }

    void RemoveFromActiveList(int u) {
        layers_[distance_[u]].active_vertices.erase(layer_list_ptr_[u]);
    }

    void AddToInactiveList(int u, Layer* layer) {
        layer->inactive_vertices.push_front(u);
        layer_list_ptr_[u] = layer->inactive_vertices.begin();
    }

    void RemoveFromInactiveList(int u) {
        layers_[distance_[u]].inactive_vertices.erase(layer_list_ptr_[u]);
    }

    //=========================================================================

    int source_{};       // Source vertex of net flow.
    int target_{};       // Target vertex of net flow.
    int n_vertices_{};   // Number of vertices of net flow.
    int n_edges_{};      // Number of edges of net flow.
    int min_active_{};   // Minimal distance with active node.
    int max_active_{};   // Maximal distance with active node.
    int max_distance_{}; // Maximal distance.

    int work_since_last_update_{};

    Array<Color> color_;  // Color label for each vertex.
    Array<int> distance_; // Distance of each vertex.

    Array<Layer> layers_;
    Array<std::list<int>::iterator> layer_list_ptr_;

    Array<std::pair<ConstIterator, ConstIterator>> current_;
    Graph::EdgeProperty<T> residual_capacity_;
    Array<T> excess_flow_;
};

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_FLOW_PUSH_RELABLE_MAX_FLOW_H_
