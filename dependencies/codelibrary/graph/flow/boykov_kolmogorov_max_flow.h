//
//  Copyright (c) 2006, Stephan Diederich
//
//  This code may be used under either of the following two licences:
//
//    Permission is hereby granted, free of charge, to any person
//    obtaining a copy of this software and associated documentation
//    files (the "Software"), to deal in the Software without
//    restriction, including without limitation the rights to use,
//    copy, modify, merge, publish, distribute, sublicense, and/or
//    sell copies of the Software, and to permit persons to whom the
//    Software is furnished to do so, subject to the following
//    conditions:
//
//    The above copyright notice and this permission notice shall be
//    included in all copies or substantial portions of the Software.
//
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
//    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
//    OTHER DEALINGS IN THE SOFTWARE. OF SUCH DAMAGE.
//
//  Or:
//
//    Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//    http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef CODELIBRARY_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_H_
#define CODELIBRARY_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_H_

#include <algorithm>
#include <limits>
#include <list>
#include <queue>
#include <utility>

#include "codelibrary/graph/graph.h"

namespace cl {
namespace graph {

/**
 * The Boykov-Kolmogorov max flow algorithm is an efficient way to compute the
 * max-flow for computer vision related graph.
 *
 * This code is adapted from Boost library.
 *
 * Reference:
 *   Boykov, Y., Kolmogorov, V. An Experimental Comparison of Min-Cut/Max-Flow
 *   Algorithms for Energy Minimization in Vision[J], In IEEE Transactions on
 *   Pattern Analysis and Machine Intelligence, vol. 26, no. 9, pp. 1124-1137,
 *   2004.
 */
template <typename T>
class BoykovKolmogorovMaxFlow {
    using Edge = Graph::Edge;
    using ConstIterator = Graph::EdgeList::ConstIterator;
    using EdgeProperty = Graph::EdgeProperty<T>;

    // Color for search tree.
    enum Color {
        BLACK, // For source tree.
        WHITE, // For target tree.
        GRAY   // For no tree.
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

        Initialize(graph, capacity);

        // Augment direct paths from source->target and source->vertex->target.
        AugmentDirectPaths(graph);

        // Start the main-loop.
        while (true) {
            // Find a path from source to target.
            const Edge* connecting_edge = Grow(graph);
            if (connecting_edge == nullptr) {
                // We're finished, no more paths were found.
                break;
            }

            ++current_time_;
            // Augment that path.
            Augment(connecting_edge);
            // Rebuild search tree structure.
            Adopt(graph);
        }

        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                (*flow)[e] = capacity[e] - residual_capacity_[e];
            }
        }

        return current_flow_;
    }

private:
    /**
     * Initialization.
     */
    void Initialize(const Graph& graph,
                    const Graph::EdgeProperty<T>& capacity) {
        in_active_list_.assign(n_vertices_, false);
        distance_map_.resize(n_vertices_);
        predecessor_map_.resize(n_vertices_);
        has_parent_.assign(n_vertices_, false);
        time_stamp_.assign(n_vertices_, 0);
        time_stamp_[source_] = 1;
        time_stamp_[target_] = 1;

        current_flow_ = 0;
        current_time_ = 1;
        last_grow_vertex_ = -1;

        // Initialize the search tree with gray-values.
        search_tree_.assign(n_vertices_, GRAY);

        // Initialize the search trees with the two terminals.
        search_tree_[source_] = BLACK;
        search_tree_[target_] = WHITE;

        // Initialize flow to zero which means initializing the residual
        // capacity equal to the capacity.
        residual_capacity_ = graph.AddEdgeProperty<T>();
        for (int i = 0; i < n_vertices_; ++i) {
            for (const Edge* e : graph.edges_from(i)) {
                residual_capacity_[e] = capacity[e];
            }
        }
    }

    /**
     * In a first step, we augment all direct paths from source->vertex->target
     * and additionally paths from source->target. This improves especially
     * graph cuts for segmentation, as most of the vertices have source/target
     * connects but shouldn't have an impact on other max flow problems.
     */
    void AugmentDirectPaths(const Graph& graph) {
        for (const Edge* from_source : graph.edges_from(source_)) {
            int current_vertex = from_source->target();
            if (current_vertex == target_) {
                current_flow_ += residual_capacity_[from_source];
                residual_capacity_[from_source] = 0;
                continue;
            }

            const Edge* to_target = graph.FindEdge(current_vertex, target_);
            if (to_target != nullptr) {
                T cap_from_source = residual_capacity_[from_source];
                T cap_to_target   = residual_capacity_[to_target];
                if (cap_from_source > cap_to_target) {
                    search_tree_[current_vertex] = BLACK;
                    AddActiveVertex(current_vertex);
                    SetEdgeToParent(current_vertex, from_source);
                    distance_map_[current_vertex] = 1;
                    time_stamp_[current_vertex] = 1;

                    // Add stuff to flow and update residuals. we do not need to
                    // update reverse_edges, as incoming/outgoing edges to/from
                    // source/target don't count for max-flow.
                    residual_capacity_[from_source] -= cap_to_target;
                    residual_capacity_[to_target] = 0;
                    current_flow_ += cap_to_target;
                } else if (cap_to_target > 0) {
                    search_tree_[current_vertex] = WHITE;
                    AddActiveVertex(current_vertex);
                    SetEdgeToParent(current_vertex, to_target);
                    distance_map_[current_vertex] = 1;
                    time_stamp_[current_vertex] = 1;

                    // Add stuff to flow and update residuals. we do not need to
                    // update reverse_edges, as incoming/outgoing edges to/from
                    // source/target don't count for max-flow.
                    residual_capacity_[to_target] -= cap_from_source;
                    residual_capacity_[from_source] = 0;
                    current_flow_ += cap_from_source;
                }
            } else if (residual_capacity_[from_source] > 0) {
                // there is no sink connect, so we can't augment this path, but
                // to avoid adding source to the active nodes, we just activate
                // this node and set the appropriate things.
                search_tree_[current_vertex] = BLACK;
                SetEdgeToParent(current_vertex, from_source);
                distance_map_[current_vertex] = 1;
                time_stamp_[current_vertex] = 1;
                AddActiveVertex(current_vertex);
            }
        }
        for (const Edge* e : graph.edges_from(target_)) {
            const Edge* to_target = e->twin();
            int current_vertex = to_target->source();
            if (residual_capacity_[to_target] > 0) {
                search_tree_[current_vertex] = WHITE;
                SetEdgeToParent(current_vertex, to_target);
                distance_map_[current_vertex] = 1;
                time_stamp_[current_vertex] = 1;
                AddActiveVertex(current_vertex);
            }
        }
    }

    /**
     * Return an connection edge a found path from source->target, or null if
     * no found.
     */
    const Edge* Grow(const Graph& graph) {
        CHECK(orphans_.empty());

        int current_vertex;
        while ((current_vertex = GetNextActiveVertex()) != -1) {
            CHECK(search_tree_[current_vertex] != GRAY &&
                  (has_parent_[current_vertex] ||
                   current_vertex == source_ ||
                   current_vertex == target_));

            if (current_vertex != last_grow_vertex_) {
                last_grow_vertex_   = current_vertex;
                last_grow_edge_it_  = graph.edges_from(current_vertex).begin();
                last_grow_edge_end_ = graph.edges_from(current_vertex).end();
            }

            // source or target.
            bool is_source = search_tree_[current_vertex] == BLACK;
            Color color = is_source ? BLACK : WHITE;

            for (; last_grow_edge_it_ != last_grow_edge_end_;
                 ++last_grow_edge_it_) {
                const Edge* e = is_source ? *last_grow_edge_it_
                                          : last_grow_edge_it_->twin();
                if (residual_capacity_[e] == 0) continue;

                // Check if we have capacity left on this edge.
                int other_vertex = is_source ? e->target() : e->source();

                if (search_tree_[other_vertex] == GRAY) {
                    // It's a free node.

                    // Acquire other node to our search tree.
                    if (is_source) {
                        search_tree_[other_vertex] = BLACK;
                    } else {
                        search_tree_[other_vertex] = WHITE;
                    }
                    SetEdgeToParent(other_vertex, e);
                    AddActiveVertex(other_vertex);
                    distance_map_[other_vertex] =
                            distance_map_[current_vertex] + 1;
                    time_stamp_[other_vertex] = time_stamp_[current_vertex];
                } else if (search_tree_[other_vertex] == color) {
                    // We do this to get shorter paths. check if we are nearer
                    // to the source as its parent is.
                    if (IsCloserToTerminal(current_vertex, other_vertex)) {
                        SetEdgeToParent(other_vertex, e);
                        distance_map_[other_vertex] =
                                distance_map_[current_vertex] + 1;
                        time_stamp_[other_vertex] = time_stamp_[current_vertex];
                    }
                } else {
                    CHECK(search_tree_[other_vertex] != color);

                    // Found a path from one to the other search tree,
                    // return the connecting edge in source->target direction.
                    return e;
                }
            }

            // All edges of that node are processed, and no more paths were
            // found. Remove if from the front of the active queue.
            FinishVertex(current_vertex);
        } // while active_list not empty

        // No active nodes anymore and no path found, we're done.
        return nullptr;
    }

    /**
     * Augment path from source->target and updates residual graph.
     *
     * This phase generates orphans on saturated edges, if the attached vertices
     * are from different search-trees orphans are ordered in distance to
     * target/source. First the furthest vertex from the source are
     * front_inserted into the orphans list, and after that the
     * target-tree-orphans are front_inserted. When going to adoption stage the
     * orphans are popped_front, and so we process the nearest vertices to the
     * terminals first.
     */
    void Augment(const Edge* e) {
        CHECK(search_tree_[e->target()] == WHITE);
        CHECK(search_tree_[e->source()] == BLACK);
        CHECK(orphans_.empty());

        const T bottleneck = FindBottleneck(e);
        // Now we push the found flow through the path for each edge we
        // saturate.
        // we have to look for the vertices that belong to that edge, one of
        // them becomes an orphans.
        // Now process the connecting edge.
        residual_capacity_[e] -= bottleneck;
        CHECK(residual_capacity_[e] >= 0);
        residual_capacity_[e->twin()] += bottleneck;

        // Now we follow the path back to the source.
        int current_vertex = e->source();
        while (current_vertex != source_) {
            const Edge* pred = predecessor_map_[current_vertex];
            residual_capacity_[pred] -= bottleneck;
            CHECK(residual_capacity_[pred] >= 0);
            residual_capacity_[pred->twin()] += bottleneck;

            if (residual_capacity_[pred] == 0) {
                has_parent_[current_vertex] = false;
                orphans_.push_front(current_vertex);
            }
            current_vertex = pred->source();
        }

        // Then go forward in the target-tree.
        current_vertex = e->target();
        while (current_vertex != target_) {
            const Edge* pred = predecessor_map_[current_vertex];
            residual_capacity_[pred] -= bottleneck;
            CHECK(residual_capacity_[pred] >= 0);
            residual_capacity_[pred->twin()] += bottleneck;
            if (residual_capacity_[pred] == 0) {
                has_parent_[current_vertex] = false;
                orphans_.push_front(current_vertex);
            }
            current_vertex = pred->target();
        }

        // And add it to the max-flow.
        current_flow_ += bottleneck;
    }

    /**
     * Return the bottleneck of a source->target path (end_of_path is last
     * vertex in source-tree, begin_of_path is first vertex in target-tree).
     */
    T FindBottleneck(const Edge* e) const {
        T minimum_cap = residual_capacity_[e];
        int current_vertex = e->source();

        // First go back in the source tree.
        while (current_vertex != source_) {
            const Edge* pred = predecessor_map_[current_vertex];
            minimum_cap = std::min(minimum_cap, residual_capacity_[pred]);
            current_vertex = pred->source();
        }

        // Then go forward in the target tree.
        current_vertex = e->target();
        while (current_vertex != target_) {
            const Edge* pred = predecessor_map_[current_vertex];
            minimum_cap = std::min(minimum_cap, residual_capacity_[pred]);
            current_vertex = pred->target();
        }
        return minimum_cap;
    }

    /**
     * Rebuild search trees.
     *
     * Empty the queue of orphans, and find new parents for them or just drop
     * them from the search trees.
     */
    void Adopt(const Graph& graph) {
        while (!orphans_.empty() || !child_orphans_.empty()) {
            int current_vertex;
            if (child_orphans_.empty()) {
                // Get the next orphan from the main-queue and remove it.
                current_vertex = orphans_.front();
                orphans_.pop_front();
            } else {
                current_vertex = child_orphans_.front();
                child_orphans_.pop();
            }

            if (search_tree_[current_vertex] == BLACK) {
                // We're in the source tree.
                int min_distance = INT_MAX;
                const Edge* new_parent_edge = nullptr;
                for (const Edge* e : graph.edges_from(current_vertex)) {
                    const Edge* in_edge = e->twin();

                    // We should be the target of this edge.
                    CHECK(in_edge->target() == current_vertex);
                    if (residual_capacity_[in_edge] > 0) {
                        int other_vertex = in_edge->source();
                        if (search_tree_[other_vertex] == BLACK &&
                            HasSourceConnect(other_vertex)) {
                            if (distance_map_[other_vertex] < min_distance) {
                                min_distance = distance_map_[other_vertex];
                                new_parent_edge = in_edge;
                            }
                        }
                    }
                }

                if (min_distance != INT_MAX) {
                    SetEdgeToParent(current_vertex, new_parent_edge);
                    distance_map_[current_vertex] = min_distance + 1;
                    time_stamp_[current_vertex] = current_time_;
                } else {
                    time_stamp_[current_vertex] = 0;
                    for (const Edge* e : graph.edges_from(current_vertex)) {
                        const Edge* in_edge = e->twin();
                        int other_vertex = in_edge->source();
                        if (search_tree_[other_vertex] == BLACK &&
                            other_vertex != source_) {
                            if (residual_capacity_[in_edge] > 0) {
                                AddActiveVertex(other_vertex);
                            }
                            if (has_parent_[other_vertex] &&
                                predecessor_map_[other_vertex]->source() ==
                                current_vertex) {
                                // We are the parent of that node.
                                // It has to find a new parent, too.
                                has_parent_[other_vertex] = false;
                                child_orphans_.push(other_vertex);
                            }
                        }
                    }
                    search_tree_[current_vertex] = GRAY;
                } // no parent found.
            } else {
                // Now we should be in the target-tree, check that.
                CHECK(search_tree_[current_vertex] == WHITE);
                const Edge* new_parent_edge = nullptr;
                int min_distance = INT_MAX;
                for (const Edge* e : graph.edges_from(current_vertex)) {
                    if (residual_capacity_[e] > 0) {
                        const int other_vertex = e->target();
                        if (search_tree_[other_vertex] == WHITE &&
                            HasTargetConnect(other_vertex)) {
                            if (distance_map_[other_vertex] < min_distance) {
                                min_distance = distance_map_[other_vertex];
                                new_parent_edge = e;
                            }
                        }
                    }
                }

                if (min_distance != INT_MAX) {
                    SetEdgeToParent(current_vertex, new_parent_edge);
                    distance_map_[current_vertex] = min_distance + 1;
                    time_stamp_[current_vertex] = current_time_;
                } else {
                    time_stamp_[current_vertex] = 0;
                    for (const Edge* e : graph.edges_from(current_vertex)) {
                        int other_vertex = e->target();
                        if (search_tree_[other_vertex] == WHITE &&
                            other_vertex != target_) {
                            if (residual_capacity_[e] > 0) {
                                AddActiveVertex(other_vertex);
                            }
                            if (has_parent_[other_vertex] &&
                                predecessor_map_[other_vertex]->target() ==
                                current_vertex) {
                                // We were it's parent, so it has to find a
                                // new one, too.
                                has_parent_[other_vertex] = false;
                                child_orphans_.push(other_vertex);
                            }
                        }
                    }
                    search_tree_[current_vertex] = GRAY;
                } // no parent found
            } // target-tree adoption
        } // while !orphans.empty()
    } // adopt

    /**
     * Add v as an active vertex, but only if its not in the list already.
     */
    void AddActiveVertex(int v) {
        CHECK(search_tree_[v] != GRAY);

        if (in_active_list_[v]) {
            if (last_grow_vertex_ == v) {
                last_grow_vertex_ = -1;
            }
            return;
        }

        in_active_list_[v] = true;
        active_list_.push(v);
    }

    /**
     * Remove a vertex from the front of the active queue (its called in grow
     * phase, if no more paths can be found using this vertex).
     */
    void FinishVertex(int v) {
        CHECK(active_list_.front() == v);

        active_list_.pop();
        in_active_list_[v] = false;
        last_grow_vertex_ = -1;
    }

    /**
     * Set given edge to parent vertex of v;
     */
    void SetEdgeToParent(int v, const Edge* e) {
        CHECK(residual_capacity_[e] > 0);

        predecessor_map_[v] = e;
        has_parent_[v] = true;
    }

    /**
     * Return the next active vertex if there is one, otherwise a nullptr_vertex.
     */
    int GetNextActiveVertex() {
        while (true) {
            if (active_list_.empty()) return -1;
            int v = active_list_.front();

            // If it has no parent, this node can't be active (if its not source
            // or target).
            if (!has_parent_[v] && v != source_ && v != target_) {
                active_list_.pop();
                in_active_list_[v] = false;
            } else {
                CHECK(search_tree_[v] == BLACK || search_tree_[v] == WHITE);
                return v;
            }
        }
    }

    /**
     * Check if vertex v has a connect to the target-vertex
     *
     * Parameter:
     *   v the vertex which is checked.
     * 
     * Return:
     *   true if a path to the target was found, false if not.
     */
    bool HasTargetConnect(int v) {
        int current_distance = 0;
        int current_vertex = v;
        while (true) {
            if (time_stamp_[current_vertex] == current_time_) {
                // We found a vertex which was already checked this round.
                // Use it for distance calculations.
                current_distance += distance_map_[current_vertex];
                break;
            }
            if (current_vertex == target_) {
                time_stamp_[target_] = current_time_;
                break;
            }
            if (has_parent_[current_vertex]) {
                current_vertex = predecessor_map_[current_vertex]->target();
                ++current_distance;
            } else {
                // No path found.
                return false;
            }
        }

        current_vertex = v;
        while (time_stamp_[current_vertex] != current_time_) {
            distance_map_[current_vertex] = current_distance;
            --current_distance;
            time_stamp_[current_vertex] = current_time_;
            current_vertex = predecessor_map_[current_vertex]->target();
        }
        return true;
    }

    /**
     * Check if vertex v has a connect to the source-vertex.
     *
     * Parameter:
     *   v - the vertex which is checked.
     * 
     * Return:
     *   true if a path to the source was found, false if not.
     */
    bool HasSourceConnect(int v) {
        int current_distance = 0;
        int current_vertex = v;
        while (true) {
            if (time_stamp_[current_vertex] == current_time_) {
                // We found a vertex which was already checked this round.
                // Use it for distance calculations.
                current_distance += distance_map_[current_vertex];
                break;
            }
            if (current_vertex == source_) {
                time_stamp_[source_] = current_time_;
                break;
            }
            if (has_parent_[current_vertex]) {
                current_vertex = predecessor_map_[current_vertex]->source();
                ++current_distance;
            } else {
                // No path found.
                return false;
            }
        }

        current_vertex = v;
        while (time_stamp_[current_vertex] != current_time_) {
            distance_map_[current_vertex] = current_distance;
            --current_distance;
            time_stamp_[current_vertex] = current_time_;
            current_vertex = predecessor_map_[current_vertex]->source();
        }
        return true;
    }

    /**
     * Returns true, if p is closer to a terminal than q.
     */
    bool IsCloserToTerminal(int p, int q) const {
        return time_stamp_[q] <= time_stamp_[p] &&
               distance_map_[q] > distance_map_[p] + 1;
    }

    // Source vertex of net flow.
    int source_ = -1;

    // Target vertex of net flow.
    int target_ = -1;

    // Number of vertices of net flow.
    int n_vertices_ = 0;

    // Number of edges of net flow.
    int n_edges_ = 0;

    // Residual capacity for each edge.
    Graph::EdgeProperty<T> residual_capacity_;

    // For child orphans.
    std::list<int> orphans_;
    std::queue<int> child_orphans_;

    // Store the distance to source/target vertices.
    Array<int> distance_map_;

    // Active list to store the active vertices.
    std::queue<int> active_list_;

    // Check if the vertex is in active list.
    Array<bool> in_active_list_;

    // Store paths found in the growth stage.
    Array<const Edge*> predecessor_map_;

    // Check if each vertex has parent.
    Array<bool> has_parent_;

    // Time stamp of each vertex, used for target/source-path calculations.
    Array<int> time_stamp_;

    // Search tree for each vertex.
    Array<int> search_tree_;

    // Maximal/current flow.
    T current_flow_ = 0;

    // Current time, for time stamp.
    int current_time_ = 0;

    // The last vertex in the active list.
    int last_grow_vertex_ = 0;

    ConstIterator last_grow_edge_it_, last_grow_edge_end_;
};

} // namespace graph
} // namespace cl

#endif // CODELIBRARY_GRAPH_FLOW_BOYKOV_KOLMOGOROV_MAX_FLOW_H_
