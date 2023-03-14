//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_HALFEDGE_GRAPH_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_HALFEDGE_GRAPH_H_

#include <unordered_map>
#include <unordered_set>

#include "codelibrary/geometry/mesh/halfedge_list.h"
#include "codelibrary/geometry/polygon_2d.h"

namespace cl {
namespace geometry {

/**
 * A graph consisting of halfedges.
 */
template <class Vertex, class Edge>
class HalfedgeGraph {
public:
    HalfedgeGraph() = default;

    HalfedgeGraph(const Array<Edge*>& edges) {
        for (auto e : edges) {
            Insert(e);
        }
    }

    /**
     * Insert a halfedge.
     */
    void Insert(Edge* e) {
        edge_lists_[e->source()].push_back(e);
    }

    const std::unordered_map<Vertex*, Array<Edge*>>& edge_lists() const {
        return edge_lists_;
    }

private:
    std::unordered_map<Vertex*, Array<Edge*>> edge_lists_;
};

/**
 * Get all euler cycles of the graph.
 */
template <class Vertex, class Edge>
void GetEulerCycles(const HalfedgeGraph<Vertex, Edge>& graph,
                    Array<Array<Edge*>>* cycles) {
    CHECK(cycles);

    cycles->clear();

    std::unordered_set<Edge*> visited;
    std::unordered_map<Vertex*, Array<Edge*>> edge_lists = graph.edge_lists();
    for (auto p : edge_lists) {
        for (auto e : p.second) {
            if (visited.find(e) != visited.end()) continue;

            Array<std::pair<Vertex*, Edge*>> stack;
            std::unordered_set<Vertex*> in_stack;
            visited.insert(e);
            in_stack.insert(e->source());
            stack.emplace_back(e->source(), nullptr);

            while (!stack.empty()) {
                Vertex* cur = stack.back().first;
                Array<Edge*>& s = edge_lists[cur];
                if (s.empty()) break;

                Edge* e = s.back();
                visited.insert(e);
                s.pop_back();

                if (in_stack.find(e->target()) != in_stack.end()) {
                    // Found a cycle.
                    Array<Edge*> edge_list;
                    edge_list.push_back(e);
                    while (stack.back().first != e->target()) {
                        edge_list.push_back(stack.back().second);
                        in_stack.erase(stack.back().first);
                        stack.pop_back();
                    }
                    std::reverse(edge_list.begin(), edge_list.end());
                    cycles->push_back(edge_list);
                } else {
                    stack.emplace_back(e->target(), e);
                    in_stack.insert(e->target());
                }
            }
        }
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_HALFEDGE_GRAPH_H_
