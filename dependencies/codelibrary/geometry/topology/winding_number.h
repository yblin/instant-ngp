//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_NUMBER_H_
#define CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_NUMBER_H_

#include <queue>
#include <unordered_set>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/topology/arrangement_2d.h"

namespace cl {
namespace geometry {

/**
 * Get winding number for each region of the arrangement.
 *
 * The winding number can be used to check if a region is internal or external.
 */
template <typename T>
void WindingNumber(const Arrangement2D<T>& arrangement, Array<int>* winding) {
    CHECK(winding);

    using Halfedge = typename HalfedgeList<Point2D<T>>::Halfedge;
    using Region   = typename Arrangement2D<T>::Region;

    winding->clear();
    if (arrangement.empty()) return;

    const Array<Region>& regions = arrangement.regions();
    if (regions.empty()) return;

    const int external = regions.size();
    Array<Array<int>> graph(regions.size() + 1);
    for (int i = 0; i < regions.size(); ++i) {
        const Region& r = regions[i];
        std::unordered_set<int> set;
        for (auto face : r) {
            Halfedge* e = face->halfedge;
            for (Halfedge* e1 : arrangement.mesh().circular_list(e)) {
                if (arrangement.is_constraint(e1)) {
                    int id = arrangement.region_id(e1->twin());
                    if (id == -1) id = external;
                    if (set.find(id) == set.end()) {
                        set.insert(id);
                        graph[i].push_back(id);
                        graph[id].push_back(i);
                    }
                }
            }
        }
    }

    winding->resize(graph.size(), INT_MAX);
    (*winding)[external] = 0;
    std::queue<int> q;
    q.push(external);
    while (!q.empty()) {
        int source = q.front();
        q.pop();

        for (int target : graph[source]) {
            if ((*winding)[target] > (*winding)[source] + 1) {
                (*winding)[target] = (*winding)[source] + 1;
                q.push(target);
            }
        }
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_TOPOLOGY_WINDING_NUMBER_H_
