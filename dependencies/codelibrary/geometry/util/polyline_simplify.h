//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_UTIL_POLYLINE_SIMPLIFY_H_
#define CODELIBRARY_GEOMETRY_UTIL_POLYLINE_SIMPLIFY_H_

#include <queue>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/polyline_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace geometry {

/**
 * The Douglas-Peucker algorithm to simplify the given polyline.
 *
 * The algorithm defines 'dissimilar' based on the maximum distance between the
 * original curve and the simplified curve (i.e., the Hausdorff distance between
 * the curves). The simplified curve consists of a subset of the points that
 * defined the original curve.
 */
template <typename T>
void DouglasPeucker(const Array<Point2D<T>>& polyline, double threshold,
                    Array<Point2D<T>>* result) {
    CHECK(result != &polyline);
    CHECK(threshold > 0.0);
    CHECK(result);

    if (polyline.size() < 2) {
        *result = polyline;
        return;
    }

    Array<bool> is_remain(polyline.size(), false);
    std::queue<std::pair<int, int>> q;
    q.push(std::make_pair(0, polyline.size() - 1));
    while (!q.empty()) {
        std::pair<int, int> p = q.front();
        q.pop();

        is_remain[p.first] = true;
        is_remain[p.second] = true;

        Segment2D<T> seg(polyline[p.first], polyline[p.second]);
        int split = -1;
        double dis = 0.0;
        for (int i = p.first + 1; i < p.second; ++i) {
            double d = Distance(polyline[i], seg);
            if (d > dis) {
                dis = d;
                split = i;
            }
        }

        if (split == -1 || dis < threshold) continue;

        if (p.first < split) q.push(std::make_pair(p.first, split));
        if (split < p.second) q.push(std::make_pair(split, p.second));
    }

    result->clear();
    for (int i = 0; i < polyline.size(); ++i) {
        if (is_remain[i]) result->push_back(polyline[i]);
    }
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_UTIL_POLYLINE_SIMPLIFY_H_
