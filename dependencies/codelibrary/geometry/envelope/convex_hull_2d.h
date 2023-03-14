//
// Copyright 2012-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_H_
#define CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_H_

#include <algorithm>

#include "codelibrary/base/array.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/predicate_2d.h"

namespace cl {
namespace geometry {

/**
 * Generate the planar convex hull of 2D points, represented as a list of
 * points arranged in counter-clockwise order.
 */
template <typename T>
class ConvexHull2D {
    using Point = Point2D<T>;

public:
    ConvexHull2D(const Array<Point>& points) {
        int size = points.size();
        if (size == 0) return;

        Array<int> seq;
        IndexSort(points.begin(), points.end(), &seq);

        if (size == 1) {
            vertices_.push_back(points[0]);
            return;
        }

        if (size == 2) {
            if (points[0] == points[1]) {
                vertices_.push_back(points[0]);
            } else {
                vertices_.push_back(points[seq[0]]);
                vertices_.push_back(points[seq[1]]);
            }
            return;
        }

        for (int is_upper_hull = 0; is_upper_hull <= 1; ++is_upper_hull) {
            if (is_upper_hull) std::reverse(seq.begin(), seq.end());

            Array<int> stack(size);
            stack[0] = 0;
            stack[1] = 1;
            int stack_top = 1;
            for (int i = 2; i < size; ++i) {
                if (Orientation(points[seq[0]], points[seq[stack[1]]],
                                points[seq[i]]) <= 0) {
                    stack[1] = i;
                }
            }

            for (int i = stack[stack_top] + 1; i < size; ++i) {
                while (Orientation(points[seq[stack[stack_top - 1]]],
                                   points[seq[stack[stack_top]]],
                                   points[seq[i]]) <= 0) {
                    --stack_top;
                }
                stack[++stack_top] = i;
            }
            for (int i = 0; i < stack_top; ++i) {
                vertices_.push_back(points[seq[stack[i]]]);
            }
        }
    }

    /**
     * Return the vertices of convex hull in counter-clockwise order.
     */
    const Array<Point>& vertices() const {
        return vertices_;
    }

    bool empty() const {
        return vertices_.empty();
    }

private:
    // Vertices of convex hull in counter-clockwise order.
    Array<Point> vertices_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_H_
