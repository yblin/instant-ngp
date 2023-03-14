//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_POINT_CLOUD_MORTON_ORDER_H_
#define CODELIBRARY_POINT_CLOUD_MORTON_ORDER_H_

#include <climits>
#include <cmath>
#include <cstdint>
#include <type_traits>

#include "codelibrary/base/bits.h"
#include "codelibrary/base/float.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_3d.h"

namespace cl {
namespace point_cloud {

/**
 * Morton order map multidimensional data to one dimension while preserving
 * locality of the data points.
 *
 * The morton order of a point in multidimensions is simply calculated by
 * interleaving the binary representations of its coordinate values.
 *
 * The resulting ordering can equivalently be described as the order one would
 * get from a depth-first traversal of a quadtree or octree.
 */
class MortonOrder {
public:
    /**
     * The relative order of the two points is determined by the pair of
     * coordinates who have the first differing bit with the highest exponent.
     *
     * Reference:
     *   Connor, Michael, and Piyush Kumar. "Fast construction of k-nearest
     *   neighbor graphs for point clouds." IEEE transactions on visualization
     *   and computer graphics 16.4 (2010): 599-608.
     */
    template <typename Point>
    bool operator() (const Point& a, const Point& b) const {
        using T = typename Point::value_type;
        static_assert(std::is_floating_point<T>::value, "");

        int d = a.size();
        CHECK(d == b.size());

        int x = INT_MIN, dim = 0;
        for (int j = 0; j < d; ++j) {
            if ((a[j] < 0) != (b[j] < 0)) return a[j] < b[j];

            int y = XORMSB(a[j], b[j]);
            if (x < y) {
                x = y;
                dim = j;
            }
        }

        return a[dim] < b[dim];
    }

private:
    /**
     * Compute highest exponent of two points by first comparing the exponents
     * of the coordinates, then comparing the bits in the mantissa, if the
     * exponents are equal.
     */
    template <typename T>
    static int XORMSB(const T& a, const T& b) {
        if (a == b || a == -b) return INT_MIN;

        Float<T> p1(a), p2(b);
        int exp1 = p1.exponent(), exp2 = p2.exponent();
        if (exp1 == exp2) {
            auto c = (p1.mantissa() ^ p2.mantissa());
            if (c == 0) return INT_MAX;
            return exp1 - Float<T>::N_MANTISSA_BITS + bits::Log2Floor(c);
        }

        return std::max(exp1, exp2);
    }
};

} // namespace point_cloud
} // namespace cl

#endif // CODELIBRARY_POINT_CLOUD_MORTON_ORDER_H_
