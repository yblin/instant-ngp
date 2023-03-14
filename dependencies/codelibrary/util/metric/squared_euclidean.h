//
// Copyright 2014 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_METRIC_SQUARED_EUCLIDEAN_H_
#define CODELIBRARY_UTIL_METRIC_SQUARED_EUCLIDEAN_H_

#include <cmath>

#include "codelibrary/base/log.h"

namespace cl {
namespace metric {

/**
 * Squared Euclidean distance for two n-dimensional points.
 */
class SquaredEuclidean {
public:
    SquaredEuclidean() = default;

    SquaredEuclidean(const SquaredEuclidean&) = delete;

    SquaredEuclidean& operator=(const SquaredEuclidean&) = delete;

    template <typename T>
    double operator() (const T& a, const T& b) const {
        auto size1 = a.size();
        auto size2 = b.size();
        CHECK(size1 == size2);

        double t = 0.0;
        for (decltype(size1) i = 0; i < size1; ++i) {
            t += static_cast<double>(a[i] - b[i]) * (a[i] - b[i]);
        }

        return t;
    }
};

} // namespace metric
} // namespace cl

#endif // CODELIBRARY_UTIL_METRIC_SQUARED_EUCLIDEAN_H_
