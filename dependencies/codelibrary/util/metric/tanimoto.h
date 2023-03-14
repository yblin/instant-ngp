//
// Copyright 2018 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_METRIC_TANIMOTO_H_
#define CODELIBRARY_UTIL_METRIC_TANIMOTO_H_

#include <algorithm>
#include <cmath>

#include "codelibrary/base/log.h"

namespace cl {
namespace metric {

/**
 * Tanimoto distance can be written as:
 *
 *                         AB
 * f(A, B) = 1 -  --------------------
 *                 |A|^2 + |B|^2 - AB
 */
class Tanimoto {
public:
    Tanimoto() = default;

    Tanimoto(const Tanimoto&) = delete;

    Tanimoto& operator=(const Tanimoto&) = delete;

    template <typename T>
    double operator() (const T& a, const T& b) const {
        auto size1 = a.size();
        auto size2 = b.size();
        CHECK(size1 == size2);

        double t0 = 0.0, t1 = 0.0, t2 = 0.0;
        for (decltype(size1) i = 0; i < size1; ++i) {
            t0 += static_cast<double>(a[i]) * b[i];
            t1 += static_cast<double>(a[i]) * a[i];
            t2 += static_cast<double>(b[i]) * b[i];
        }

        return 1.0 - t0 / (t1 + t2 - t0);
    }
};

} // namespace metric
} // namespace cl

#endif // CODELIBRARY_UTIL_METRIC_TANIMOTO_H_
