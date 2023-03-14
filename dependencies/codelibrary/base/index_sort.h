//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_INDEX_SORT_H_
#define CODELIBRARY_BASE_INDEX_SORT_H_

#include <algorithm>
#include <numeric>

#include "codelibrary/base/array.h"

namespace cl {

/**
 * Get the sorted indices of [first, last)
 */
template <typename Iterator>
void IndexSort(Iterator first, Iterator last, Array<int>* indices) {
    CHECK(indices);

    auto n1 = std::distance(first, last);
    CHECK(n1 >= 0);
    CHECK(n1 <= INT_MAX);

    int n = static_cast<int>(n1);

    indices->resize(n);
    std::iota(indices->begin(), indices->end(), 0);

    std::sort(indices->begin(), indices->end(), [&](int a, int b) {
        return first[a] < first[b];
    });
}
template <typename Iterator, class Compare>
void IndexSort(Iterator first, Iterator last, Compare compare,
               Array<int>* indices) {
    CHECK(indices);

    auto n1 = std::distance(first, last);
    CHECK(n1 >= 0);
    CHECK(n1 <= INT_MAX);

    int n = static_cast<int>(n1);

    indices->resize(n);
    std::iota(indices->begin(), indices->end(), 0);

    std::sort(indices->begin(), indices->end(), [&](int a, int b) {
        return compare(first[a], first[b]);
    });
}

} // namespace cl

#endif // CODELIBRARY_BASE_INDEX_SORT_H_
