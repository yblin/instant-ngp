//
// Copyright 2016 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_TREE_CARTESIAN_TREE_TEST_H_
#define CODELIBRARY_TEST_UTIL_TREE_CARTESIAN_TREE_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/tree/cartesian_tree.h"

namespace cl {
namespace test {

TEST(CartesianTest, Test) {
    Array<int> a = {5, 1, 3, 2, 4};
    CartesianTree<int> cartesian_tree(a.begin(), a.end());
    Array<int> parents = cartesian_tree.parents();
    Array<int> result = {1, -1, 3, 1, 3};
    CHECK_EQ_RANGE(parents.begin(), parents.end(),
                    result.begin(), result.end());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_TREE_CARTESIAN_TREE_TEST_H_
