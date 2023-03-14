//
// Copyright 2016 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_TREE_QUAD_TREE_TEST_H_
#define CODELIBRARY_TEST_UTIL_TREE_QUAD_TREE_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/tree/quad_tree.h"

namespace cl {
namespace test {

TEST(QuadtreeTest, Insert) {
    Quadtree<int> quad_tree(2, 2);

    quad_tree(0, 0) = 1;
    quad_tree(0, 1) = 2;
    quad_tree(1, 0) = 3;
    quad_tree(1, 1) = 4;

    ASSERT_EQ(quad_tree(0, 0), 1);
    ASSERT_EQ(quad_tree(0, 1), 2);
    ASSERT_EQ(quad_tree(1, 0), 3);
    ASSERT_EQ(quad_tree(1, 1), 4);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_TREE_QUAD_TREE_TEST_H_
