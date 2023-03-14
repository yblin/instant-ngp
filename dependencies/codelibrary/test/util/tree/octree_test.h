//
// Copyright 2014 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_TREE_OCTREE_TEST_H_
#define CODELIBRARY_TEST_UTIL_TREE_OCTREE_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/tree/octree.h"

namespace cl {
namespace test {

TEST(OctreeTest, Insert) {
    Octree<int> octree(2, 2, 2);

    octree(0, 0, 0) = 1;
    octree(0, 0, 1) = 2;
    octree(0, 1, 0) = 3;
    octree(0, 1, 1) = 4;
    octree(1, 0, 0) = 5;
    octree(1, 0, 1) = 6;
    octree(1, 1, 0) = 7;
    octree(1, 1, 1) = 8;

    ASSERT_EQ(octree(0, 0, 0), 1);
    ASSERT_EQ(octree(0, 0, 1), 2);
    ASSERT_EQ(octree(0, 1, 0), 3);
    ASSERT_EQ(octree(0, 1, 1), 4);
    ASSERT_EQ(octree(1, 0, 0), 5);
    ASSERT_EQ(octree(1, 0, 1), 6);
    ASSERT_EQ(octree(1, 1, 0), 7);
    ASSERT_EQ(octree(1, 1, 1), 8);

    Octree<bool> octree1(1, 1, 1);
    octree1(0, 0, 0) = true;
    ASSERT(octree1(0, 0, 0));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_TREE_OCTREE_TEST_H_
