//
// Copyright 2018-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_LIST_INDEXED_LIST_TEST_H_
#define CODELIBRARY_TEST_UTIL_LIST_INDEXED_LIST_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/list/indexed_list.h"

namespace cl {
namespace test {

TEST(IndexedListTest, Test) {
    struct BaseNode {};

    IndexedList<BaseNode> list;

    using Node = typename IndexedList<BaseNode>::Node;
    Node* n0 = list.Allocate();
    Node* n1 = list.Allocate();
    Node* n2 = list.Allocate();
    Node* n3 = list.Allocate();
    Node* n4 = list.Allocate();
    Node* n5 = list.Allocate();
    Array<Node*> res = {n0, n1, n2, n3, n4, n5};
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    list.Deallocate(n3);
    res = {n0, n1, n2, n5, n4};
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    res = {n0, n1, n2, n5};
    list.Deallocate(n4);
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    res = {n0, n1, n2, n5, n4};
    list.Allocate();
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    res = {n0, n1, n2, n5, n4, n3};
    list.Allocate();
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    Node* n6 = list.Allocate();
    Node* n7 = list.Allocate();
    Node* n8 = list.Allocate();
    res = {n0, n1, n2, n5, n4, n3, n6, n7, n8};
    ASSERT_EQ_RANGE(res.begin(), res.end(), list.begin(), list.end());

    IndexedList<BaseNode> list1;
    list.Clone(&list1);
    ASSERT_EQ(list.n_allocated(), list1.n_allocated());
    ASSERT_EQ(list.n_available(), list1.n_available());
    auto i = list.begin();
    auto i1 = list1.begin();
    for (; i != list.end(); ++i, ++i1) {
        ASSERT_EQ(i->id(), i1->id());
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_LIST_INDEXED_LIST_TEST_H_
