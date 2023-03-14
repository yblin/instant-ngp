//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_SET_TEST_H_
#define CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_SET_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/util/interval/interval_set.h"

namespace cl {
namespace test {

TEST(IntervalSetTest, InsertTest) {
    // [1 3) + [3 5) -> [1 5).
    Interval<int> a1(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b1(3, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    IntervalSet<int> s1;
    s1.Insert(a1);
    s1.Insert(b1);
    Array<Interval<int>> r1;
    r1.emplace_back(1, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    ASSERT_EQ_RANGE(s1.begin(), s1.end(), r1.begin(), r1.end());

    // [1 3) + (3 5] -> [1 3) (3 5]
    Interval<int> a2(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b2(3, 5, Interval<int>::OPEN, Interval<int>::CLOSED);
    IntervalSet<int> s2;
    s2.Insert(a2);
    s2.Insert(b2);
    Array<Interval<int>> r2;
    r2.emplace_back(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    r2.emplace_back(3, 5, Interval<int>::OPEN, Interval<int>::CLOSED);
    ASSERT_EQ_RANGE(s2.begin(), s2.end(), r2.begin(), r2.end());

    // [1 3) (3 5] + [3 5] -> [1 5]
    Interval<int> b3(3, 5, Interval<int>::CLOSED, Interval<int>::CLOSED);
    s2.Insert(b3);
    Array<Interval<int>> r3;
    r3.emplace_back(1, 5, Interval<int>::CLOSED, Interval<int>::CLOSED);
    ASSERT_EQ_RANGE(s2.begin(), s2.end(), r3.begin(), r3.end());

    // [1 3] [4 6] + [2 5] -> [1 6].
    Interval<int> a4(1, 3, Interval<int>::CLOSED, Interval<int>::CLOSED);
    Interval<int> b4(4, 6, Interval<int>::CLOSED, Interval<int>::CLOSED);
    Interval<int> c4(2, 5, Interval<int>::CLOSED, Interval<int>::CLOSED);
    IntervalSet<int> s4;
    s4.Insert(a4);
    s4.Insert(b4);
    s4.Insert(c4);
    Array<Interval<int>> r4;
    r4.emplace_back(1, 6, Interval<int>::CLOSED, Interval<int>::CLOSED);
    ASSERT_EQ_RANGE(s4.begin(), s4.end(), r4.begin(), r4.end());
}

TEST(IntervalSetTest, EraseTest) {
    // [1 3) [4 5) - [2 4] -> [1 2) (4 5).
    Interval<int> a1(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b1(4, 5, Interval<int>::CLOSED, Interval<int>::OPEN);
    IntervalSet<int> s1;
    s1.Insert(a1);
    s1.Insert(b1);
    Interval<int> c1(2, 4, Interval<int>::CLOSED, Interval<int>::CLOSED);
    s1.Erase(c1);
    Array<Interval<int>> r1;
    r1.emplace_back(1, 2, Interval<int>::CLOSED, Interval<int>::OPEN);
    r1.emplace_back(4, 5, Interval<int>::OPEN, Interval<int>::OPEN);
    ASSERT_EQ_RANGE(s1.begin(), s1.end(), r1.begin(), r1.end());

    // [1 3) - [1 3) -> empty.
    Interval<int> a2(1, 3, Interval<int>::CLOSED, Interval<int>::OPEN);
    IntervalSet<int> s2;
    s2.Insert(a2);
    s2.Erase(a2);
    Array<Interval<int>> r2;
    ASSERT_EQ_RANGE(s2.begin(), s2.end(), r2.begin(), r2.end());

    // [1 4) - [0 5] -> empty.
    Interval<int> a3(1, 4, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b3(0, 5, Interval<int>::CLOSED, Interval<int>::CLOSED);
    IntervalSet<int> s3;
    s3.Insert(a3);
    s3.Erase(b3);
    Array<Interval<int>> r3;
    ASSERT_EQ_RANGE(s3.begin(), s3.end(), r3.begin(), r3.end());

    // [1 4) - [2 3]  -> [1 2) (3 4).
    Interval<int> a4(1, 4, Interval<int>::CLOSED, Interval<int>::OPEN);
    Interval<int> b4(2, 3, Interval<int>::CLOSED, Interval<int>::CLOSED);
    IntervalSet<int> s4;
    s4.Insert(a4);
    s4.Erase(b4);
    Array<Interval<int>> r4;
    r4.emplace_back(1, 2, Interval<int>::CLOSED, Interval<int>::OPEN);
    r4.emplace_back(3, 4, Interval<int>::OPEN, Interval<int>::OPEN);
    ASSERT_EQ_RANGE(s4.begin(), s4.end(), r4.begin(), r4.end());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_INTERVAL_INTERVAL_SET_TEST_H_
