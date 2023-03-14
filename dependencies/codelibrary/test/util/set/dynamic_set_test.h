//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_SET_DYNAMIC_SET_TEST_H_
#define CODELIBRARY_TEST_UTIL_SET_DYNAMIC_SET_TEST_H_

#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/util/set/dynamic_bitset.h"

namespace cl {
namespace test {

TEST(DynamicBitsetTest, Constructor) {
    DynamicBitset a(16);
    DynamicBitset b(16, 0xfa2);
    DynamicBitset c("0101111001");

    std::string str1 = "0000000000000000";
    std::string str2 = "0000111110100010";
    std::string str3 = "0101111001";

    ASSERT_EQ(a.ToString(), str1);
    ASSERT_EQ(b.ToString(), str2);
    ASSERT_EQ(c.ToString(), str3);
}

TEST(DynamicBitsetTest, TestBit) {
    DynamicBitset a("01011");

    ASSERT_EQ(a.Test(0), true);
    ASSERT_EQ(a.Test(1), true);
    ASSERT_EQ(a.Test(2), false);
    ASSERT_EQ(a.Test(3), true);
    ASSERT_EQ(a.Test(4), false);
}

TEST(DynamicBitsetTest, TestOperators) {
    DynamicBitset foo("1001");
    DynamicBitset bar("0011");

    foo ^= bar;
    ASSERT_EQ(foo.ToString(), "1010");

    foo &= bar;
    ASSERT_EQ(foo.ToString(), "0010");

    foo |= bar;
    ASSERT_EQ(foo.ToString(), "0011");

    foo <<= 2;
    ASSERT_EQ(foo.ToString(), "1100");

    foo >>= 1;
    ASSERT_EQ(foo.ToString(), "0110");

    ASSERT_EQ((~bar).ToString(), "1100");
    ASSERT_EQ((bar << 1).ToString(), "0110");
    ASSERT_EQ((bar >> 1).ToString(), "0001");

    ASSERT_FALSE(foo == bar);
    ASSERT(foo != bar);

    ASSERT_EQ((foo & bar).ToString(), "0010");
    ASSERT_EQ((foo | bar).ToString(), "0111");
    ASSERT_EQ((foo ^ bar).ToString(), "0101");
}

TEST(DynamicBitsetTest, TestFlip) {
    DynamicBitset foo("0001");

    ASSERT_EQ(foo.Flip(2).ToString(), "0101");
    ASSERT_EQ(foo.Flip().ToString(), "1010");
}

TEST(DynamicBitsetTest, TestReset) {
    DynamicBitset foo("1011");

    ASSERT_EQ(foo.Reset(1).ToString(), "1001");
    ASSERT_EQ(foo.Reset().ToString(), "0000");
}

TEST(DynamicBitsetTest, TestSet) {
    DynamicBitset foo(4);

    ASSERT_EQ(foo.Set().ToString(), "1111");
    ASSERT_EQ(foo.Set(2, 0).ToString(), "1011");
    ASSERT_EQ(foo.Set(2).ToString(), "1111");
}

TEST(DynamicBitsetTest, TestCount) {
    DynamicBitset foo1("10110011");
    DynamicBitset foo2("0000000000000000000000000000000110110011");
    DynamicBitset foo3("0000000000000000111100000000000110110011");

    ASSERT_EQ(foo1.Count(), 5);
    ASSERT_EQ(foo2.Count(), 6);
    ASSERT_EQ(foo3.Count(), 10);
}

TEST(DynamicBitsetTest, TestAccess) {
    DynamicBitset foo(4);

    foo[1] = 1;
    ASSERT_EQ(foo.ToString(), "0010");

    foo[2] = foo[1];
    ASSERT_EQ(foo.ToString(), "0110");
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_SET_DYNAMIC_SET_TEST_H_
