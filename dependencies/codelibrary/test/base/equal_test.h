//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_BASE_EQUAL_TEST_H_
#define CODELIBRARY_TEST_BASE_EQUAL_TEST_H_

#include "codelibrary/base/equal.h"
#include "codelibrary/base/testing.h"

namespace cl {
namespace test {

TEST(EqualTest, IntEqual) {
    ASSERT(Equal(1, 1));
}

TEST(EqualTest, FloatEqual) {
    ASSERT(1.0f != 0.9999999f);
    ASSERT(Equal(1.0f, 0.9999999f));
}

TEST(EqualTest, DoubleEqual) {
    ASSERT(1.0 != 0.9999999999999996);
    ASSERT(Equal(1.0, 0.9999999999999996));
    ASSERT_FALSE(Equal(1.0, 0.9999999999999995));
}

TEST(EqualTest, ULPEqual) {
    ASSERT_FALSE(ULPEqual(std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity()));
    ASSERT_FALSE(ULPEqual(std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::quiet_NaN()));

    ASSERT(ULPEqual(1.0f, 0.9999999f));
    ASSERT(ULPEqual(1.0, 0.9999999999999996));
    ASSERT_FALSE(ULPEqual(1.0, 0.9999999999999995));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_BASE_EQUAL_TEST_H_
