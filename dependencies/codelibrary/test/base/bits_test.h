//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_BASE_BITS_TEST_H_
#define CODELIBRARY_TEST_BASE_BITS_TEST_H_

#include "codelibrary/base/bits.h"
#include "codelibrary/base/testing.h"

namespace cl {
namespace test {

TEST(BitsTest, Log2Floor) {
    ASSERT_EQ(bits::Log2Floor(1), 0);
    ASSERT_EQ(bits::Log2Floor(2), 1);
    ASSERT_EQ(bits::Log2Floor(3), 1);
    ASSERT_EQ(bits::Log2Floor(4), 2);

    for (int i = 3; i < 31; ++i) {
        unsigned int value = 1U << i;
        ASSERT_EQ(bits::Log2Floor(value), i);
        ASSERT_EQ(bits::Log2Floor(value + 1), i);
        ASSERT_EQ(bits::Log2Floor(value + 2), i);
        ASSERT_EQ(bits::Log2Floor(value - 1), i - 1);
        ASSERT_EQ(bits::Log2Floor(value - 2), i - 1);
    }

    ASSERT_EQ(bits::Log2Floor(0xffffffffU), 31);
    ASSERT_EQ(bits::Log2Floor(0xffffffffffffffffUL), 63);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_BASE_BITS_TEST_H_
