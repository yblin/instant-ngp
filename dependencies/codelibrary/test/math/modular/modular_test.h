//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_MODULAR_MODULAR_TEST_H_
#define CODELIBRARY_TEST_MATH_MODULAR_MODULAR_TEST_H_

#include <cstdint>

#include "codelibrary/base/testing.h"
#include "codelibrary/math/modular/modular.h"

namespace cl {
namespace test {

TEST(ModularTest, ModAdd) {
    ASSERT_EQ(ModAdd(INT_MAX - 1, INT_MAX - 1, INT_MAX), INT_MAX - 2);
    ASSERT_EQ(ModAdd(UINT_MAX - 1, UINT_MAX - 1, UINT_MAX), UINT_MAX - 2);
}

TEST(ModularTest, ModMul) {
    ASSERT_EQ(ModMul(UINT64_MAX, UINT64_MAX, UINT64_MAX - 1), 1ULL);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_NUMBER_MODULAR_MODULAR_TEST_H_
