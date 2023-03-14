//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_BASE_FLOAT_TEST_H_
#define CODELIBRARY_TEST_BASE_FLOAT_TEST_H_

#include "codelibrary/base/float.h"
#include "codelibrary/base/testing.h"

namespace cl {
namespace test {

TEST(FloatTest, TestDecompose) {
    Float<double> x1(1.0);
    ASSERT_EQ(x1.mantissa(), 0LL);
    ASSERT_EQ(x1.exponent(), 0x3FF);

    Float<double> x2(2.0);
    ASSERT_EQ(x2.mantissa(), 0LL);
    ASSERT_EQ(x2.exponent(), 0x400);

    Float<double> x23(23.0);
    ASSERT_EQ(x23.mantissa(), 0x7000000000000LL);
    ASSERT_EQ(x23.exponent(), 0x403);

    Float<double> x(0.01171875);
    ASSERT_EQ(x.mantissa(), 0x8000000000000LL);
    ASSERT_EQ(x.exponent(), 0x3F8);
}

TEST(FloatTest, TestInf) {
    Float<double> x1(std::numeric_limits<double>::infinity());
    ASSERT(x1.is_inf());
    Float<double> x2(-std::numeric_limits<double>::infinity());
    ASSERT(x2.is_inf());

    Float<float> x3(std::numeric_limits<float>::infinity());
    ASSERT(x3.is_inf());
    Float<float> x4(-std::numeric_limits<float>::infinity());
    ASSERT(x4.is_inf());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_BASE_BITS_TEST_H_
