//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_DECIMAL_TEST_H_
#define CODELIBRARY_TEST_MATH_DECIMAL_TEST_H_

#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/math/number/decimal.h"

namespace cl {
namespace test {

TEST(DecimalTest, IO) {
    using D = Decimal<10>;

    ASSERT_EQ(D("0").ToString(), std::string("0"));
    ASSERT_EQ(D("1").ToString(), std::string("1"));
    ASSERT_EQ(D("1.0").ToString(), std::string("1"));
    ASSERT_EQ(D("1.00").ToString(), std::string("1"));
    ASSERT_EQ(D("10").ToString(), std::string("10"));
    ASSERT_EQ(D("1000").ToString(), std::string("1000"));
    ASSERT_EQ(D("10.00000000000").ToString(), std::string("10"));
    ASSERT_EQ(D("-0.0").ToString(), std::string("0"));
    ASSERT_EQ(D("-00.00").ToString(), std::string("0"));
    ASSERT_EQ(D("10.00e12").ToString(), std::string("1e+13"));
    ASSERT_EQ(D("12345.6789").ToString(), std::string("12345.6789"));
    ASSERT_EQ(D("12345.00000").ToString(), std::string("12345"));
    ASSERT_EQ(D("123456789").ToString(), std::string("123456789"));
    ASSERT_EQ(D("0.0000012345").ToString(), std::string("0.0000012345"));
    ASSERT_EQ(D("0.00000012345").ToString(), std::string("1.2345e-7"));
    ASSERT_EQ(D("+12.76").ToString(), std::string("12.76"));
    ASSERT_EQ(D("012.76").ToString(), std::string("12.76"));
    ASSERT_EQ(D("+0.003").ToString(), std::string("0.003"));
    ASSERT_EQ(D("5E-6").ToString(), std::string("0.000005"));
    ASSERT_EQ(D("50E-7").ToString(), std::string("0.000005"));
    ASSERT_EQ(D(".12345678901").ToString(), std::string("0.123456789"));
    ASSERT_EQ(D(".00000000001").ToString(), std::string("1e-11"));
    ASSERT_EQ(D("1E0011").ToString(), std::string("1e+11"));
    ASSERT_EQ(D("-inf").ToString(), std::string("-inf"));
    ASSERT_EQ(D("inf").ToString(), std::string("inf"));
    ASSERT_EQ(D("nan").ToString(), std::string("nan"));
    ASSERT_EQ(D("-99e-9999999999").ToString(), std::string("0"));
    ASSERT_EQ(D("-99e-999999999").ToString(), std::string("-9.9e-999999998"));
}

TEST(DecimalTest, Compare) {
    using D = Decimal<10>;

    ASSERT_FALSE(D("nan") > D(0));
    ASSERT_FALSE(D("nan") < D(0));
    ASSERT_FALSE(D("nan") < D("inf"));
    ASSERT_FALSE(D("nan") < D("-inf"));
    ASSERT(D("inf") > D("-inf"));
    ASSERT(D("inf") > D("1e123231"));
    ASSERT(D("inf") == D("inf"));
    ASSERT(D("-inf") == D("-inf"));
    ASSERT(D("inf") != D("-inf"));
    ASSERT(D("inf") != D("nan"));

    ASSERT(D(12) > D(11));
    ASSERT(D("1e-12") < D("1e-11"));
    ASSERT(D("0.33333333") != D("0.3333"));
}

TEST(DecimalTest, Add) {
    using D = Decimal<10>;

    ASSERT_EQ(D("1") + D("1"), D("2"));
    ASSERT_EQ(D("2") + D("3"), D("5"));
    ASSERT_EQ(D("5.75") + D("3.3"), D("9.05"));
    ASSERT_EQ(D("5") + D("-3"), D("2"));
    ASSERT_EQ(D("-7") + D("2.5"), D("-4.5"));
    ASSERT_EQ(D("0.7") + D("0.3"), D("1.0"));
    ASSERT_EQ(D("1.25") + D("1.25"), D("2.50"));
    ASSERT_EQ(D("1.23456789") + D("1.0000000"), D("2.23456789"));
    ASSERT_EQ(D("1.23456789") + D("1.00000011"), D("2.23456800"));
    ASSERT_EQ(D(1) + D(1) / D(3), D("1.3333333333"));
}

TEST(DecimalTest, Divide) {
    using D = Decimal<9>;

    ASSERT_EQ(D("1") / D("1"), D("1"));
    ASSERT_EQ(D("2") / D("1"), D("2"));
    ASSERT_EQ(D("1") / D("2"), D("0.5"));
    ASSERT_EQ(D("2") / D("2"), D("1"));
    ASSERT_EQ(D("0") / D("2"), D("0"));

    ASSERT_EQ((D(1) / D(3)).ToString(), std::string("0.333333333"));
    ASSERT_EQ((D(2) / D(3)).ToString(), std::string("0.666666666"));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_NUMBER_DECIMAL_TEST_H_
