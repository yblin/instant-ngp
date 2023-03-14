//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_NUMBER_BIGINT_TEST_H_
#define CODELIBRARY_TEST_MATH_NUMBER_BIGINT_TEST_H_

#include <limits>
#include <random>
#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/base/timer.h"
#include "codelibrary/math/number/bigint.h"

namespace cl {
namespace test {

class BigIntTest : public Test {
protected:
    BigIntTest() = default;

    // Generate binary string with the given length.
    std::string GenerateBinaryString(int n) const {
        std::string binary;
        for (int i = 0; i < n - 1; ++i)
            binary += static_cast<char>((random_engine_() & 0x01U) + '0');
        return "1" + binary;
    }

    // Get the sum of two binary strings with the same length.
    std::string GetSum(const std::string& a, const std::string& b) const {
        CHECK(a.size() == b.size());

        std::string c(a.size(), '0');
        bool carry = false;

        for (size_t t = a.size(); t > 0; --t) {
            size_t i = t - 1;
            if (a[i] == '1' && b[i] == '1') {
                c[i] = carry ? '1' : '0';
                carry = true;
            } else if ((a[i] == '1' && b[i] == '0') ||
                       (a[i] == '0' && b[i] == '1')) {
                if (carry) {
                    c[i] = '0';
                } else {
                    c[i] = '1';
                }
            } else {
                c[i] = carry ? '1' : '0';
                carry = false;
            }
        }

        return carry ? "1" + c : c;
    }

    // Get the production of two binary strings with the same length.
    std::string GetProduct(const std::string& a, const std::string& b) const {
        CHECK(a.size() == b.size());

        std::string c(a.size() + b.size(), '0');
        for (size_t t = a.size(); t > 0; --t) {
            size_t i = t - 1;
            std::string t1(a.size() + b.size(), '0');
            if (b[i] == '1') {
                for (size_t j = 0; j < a.size(); ++j) {
                    t1[i + j + 1] = a[j];
                }

                c = GetSum(c, t1);
            }
        }

        // Remove the leading zero.
        size_t start = 0;
        while (start < c.size() && c[start] == '0') {
            ++start;
        }

        if (start == c.size()) return "0";

        return c.substr(start);
    }

    // Test the multiply performance.
    void MultiplyPerformance(int bits1, int bits2) const {
        std::string s1 = GenerateBinaryString(bits1);
        std::string s2 = GenerateBinaryString(bits2);

        BigInt a(s1, 2), b(s2, 2), c;
        c = a * b;
    }

    // Test the divide performance.
    void DividePerformance(int bits1, int bits2) const {
        std::string s1 = GenerateBinaryString(bits1);
        std::string s2 = GenerateBinaryString(bits2);

        BigInt a(s1, 2), b(s2, 2), c;
        c = a / b;
    }

    mutable std::mt19937 random_engine_;
};

TEST_F(BigIntTest, Constructor) {
    BigInt a(-10), b(-10);
    ASSERT_EQ(a, BigInt("-10"));
    ASSERT_EQ(a, BigInt("-1010", 2));
    ASSERT_EQ(a, BigInt("-a", 16));
    ASSERT_EQ(a, BigInt("-010"));
    ASSERT_EQ(a, b);

    ASSERT_EQ(BigInt(0xffffffffU), BigInt("4294967295"));
    ASSERT_EQ(BigInt(std::numeric_limits<int32_t>::min()),
              BigInt("-2147483648"));
    ASSERT_EQ(BigInt(0xffffffffffffffffUL), BigInt("18446744073709551615"));
    ASSERT_EQ(BigInt(std::numeric_limits<int64_t>::min()),
              BigInt("-9223372036854775808"));
}

TEST_F(BigIntTest, Add) {
    for (int i = 0; i < 100; ++i) {
        std::string s1 = GenerateBinaryString(128);
        std::string s2 = GenerateBinaryString(128);
        std::string s3 = GetSum(s1, s2);

        BigInt a(s1, 2), b(s2, 2);
        BigInt c = a + b;
        ASSERT_EQ(c.ToString(2), s3) << "a: " << a << "\n"
                                    << "b: " << b << "\n";
    }
}

TEST_F(BigIntTest, Substract) {
    for (int i = 0; i < 100; ++i) {
        std::string s1 = GenerateBinaryString(128);
        std::string s2 = GenerateBinaryString(128);

        BigInt a(s1, 2), b(s2, 2);
        BigInt c = a - b;
        ASSERT_EQ(c + b, a) << "a: " << a << "\n"
                           << "b: " << b << "\n"
                           << "c: " << c << "\n";
    }
}

TEST_F(BigIntTest, Multiply) {
    for (int i = 0; i < 100; ++i) {
        std::string s1 = GenerateBinaryString(128);
        std::string s2 = GenerateBinaryString(128);
        std::string s3 = GetProduct(s1, s2);

        BigInt a(s1, 2), b(s2, 2);
        BigInt c = a * b;
        ASSERT_EQ(c.ToString(2), s3) << "a: " << a << "\n"
                                    << "b: " << b << "\n"
                                    << "c: " << c << "\n";
    }
}

TEST_F(BigIntTest, Divide) {
    for (int i = 0; i < 100; ++i) {
        std::string s1 = GenerateBinaryString(128);
        std::string s2 = GenerateBinaryString(64);

        BigInt a(s1, 2), b(s2, 2);
        BigInt c = a / b;
        BigInt d = a % b;

        ASSERT_EQ(c * b + d, a);
        ASSERT_EQ(c * b + d, a) << "a: " << a << "\n"
                               << "b: " << b << "\n"
                               << "c: " << c << "\n"
                               << "d: " << d << "\n";
    }
}

TEST_F(BigIntTest, Sqrt) {
    for (int i = 0; i < 100; ++i) {
        std::string s = GenerateBinaryString(128);

        BigInt a(s, 2);
        BigInt b = a.Sqrt();

        ASSERT(b * b <= a && (b + 1) * (b + 1) > a) << "a: " << a << "\n"
                                                   << "b: " << b << "\n";
    }
}

TEST_F(BigIntTest, MultiplyPerformance) {
    const int n_test = 10;
    int n1[]    = { 1, 2, 4, 8, 16, 20, 20, 20, 20, 20 };
    int n2[]    = { 1, 2, 4, 8, 16, 10, 16, 18, 19, 20 };
    int tests[] = { 100000, 100000, 100000, 10000, 10, 10, 1, 1, 1, 1 };

    printf("\n");
    printf("  Bits of a   Bits of b     Time\n");
    printf("----------------------------------\n");

    for (int i = 0; i < n_test; ++i) {
        std::string s1 = GenerateBinaryString(1 << n1[i]);
        std::string s2 = GenerateBinaryString(1 << n2[i]);

        BigInt a(s1, 2), b(s2, 2), c;

        Timer timer;
        timer.Start();
        for (int j = 0; j < tests[i]; ++j) {
            c = a * b;
        }
        timer.Stop();

        printf("%9d %11d %11s\n", (1 << n1[i]), (1 << n2[i]),
               timer.average_time(tests[i]).c_str());
    }
    printf("----------------------------------\n");
    printf("\n");
}

TEST_F(BigIntTest, DividePerformance) {
    const int n_test = 9;
    int n1[]    = { 17, 17, 17, 17, 17, 17, 17, 17, 17 };
    int n2[]    = {  1,  2,  4,  8, 10, 12, 14, 15, 16 };
    int tests[] = { 10000, 1000, 1000, 5, 5, 5, 5, 5, 5 };

    printf("\n");
    printf("  Bits of a   Bits of b     Time\n");
    printf("----------------------------------\n");

    for (int i = 0; i < n_test; ++i) {
        std::string s1 = GenerateBinaryString(1 << n1[i]);
        std::string s2 = GenerateBinaryString(1 << n2[i]);

        BigInt a(s1, 2), b(s2, 2), c;

        Timer timer;
        timer.Start();
        for (int j = 0; j < tests[i]; ++j) {
            c = a / b;
        }
        timer.Stop();

        printf("%9d %11d %11s\n", (1 << n1[i]), (1 << n2[i]),
               timer.average_time(tests[i]).c_str());
    }
    printf("----------------------------------\n");
    printf("\n");
}

TEST_F(BigIntTest, SqrtPerformance) {
    const int n_test = 4;
    int n[]     = {   10,  12, 14, 16 };
    int tests[] = { 1000, 100, 5, 5 };

    printf("\n");
    printf("  Bits of n       Time\n");
    printf("------------------------\n");

    for (int i = 0; i < n_test; ++i) {
        std::string s = GenerateBinaryString(1 << n[i]);

        BigInt a(s, 2);

        Timer timer;
        timer.Start();
        for (int j = 0; j < tests[i]; ++j) {
            a.Sqrt();
        }
        timer.Stop();

        printf("%9d %13s\n", (1 << n[i]), timer.average_time(tests[i]).c_str());
    }
    printf("------------------------\n");
    printf("\n");
}

TEST_F(BigIntTest, AssignFromDecimalPerformance_100000) {
    std::string decimal(100000, '1');
    BigInt a;
    a = decimal;
}

TEST_F(BigIntTest, ToDecimalPerformance_100000) {
    BigInt a(2);
    a <<= 332190;
    a.ToString(10);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_NUMBER_BIGINT_TEST_H_
