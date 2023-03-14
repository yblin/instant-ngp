//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_FACTOR_TEST_H_
#define CODELIBRARY_TEST_MATH_FACTOR_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/factor/factor.h"

namespace cl {
namespace test {

TEST(Factor, TestUint64Semiprimes) {
    Array<uint64_t> semiprimes = {
        3369738766071892021ULL,
        8831269065180497ULL,
        2843901546547359024ULL,
        6111061272747645669ULL,
        11554045868611683619ULL,
        6764921230558061729ULL,
        16870180535862877896ULL,
        3778974635503891117ULL,
        204667546124958269ULL,
        16927447722109721827ULL
    };

    for (const auto& n : semiprimes) {
        uint64_t res = Factor(n);
        ASSERT(res != 1 && res != n) << n;
        ASSERT_EQ(res * (n / res), n);
    }
}

/**
 * Test semiprimes via Pollard's rho at 10^12.
 */
TEST(Factor, TestPollardRho10_12) {
    const int bound = 1000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(1000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(1000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = cl::factor::PollardRho(n);
            ASSERT(factor != 1 && factor != n);
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes via one line factor at 10^12.
 */
TEST(Factor, TestOneLineFactor10_12) {
    const int bound = 1000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(1000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(1000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = cl::factor::OneLineFactor(n);
            ASSERT(factor != 1 && factor != n) << n;
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes via SQUFOF at 10^12.
 */
TEST(Factor, TestSQUFOF10_12) {
    const int bound = 1000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(1000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(1000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = factor::SQUFOF(n);
            ASSERT(factor != 1 && factor != n) << n;
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes via Pollard's rho at 10^18.
 */
TEST(Factor, TestPollardRho10_18) {
    const int bound = 100000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(100000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(100000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = cl::factor::PollardRho(n);
            ASSERT(factor != 1 && factor != n);
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes via SQUFOF at 10^18.
 */
TEST(Factor, TestSQUFOF10_18) {
    const int bound = 100000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(100000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(100000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = factor::SQUFOF(n);
            ASSERT(factor != 1 && factor != n) << n;
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes at 10^18.
 */
TEST(Factor, Test10_18) {
    const int bound = 100000100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(100000000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(100000000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            uint64_t n = uint64_t(*i1) * *i2;
            uint64_t factor = Factor(n);
            ASSERT(factor != 1 && factor != n) << n;
            ASSERT_EQ(factor * (n / factor), n);
        }
    }
}

/**
 * Test semiprimes: n = pqr.
 */
TEST(Factor, TestPQRPollardRho) {
    const int bound = 100100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(100000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(100000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            for (auto i3 = s.Find(100000); i3 != s.end(); ++i3) {
                if (!s.IsPrime(*i3)) continue;
                uint64_t n = uint64_t(*i1) * *i2 * *i3;
                uint64_t factor = factor::PollardRho(n);
                ASSERT(factor != 1 && factor != n) << n;
                ASSERT_EQ(factor * (n / factor), n);
            }
        }
    }
}

/**
 * Test semiprimes: n = pqr.
 */
TEST(Factor, TestPQRSQUFOF) {
    const int bound = 100100;
    prime::WheelSieve30 s(bound);

    for (auto i1 = s.Find(100000); i1 != s.end(); ++i1) {
        if (!s.IsPrime(*i1)) continue;
        for (auto i2 = s.Find(100000); i2 != s.end(); ++i2) {
            if (!s.IsPrime(*i2)) continue;
            for (auto i3 = s.Find(100000); i3 != s.end(); ++i3) {
                if (!s.IsPrime(*i3)) continue;
                uint64_t n = uint64_t(*i1) * *i2 * *i3;
                // We do not ASSERT the results, because the Squfof may fails.
                factor::SQUFOF(n);
            }
        }
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_FACTOR_TEST_H_
