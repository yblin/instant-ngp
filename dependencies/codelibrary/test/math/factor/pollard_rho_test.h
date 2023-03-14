//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_FACTOR_POLLARD_RHO_TEST_H_
#define CODELIBRARY_TEST_MATH_FACTOR_POLLARD_RHO_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/math/factor/pollard_rho.h"
#include "codelibrary/math/prime/is_prime.h"

namespace cl {
namespace test {

TEST(PollarRhoTest, TestUint64Semiprimes) {
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
        uint64_t res = factor::PollardRho(n);
        ASSERT(res != 1 && res != n) << n;
        ASSERT_EQ(res * (n / res), n);
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_FACTOR_POLLARD_RHO_TEST_H_
