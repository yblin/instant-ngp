//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_MATH_PRIME_IS_PRIME_TEST_H_
#define CODELIBRARY_TEST_MATH_PRIME_IS_PRIME_TEST_H_

#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/math/prime/is_prime.h"

namespace cl {
namespace test {

TEST(IsPrimeTest, TestSmallNumbers) {
    ASSERT_FALSE(IsPrime(0));
    ASSERT_FALSE(IsPrime(1));

    for (int i = 2; i <= 100000; ++i) {
        int j = 2;
        for (j = 2; j * j <= i; ++j) {
            if (i % j == 0) break;
        }
        bool t = IsPrime(i);
        if (j * j > i) {
            ASSERT(t) << " " << i << " is a prime.";
        } else {
            ASSERT_FALSE(t) << " " << i << " is not a prime.";
        }
    }
}

TEST(IsProbablePrimeTest, TestSmallNumbers) {
    for (int i = 2; i <= 100000; ++i) {
        int j = 2;
        for (j = 2; j * j <= i; ++j) {
            if (i % j == 0) break;
        }
        bool t = IsProbablePrime(BigInt(i));
        if (j * j > i) {
            ASSERT(t) << " " << i << " is a prime.";
        } else {
            ASSERT_FALSE(t) << " " << i << " is not a prime.";
        }
    }
}

TEST(IsProbablePrimeTest, TestLargeNumbers) {
    Array<std::string> primes = {
        // Bell number primes.
        "27644437",
        "35742549198872617291353508656626642567",
        "359334085968622831041960188598043661065388726959079837",

        // Carol primes.
        "1046527",
        "16769023",
        "1073676287",
        "68718952447",
        "274876858367",
        "4398042316799",
        "1125899839733759",
        "18014398241046527",
        "1298074214633706835075030044377087",

        // Mersenne primes.
        "2305843009213693951",
        "618970019642690137449562111",
        "162259276829213363391578010288127",
        "170141183460469231731687303715884105727",

        // Woodall primes.
        "32212254719",
        "2833419889721787128217599",
        "195845982777569926302400511",
        "4776913109852041418248056622882488319"
    };

    for (const auto & prime : primes) {
        ASSERT(IsProbablePrime(BigInt(prime))) << " " << prime
                                                    << " is a prime.";
    }

    for (int i = 0; i < primes.size(); ++i) {
        for (int j = i + 1; j < primes.size(); ++j) {
            BigInt a(primes[i]), b(primes[j]);
            ASSERT_FALSE(IsProbablePrime(a * b)) << " " << a * b
                                                 << " is not a prime.";
        }
    }
}

TEST(IsMersennePrime, TrueCases) {
    ASSERT(IsMersennePrime(2));
    ASSERT(IsMersennePrime(3));
    ASSERT(IsMersennePrime(5));
    ASSERT(IsMersennePrime(7));
    ASSERT(IsMersennePrime(13));
    ASSERT(IsMersennePrime(17));
    ASSERT(IsMersennePrime(19));
    ASSERT(IsMersennePrime(31));
    ASSERT(IsMersennePrime(61));
    ASSERT(IsMersennePrime(89));
    ASSERT(IsMersennePrime(107));
    ASSERT(IsMersennePrime(127));
    ASSERT(IsMersennePrime(521));
    ASSERT(IsMersennePrime(607));
    ASSERT(IsMersennePrime(1279));
    ASSERT(IsMersennePrime(2203));
    ASSERT(IsMersennePrime(2281));
}

TEST(IsMersennePrime, FalseCases) {
    ASSERT_FALSE(IsMersennePrime(4));
    ASSERT_FALSE(IsMersennePrime(6));
    ASSERT_FALSE(IsMersennePrime(8));
    ASSERT_FALSE(IsMersennePrime(9));
    ASSERT_FALSE(IsMersennePrime(10));
    ASSERT_FALSE(IsMersennePrime(11));
    ASSERT_FALSE(IsMersennePrime(12));
    ASSERT_FALSE(IsMersennePrime(14));
    ASSERT_FALSE(IsMersennePrime(15));
    ASSERT_FALSE(IsMersennePrime(16));
    ASSERT_FALSE(IsMersennePrime(1001));
    ASSERT_FALSE(IsMersennePrime(10001));
    ASSERT_FALSE(IsMersennePrime(100001));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_MATH_PRIME_IS_PRIME_TEST_H_
