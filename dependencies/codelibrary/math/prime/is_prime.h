//
// Copyright 2015-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_PRIME_IS_PRIME_H_
#define CODELIBRARY_MATH_PRIME_IS_PRIME_H_

#include <climits>
#include <cstdint>
#include <limits>

#include "codelibrary/base/array.h"
#include "codelibrary/math/number/bigint.h"
#include "codelibrary/math/factor/trial_division.h"
#include "codelibrary/math/prime/miller_rabin_test.h"

namespace cl {

/**
 * Return true if 'n' is a small prime (up to 4093).
 */
inline bool IsSmallPrime(uint32_t n) {
    if (n == 2) return true;
    if (n == 0 || n == 1 || n % 2 == 0) return false;

    static uint32_t ODD_PRIME_LOOKUP[] = {
        0x64b4cb6eUL, 0x816d129aUL, 0x864a4c32UL, 0x2196820dUL,
        0x5a0434c9UL, 0xa4896120UL, 0x29861144UL, 0x4a2882d1UL,
        0x32424030UL, 0x8349921UL,  0x4225064bUL, 0x148a4884UL,
        0x6c304205UL, 0xb40b408UL,  0x125108a0UL, 0x65048928UL,
        0x804c3098UL, 0x80124496UL, 0x41124221UL, 0xc02104c9UL,
        0x982d32UL,   0x8044900UL,  0x82689681UL, 0x220825b0UL,
        0x40a28948UL, 0x90042659UL, 0x30434006UL, 0x69009244UL,
        0x8088210UL,  0x12410da4UL, 0x2400c060UL, 0x86122d2UL,
        0x821b0484UL, 0x110d301UL,  0xc044a002UL, 0x14916022UL,
        0x4a6400cUL,  0x92094d2UL,  0x522094UL,   0x4ca21008UL,
        0x51018200UL, 0xa48b0810UL, 0x44309a25UL, 0x34c1081UL,
        0x80522502UL, 0x20844908UL, 0x18003250UL, 0x241140a2UL,
        0x1840128UL,  0xa41a001UL,  0x36004512UL, 0x29260008UL,
        0xc0618283UL, 0x10100480UL, 0x4822006dUL, 0xc20c2658UL,
        0x24894810UL, 0x45205820UL, 0x19002488UL, 0x10c02502UL,
        0x1140868UL,  0x802832caUL, 0x264b0400UL, 0x60901300UL
    };

    const int bits = 32;
    uint32_t q = n / 2;
    uint32_t x = (q & (bits - 1));
    return (ODD_PRIME_LOOKUP[q / bits] & (1UL << x)) >> x;
}

/**
 * Test if an 32/64-bit integer is a prime.
 *
 * We adopt trial division prime test for the small integers, and adopt Miller
 * Rabin test for big integers.
 *
 * It had been proved that, we only need to test the following witnesses:
 * if n < 2^32, it is enough to test {2, 7, 61}. （found by Gerhard Jaeschke)
 * if n < 2^64, it is enough to test {2, 325, 9375, 28178, 450775, 9780504,
 * 1795265022}. (found by Jim Sinclair)
 */
inline bool IsPrime(uint32_t n) {
    if (n <= 4093) return IsSmallPrime(n);

    if (factor::SmallFactor(n) != n) return false;
    // factor::SmallFactor(n) can test the prime under 211 * 211.
    if (n <= 211 * 211) return true;

    if (n < 316349281) {
        return prime::MillerRabinTest(n, {11000544, 31481107});
    }
    return prime::MillerRabinTest(n, {2, 7, 61});
}
inline bool IsPrime(int32_t n) {
    return IsPrime(static_cast<uint32_t>(n));
}

inline bool IsPrime(uint64_t n) {
    if (n <= UINT32_MAX) return IsPrime(static_cast<uint32_t>(n));
    if (factor::SmallFactor(n) != n) return false;

    if (n < 341531ULL) {
        return prime::MillerRabinTest(n, {9345883071009581737ULL});
    }

    if (n < 1050535501ULL) {
        return prime::MillerRabinTest(n, {336781006125ULL,
                                          9639812373923155ULL});
    }

    if (n < 350269456337ULL) {
        return prime::MillerRabinTest(n, {4230279247111683200ULL,
                                          14694767155120705706ULL,
                                          1664113952636775035ULL});
    }

    if (n < 55245642489451ULL) {
        return prime::MillerRabinTest(n, {2ULL,
                                          141889084524735ULL,
                                          1199124725622454117ULL,
                                          11096072698276303650ULL});
    }

    if (n < 7999252175582851ULL) {
        return prime::MillerRabinTest(n, {2ULL,
                                          4130806001517ULL,
                                          149795463772692060ULL,
                                          186635894390467037ULL,
                                          3967304179347715805ULL});
    }

    if (n < 585226005592931977ULL) {
        return prime::MillerRabinTest(n, {2ULL,
                                          123635709730000ULL,
                                          9233062284813009ULL,
                                          43835965440333360ULL,
                                          761179012939631437ULL,
                                          1263739024124850375ULL});
    }

    return prime::MillerRabinTest(n, {2, 325, 9375, 28178, 450775, 9780504,
                                      1795265022});
}
inline bool IsPrime(int64_t n) {
    return IsPrime(static_cast<uint64_t>(n));
}

/**
 * Test if an integer is a probable prime.
 *
 * This function uses the Miller-Rabin probabilistic primality test, the running
 * time should be somewhere around O(k(logn)^3) (k: the number of attempts).
 * It declares n be a probably prime with a probability at least (1 - 4^−k).
 *
 * Parameters:
 *  n      - the number of test.
 *  trials - the number of attempts.
 *
 * Return true if n is probably prime, false if it's composite.
 */
inline bool IsProbablePrime(const BigInt& n, int trials = 25) {
    assert(trials > 0);

    if (n.size() <= 1) return IsPrime(n.ToUInt32());
    if (n.size() <= 2) return IsPrime(n.ToUInt64());
    if (factor::SmallFactor(n) != n) return false;

    BigIntRandomGenerator generator;
    Array<BigInt> witnesses(trials);
    for (int i = 0; i < trials; ++i) {
        witnesses[i] = generator.Generate(n);
        witnesses[i] += 2;
    }

    return prime::MillerRabinTest(n, witnesses);
}

/**
 * Test if a Mersennet number M_n = 2^p-1 is prime.
 *
 * This function uses the Lucas-Lehmer test, which is an efficient deterministic
 * primality test for determining if a Mersenne number M_n is prime. Since it is
 * known that Mersenne numbers can only be prime for prime subscripts, attention
 * can be restricted to Mersenne numbers of the form M_p = 2^p-1, where p is an
 * odd prime.
 */
inline bool IsMersennePrime(int p) {
    CHECK(p > 0);
    CHECK(p < (1 << 26)) << "p is too large.";

    if (p == 2) return true;

    // If p is not a odd prime, 2^p - 1 is not prime too.
    if (!IsPrime(p)) return false;

    // The first 7-th mersenne number is M2 M3 M5 M7 M13 M17 M19.
    if (p < 23) {
        return p != 11;
    }

    BigInt mp = (BigInt(1) << p) - 1;

    // If p = 3 (mod 4) and p, 2p + 1 both prime, then 2^p - 1 divides
    // 2p + 1. Cheap test.
    if (p % 4 == 3) {
        if (IsPrime(2 * p + 1) && mp % (2 * p + 1) == 0) {
            return false;
        }
    }

    // Do a little trial division first. Save quite a bit of time.
    int int_max = std::numeric_limits<int>::max();
    int tlim = p / 2;
    if (tlim > int_max / (2 * p))
        tlim = int_max / (2 * p);
    for (int k = 1; k < tlim; ++k) {
        int q = 2 * p * k + 1;
        // Factor must be 1 or 7 (mod 8) and a prime.
        if ((q % 8 == 1 || q % 8 == 7) && (q % 3 != 0) && (q % 5 != 0) &&
            (q % 7 != 0) && (mp % q == 0)) {
            return false;
        }
    }

    BigInt s = 4, t1, t2;
    for (int k = 3; k <= p; ++k) {
        s = s * s - 2;

        // s = s % mp, but more efficiently.
        if (s < 0) s += mp;
        t1 = s >> p;
        t2 = s - (t1 << p);
        s = t1 + t2;
        while (s >= mp) {
            s -= mp;
        }
    }

    return s == 0;
}

} // namespace cl

#endif // CODELIBRARY_MATH_PRIME_IS_PRIME_H_
