//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FACTOR_SQUFOF_H_
#define CODELIBRARY_MATH_FACTOR_SQUFOF_H_

#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/isqrt.h"

namespace cl {
namespace factor {

/**
 * Factor a number up to 62 bits in size using SQUFOF.
 *
 * For n the product of two primes, this routine will succeed with very high
 * probability, although the likelihood of failure goes up as n increases in
 * size.
 *
 * Empirically, 62-bit factorizations fail about 5% of the time; for smaller n
 * the failure rate is nearly zero.
 */
class SQUFOFFactor {
    static const int MAX_MULTIPLIERS = 16;

    static const uint64_t MULTIPLIERS[MAX_MULTIPLIERS];

    static const int QSIZE = 50;

    // The maximum number of inner loop iterations for all multipliers combined.
    static const int MAX_CYCLES = 40000;

    // The number of iterations to do before switching to the next multiplier.
    static const int ONE_CYCLE_ITER = 300;

    struct Data {
        uint64_t n;
        uint64_t sqrtn[MAX_MULTIPLIERS];
        uint64_t cutoff[MAX_MULTIPLIERS];
        uint64_t q0[MAX_MULTIPLIERS];
        uint64_t p1[MAX_MULTIPLIERS];
        uint64_t q1[MAX_MULTIPLIERS];
        int      num_saved[MAX_MULTIPLIERS];
        uint16_t saved[MAX_MULTIPLIERS][QSIZE];
        bool     failed[MAX_MULTIPLIERS];
    };

public:
    explicit SQUFOFFactor(uint64_t n) {
        CHECK(n >= 2);

        data_.n = n;
    }

    /**
     * If a factor is found, it is returned. If it is a trivial factor, the
     * return value is 'n'. If SQUFOF failed, the return value is 1.
     */
    uint64_t Factor() {
        uint64_t n = data_.n;
        uint64_t factor_found = 0;

        int i = 0;
        for (i = 0; i < MAX_MULTIPLIERS; ++i) {
            // Use the multiplier if the multiplier times n will fit in 62 bits.
            if (n > ((1ULL << 62) - 1) / MULTIPLIERS[i]) break;

            uint64_t tmp = n * MULTIPLIERS[i];
            uint64_t sqrt_tmp = ISqrt(tmp);
            tmp = tmp - sqrt_tmp * sqrt_tmp;

            // Initialize the rest of the fields for this multiplier.
            data_.sqrtn[i]     = sqrt_tmp;
            data_.cutoff[i]    = ISqrt(2 * data_.sqrtn[i]);
            data_.num_saved[i] = 0;
            data_.failed[i]    = false;
            data_.q0[i]        = 1;
            data_.p1[i]        = data_.sqrtn[i];
            data_.q1[i]        = tmp;

            // if n is a perfect square, the factorization has already taken
            // place.
            if (data_.q1[i] == 0) return data_.p1[i];
        }
        if (i == 0) return 1;

        // Perform a block of work using each multiplier in turn, until our
        // budget of work for factoring n is exhausted.
        int num_mult = i;
        int num_iter = 0;
        int num_failed = 0;
        while (num_iter < MAX_CYCLES) {
            // For each cycle of multipliers, begin with the multiplier that is
            // largest. These have a higher probability of factoring n quickly.
            for (int i = num_mult - 1; i >= 0; --i) {
                if (data_.failed[i]) continue;

                // Work on this multiplier for a little while.
                num_iter += OneCycle(i, ONE_CYCLE_ITER, &factor_found);

                // If all multipliers have failed, then SQUFOF has failed.
                if (data_.failed[i]) {
                    if (++num_failed == num_mult) return 1;
                }

                if (factor_found) return factor_found;
            }
        }

        return 1;
    }

private:
    /**
     * Perform one unit of work for SQUFOF with one multiplier.
     */
    int OneCycle(int mult_index, int num_iter, uint64_t* factor) {
        uint64_t sqrtn         = data_.sqrtn[mult_index];
        uint64_t cutoff        = data_.cutoff[mult_index];
        int      num_saved     = data_.num_saved[mult_index];
        uint64_t multiplier    = 2 * MULTIPLIERS[mult_index];
        uint64_t coarse_cutoff = cutoff * multiplier;
        uint16_t *saved        = data_.saved[mult_index];
        uint64_t q0            = data_.q0[mult_index];
        uint64_t p1            = data_.p1[mult_index];
        uint64_t q1            = data_.q1[mult_index];

        uint64_t sqrtq = 0, p0 = 0;
        int i = 0;
        for (i = 0; i < num_iter; ++i) {
            uint64_t tmp = sqrtn + p1 - q1;
            uint64_t q = 1;
            if (tmp >= q1) q += tmp / q1;

            p0 = q * q1 - p1;
            q0 = q0 + (p1 - p0) * q;

            if (q1 < coarse_cutoff) {
                tmp = q1 / EuclideanGCD(q1, multiplier);

                if (tmp < cutoff) {
                    if (num_saved >= QSIZE) {
                        data_.failed[mult_index] = 1;
                        return i;
                    }
                    saved[num_saved++] = static_cast<uint16_t>(tmp);
                }
            }

            // If q0 is a perfect square, then the factorization has probably
            // succeeded.
            sqrtq = ISqrt(q0);
            if (sqrtq * sqrtq == q0) {
                // it *is* a perfect square. If it has not appeared
                // previously in the list for this multiplier, then we're
                // almost finished.
                int j = 0;
                for (j = 0; j < num_saved; ++j) {
                    if (saved[j] == sqrtq)
                        break;
                }

                if (j == num_saved) break;
            }

            // Perform the odd half of the SQUFOF cycle.
            tmp = sqrtn + p0 - q0;
            q = 1;
            if (tmp >= q0) q += tmp / q0;

            p1 = q * q0 - p0;
            q1 = q1 + (p0 - p1) * q;

            if (q0 < coarse_cutoff) {
                tmp = q0 / EuclideanGCD(q0, multiplier);

                if (tmp < cutoff) {
                    if (num_saved >= QSIZE) {
                        data_.failed[mult_index] = 1;
                        return i;
                    }
                    saved[num_saved++] = static_cast<uint16_t>(tmp);
                }
            }
        }

        if (sqrtq == 1) {
            // The above found a trivial factor, so this multiplier has failed.
            data_.failed[mult_index] = true;
            return i;
        } else if (i == num_iter) {
            // No square root found; save the parameters and go on to the next
            // multiplier.
            data_.q0[mult_index] = q0;
            data_.p1[mult_index] = p1;
            data_.q1[mult_index] = q1;
            data_.num_saved[mult_index] = num_saved;
            return i;
        }

        // Square root found; the algorithm cannot fail now.
        // Compute the inverse quadratic form and iterate.
        q0 = sqrtq;
        p1 = p0 + sqrtq * ((sqrtn - p0) / sqrtq);
        uint64_t scaledn = data_.n * MULTIPLIERS[mult_index];
        q1 = (scaledn - uint64_t(p1) * p1) / q0;

        while (true) {
            uint64_t tmp = sqrtn + p1 - q1;
            uint64_t q = 1;
            if (tmp >= q1) q += tmp / q1;

            p0 = q * q1 - p1;
            q0 = q0 + (p1 - p0) * q;

            if (p0 == p1) {
                q0 = q1;
                break;
            }

            tmp = sqrtn + p0 - q0;
            q = 1;
            if (tmp >= q0) q += tmp / q0;

            p1 = q * q0 - p0;
            q1 = q1 + (p0 - p1) * q;

            if (p0 == p1) break;
        }

        // q0 is the factor of n. Remove factors that exist in the multiplier
        // and save whatever's left.
        q0 = q0 / EuclideanGCD(q0, multiplier);
        *factor = q0;
        return i;
    }

    Data data_;
};

const uint64_t SQUFOFFactor::MULTIPLIERS[16] = {
    1, 3, 5, 7, 11, 3*5, 3*7, 3*11, 5*7, 5*11, 7*11, 3*5*7, 3*5*11, 3*7*11,
    5*7*11, 3*5*7*11
};

inline uint64_t SQUFOF(uint64_t n) {
    SQUFOFFactor s(n);
    return s.Factor();
}

} // namespace factor
} // namespace cl

#endif // CODELIBRARY_MATH_FACTOR_SQUFOF_H_
