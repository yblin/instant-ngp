//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_PRIME_WHEEL_SIEVE_H_
#define CODELIBRARY_MATH_PRIME_WHEEL_SIEVE_H_

#include <cstdint>
#include <limits>

#include "codelibrary/base/array.h"
#include "codelibrary/math/isqrt.h"

namespace cl {
namespace prime {

/**
 * Implement a generic wheel sieve. It removes all integers that are not
 * relatively prime to the given wheel_primes.
 *
 * For example, if the wheel_primes[] = {2}, then all even numbers will be
 * ignores. This class is a 2 * 3 * 5 wheel, which means the wheel_primes is
 * {2, 3, 5}, and the period of wheel is 30.
 *
 * The wheel sieve is stored in a bit vector, where each wheel can be
 * represented by just one 8-bit integer.
 *
 * This code also implements a cache optimized segmented sieving strategy, which
 * is fast.
 *
 * Comparison:
 *            Eratosthenes Sieve  Euler Sieve   Wheel30(No segment)  This one
 * N = 10^6        0.005 s          0.02  s         0.001 s           0.001 s
 * N = 10^7        0.063 s          0.145 s         0.007 s           0.001 s
 * N = 10^8        1.429 s          1.437 s         0.132 s           0.062 s
 * N = 10^9        14.149s          14.742s         3.334 s           0.625 s
 *
 * Users must use the specialized class: WheelSieve30 or WheelSieve210.
 * The following is the sample usage to print the primes under 1000:
 *
 *   WheelSieve30 w(1000);
 *   // WheelSieve30 starts from 7.
 *   cout << 2 << endl;
 *   cout << 3 << endl;
 *   cout << 5 << endl;
 *   for (auto p : w) {
 *       if (w.IsPrime(p))
 *           cout << p << endl;
 *   }
 *
 * Tempalte parameters:
 *   Byte         - uint8_t for WheelSieve30, and uint64_t for WheelSieve210.
 *   WHEEL_PERIOD - Period of wheel, equal to the product of all wheel_primes.
 *   N_COPRIMES   - the number of integers that are relatively prime to all
 *                  wheel_primes.
 */
template <typename Byte, int WHEEL_PERIOD, int N_COPRIMES, int FIRST_PRIME>
class WheelSieve {
    // CPU's L1 data cache size (in bytes).
    static const int L1_DATA_CACHE_SIZE = 32768;

    // If 'p' is a prime, then the next number need to be tested is
    // p + offset[p / WHEEL_PERIOD].
    // In the sieve of Eratosthenes, the next number need to be tested is
    // p * (p + offset[p / WHEEL_PERIOD]).
    static const int OFFSET[N_COPRIMES];

    // The array of all integers that are smaller than primorial and relatively
    // prime to wheel_primes.
    static const int COPRIMES[N_COPRIMES];

    // Cell index for a number mod WHEEL_PERIOD.
    static const int CELL_INDEX[WHEEL_PERIOD];

    // Used to fast check if a number is a prime.
    static const Byte BIT_POSITION[WHEEL_PERIOD];

public:
    /**
     * Iterator for WheelSieve, it starts from FIRST_PRIME.
     */
    class Iterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = int;
        using difference_type   = int;
        using pointer           = const value_type*;
        using reference         = const value_type&;

        Iterator() = default;

        /**
         * Construct a iterator by the given index of wheel and cell.
         */
        Iterator(int n_wheel, int n_cell)
            : n_wheel_(n_wheel), n_cell_(n_cell) {}

        bool operator == (const Iterator& rhs) const {
            return n_wheel_ == rhs.n_wheel_ && n_cell_ == rhs.n_cell_;
        }

        bool operator != (const Iterator& rhs) const {
            return !(*this == rhs);
        }

        int operator*() const {
            return n_wheel_ * WHEEL_PERIOD + COPRIMES[n_cell_];
        }

        Iterator& operator ++() {
            ++n_cell_;
            if (n_cell_ == N_COPRIMES) {
                n_cell_ = 0;
                ++n_wheel_;
            }
            return *this;
        }

        /**
         * Return the index of wheel.
         */
        int n_wheel() const {
            return n_wheel_;
        }

        /**
         * Return the index of cell.
         */
        int n_cell() const {
            return n_cell_;
        }

    private:
        int n_wheel_ = -1; // The index of wheel.
        int n_cell_ = -1;  // The index of cell.
    };

    explicit WheelSieve(int size) {
        CHECK(size > 0);

        Initialize(size);
        SegmentedSieve();
    }

    /**
     * Reset the size and recompute the sieve.
     */
    void Reset(int size) {
        CHECK(size > 0);

        Initialize(size);
        SegmentedSieve();
    }

    /**
     * Note that, it starts with 7, not 2.
     */
    Iterator begin() const {
        return {0, 1};
    }

    Iterator end() const {
        return end_;
    }

    /**
     * Find the iterator that point to the number no smaller than n.
     *
     * It can be used for fast next_prime(). For example:
     *
     *   Iterator i = wheel.Find(n);
     *   While (i != wheel.end() && !wheel.IsPrime(i))
     *       ++i;
     */
    Iterator Find(int n) const {
        CHECK(n > 0);

        if (n >= size_) return end_;
        if (n < FIRST_PRIME) return begin();

        return Iterator(n / WHEEL_PERIOD, CELL_INDEX[n % WHEEL_PERIOD]);
    }

    /**
     * Return true if the given number is a prime.
     */
    bool IsPrime(int n) const {
        CHECK(n / WHEEL_PERIOD < n_wheels_);

        if (n >= FIRST_PRIME) {
            return (bit_sieve_[n / WHEEL_PERIOD] &
                    BIT_POSITION[n % WHEEL_PERIOD]) != 0;
        }

        return n == 2 || n == 3 || n == 5 || n == 7;
    }

    /**
     * Return true if the given iterator point to a prime.
     */
    bool IsPrime(Iterator i) const {
        CHECK(i.n_wheel() >= 0 && i.n_wheel() < n_wheels_);

        return IsPrime(i.n_wheel(), i.n_cell());
    }

    int size() const {
        return size_;
    }

protected:
    /**
     * Initialize the sieve.
     */
    void Initialize(int size) {
        size_ = size;
        end_ = Iterator(size_ / WHEEL_PERIOD, CELL_INDEX[size_ % WHEEL_PERIOD]);
        if (BIT_POSITION[size_ % WHEEL_PERIOD] != 0) ++end_;
        n_wheels_ = size_ / WHEEL_PERIOD + 1;
        bit_sieve_.assign(n_wheels_, std::numeric_limits<Byte>::max());
    }

    /**
     * Check if the number at the position (whell, cell) is a prime.
     */
    bool IsPrime(int wheel, int cell) const {
        return (bit_sieve_[wheel] & (Byte(1) << cell)) != 0;
    }

    /**
     * Get the number at the position (wheel, cell).
     */
    int GetNumber(int wheel, int cell) const {
        return wheel * WHEEL_PERIOD + COPRIMES[cell];
    }

    /**
     * Implement segmented sieve of Eratosthenes with wheel factorization.
     */
    void SegmentedSieve() {
        // Perform segmented sieve of Eratosthenes.
        int sqrt_n = ISqrt(size_);
        int segment_size = std::max(L1_DATA_CACHE_SIZE, sqrt_n / WHEEL_PERIOD);

        // Store the primes below sqrt_n.
        Array<int> primes;

        int n_cell = 1, n_wheel = 0;
        for (int low = 0; low < n_wheels_; low += segment_size) {
            int high = low + segment_size;
            high = std::min(n_wheels_ - 1, high);
            int sqrt_high = static_cast<int>(
                        ISqrt(static_cast<unsigned int>(high) * WHEEL_PERIOD));

            for (; n_wheel < high; ++n_wheel) {
                bool finish = false;
                while (true) {
                    if (IsPrime(n_wheel, n_cell)) {
                        int prime = GetNumber(n_wheel, n_cell);
                        if (prime > sqrt_high) {
                            finish = true;
                            break;
                        }

                        primes.push_back(prime);
                        if (prime * prime <= sqrt_high) {
                            Sieve(prime, prime * prime / WHEEL_PERIOD,
                                  sqrt_high / WHEEL_PERIOD + 1);
                        }
                    }
                    ++n_cell;
                    if (n_cell == N_COPRIMES) {
                        n_cell = 0;
                        break;
                    }
                }
                if (finish) break;
            }

            // Sieve the current segment.
            for (int p : primes) {
                Sieve(p, low, high);
            }
        }
    }

    /**
     * Strike off all multiplers of 'p' in range [low, high].
     */
    void Sieve(int p, int low, int high) {
        int index = CELL_INDEX[p % WHEEL_PERIOD];
        int offset[N_COPRIMES];
        Byte b[N_COPRIMES];

        int j = index;
        unsigned int q = p * p;
        int sum_total = 0;
        for (int i = 0; i < N_COPRIMES; ++i) {
            b[j] = ~BIT_POSITION[q % WHEEL_PERIOD];
            unsigned int q1 = q + p * OFFSET[j];
            offset[j] = q1 / WHEEL_PERIOD - q / WHEEL_PERIOD;
            sum_total += offset[j];
            q = q1;

            if (N_COPRIMES == 8) // Trick for WheelSieve30
                j = (j + 1) & 7;
            else
                j = (j + 1 == N_COPRIMES) ? 0 : j + 1;
        }

        int begin = p * p / WHEEL_PERIOD;
        if (begin < low) {
            begin += (low - begin) / sum_total * sum_total;
        }

        for (int i = begin; i <= high; ) {
            bit_sieve_[i] &= b[index];
            i += offset[index];
            if (N_COPRIMES == 8) // Trick for WheelSieve30
                index = (index + 1) & 7;
            else
                index = (index + 1 == N_COPRIMES) ? 0 : index + 1;
        }
    }

    // Upper bound of the wheel sieve.
    int size_ = 0;

    // The number of wheels.
    int n_wheels_ = 0;

    // The end iterator.
    Iterator end_;

    // Each wheel is represented by one byte integer.
    Array<Byte> bit_sieve_;
};

using WheelSieve30 = WheelSieve<uint8_t, 30, 8, 7>;

template <>
const int WheelSieve30::OFFSET[8]   = { 6, 4, 2, 4, 2, 4, 6, 2 };
template <>
const int WheelSieve30::COPRIMES[8] = { 1, 7, 11, 13, 17, 19, 23, 29 };
template <>
const int WheelSieve30::CELL_INDEX[30] = {
    0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 4, 4,
    5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7
};
template <>
const uint8_t WheelSieve30::BIT_POSITION[30] = {
    0, 1 << 0, 0, 0, 0, 0, 0, 1 << 1, 0, 0, 0, 1 << 2, 0, 1 << 3, 0, 0,
    0, 1 << 4, 0, 1 << 5, 0, 0, 0, 1 << 6, 0, 0, 0, 0, 0, 1 << 7
};

using WheelSieve210 = WheelSieve<uint64_t, 210, 48, 11>;

template <>
const int WheelSieve210::OFFSET[48]   = {
    10, 2, 4, 2, 4, 6, 2, 6, 4, 2, 4, 6, 6, 2, 6, 4, 2, 6,
    4, 6, 8, 4, 2, 4, 2, 4, 8, 6, 4, 6, 2, 4, 6, 2, 6,
    6, 4, 2, 4, 6, 2, 6, 4, 2, 4, 2, 10, 2
};
template <>
const int WheelSieve210::COPRIMES[48] = {
    1, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79,
    83, 89, 97, 101, 103, 107, 109, 113, 121, 127, 131, 137, 139, 143, 149, 151,
    157, 163, 167, 169, 173, 179, 181, 187, 191, 193, 197, 199, 209
};
template <>
const int WheelSieve210::CELL_INDEX[210] = {
    0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  3,  3,  3,  3,
    4,  4,  5,  5,  5,  5,  6,  6,  6,  6,  6,  6,  7,  7,  8,  8,  8,  8,
    8,  8,  9,  9,  9,  9,  10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16,
    17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20,
    21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 24, 24, 24, 24,
    25, 25, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28,
    28, 28, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 31, 31, 32, 32, 32, 32,
    33, 33, 33, 33, 33, 33, 34, 34, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36,
    36, 36, 37, 37, 37, 37, 38, 38, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40,
    41, 41, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 44, 44, 45, 45, 45, 45,
    46, 46, 47, 47, 47, 47, 47, 47, 47, 47, 47, 47
};
template <>
const uint64_t WheelSieve210::BIT_POSITION[210] = {
    0, 1ULL << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1ULL << 1, 0, 1ULL << 2, 0, 0,
    0, 1ULL << 3, 0, 1ULL << 4, 0, 0, 0, 1ULL << 5, 0, 0, 0, 0, 0, 1ULL << 6,
    0, 1ULL << 7, 0, 0, 0, 0, 0, 1ULL << 8, 0, 0, 0, 1ULL << 9, 0, 1ULL << 10,
    0, 0, 0, 1ULL << 11, 0, 0, 0, 0, 0, 1ULL << 12, 0, 0, 0, 0, 0, 1ULL << 13,
    0, 1ULL << 14, 0, 0, 0, 0, 0, 1ULL << 15, 0, 0, 0, 1ULL << 16, 0,
    1ULL << 17, 0, 0, 0, 0, 0, 1ULL << 18, 0, 0, 0, 1ULL << 19, 0, 0, 0, 0, 0,
    1ULL << 20, 0, 0, 0, 0, 0, 0, 0, 1ULL << 21, 0, 0, 0, 1ULL << 22, 0,
    1ULL << 23, 0, 0, 0, 1ULL << 24, 0, 1ULL << 25, 0, 0, 0, 1ULL << 26, 0, 0,
    0, 0, 0, 0, 0, 1ULL << 27, 0, 0, 0, 0, 0, 1ULL << 28, 0, 0, 0, 1ULL << 29,
    0, 0, 0, 0, 0, 1ULL << 30, 0, 1ULL << 31, 0, 0, 0, 1ULL << 32, 0, 0, 0, 0,
    0, 1ULL << 33, 0, 1ULL << 34, 0, 0, 0, 0, 0, 1ULL << 35, 0, 0, 0, 0, 0,
    1ULL << 36, 0, 0, 0, 1ULL << 37, 0, 1ULL << 38, 0, 0, 0, 1ULL << 39, 0, 0,
    0, 0, 0, 1ULL << 40, 0, 1ULL << 41, 0, 0, 0, 0, 0, 1ULL << 42, 0, 0, 0,
    1ULL << 43, 0, 1ULL << 44, 0, 0, 0, 1ULL << 45, 0, 1ULL << 46, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1ULL << 47
};

} // namespace prime
} // namespace cl

#endif // CODELIBRARY_MATH_PRIME_WHEEL_SIEVE_H_
