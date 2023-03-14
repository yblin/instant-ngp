//
// Copyright 2017-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_H_
#define CODELIBRARY_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_H_

#include <utility>

#include "codelibrary/base/array.h"
#include "codelibrary/math/common_factor.h"
#include "codelibrary/math/modular/modular.h"

namespace cl {

/**
 * Simultaneous congruences solver.
 *
 * Consider a sequence of congruence equations:
 *
 *   x = r_1 (mod m_1)
 *    ...
 *   x = r_k (mod m_k)
 *
 * where the m_i are pairwise coprime.
 *
 * Using china remainder theorem, the solution of the set of congruences is
 *
 *   x = r_1 * b_1 * M/m_1 + ... + a_k * b_k * M/m_k (mod M)
 *
 * where
 *
 *   M = m_1 * m_2 * ... * m_k
 *
 * and the b_i are determined from
 *
 *   b_i * M / m_i = 1 (mod m_i)
 */
template <typename T>
class SimultaneousCongruencesSolver {
public:
    SimultaneousCongruencesSolver() = default;

    SimultaneousCongruencesSolver(const SimultaneousCongruencesSolver&)
        = delete;

    SimultaneousCongruencesSolver&
    operator=(const SimultaneousCongruencesSolver&) = delete;

    /**
     * Insert a congruence relation into solver.
     * And compute a solution that satisfy the current congruences.
     *
     * This function will done in nearly O(b^2), where b is the number of digits
     * of modulo.
     */
    void InsertCongruence(const T& modulo, const T& remainder) {
        CHECK(modulo > 0);
        CHECK(remainder >= 0 && remainder < modulo);

        if (congruences_.empty()) {
            m_ = modulo;
            x_ = remainder;
        } else {
            T x, y;
            T d = ExtendedGCD(m_, modulo, &x, &y); // a * x + m * y = GCD(a, m).

            T r = remainder - x_;
            CHECK(r % d == 0 && "No solution.");

            T m = modulo / d;
            x = ModMul(r / d, x, m);
            x_ += x * m_;

            T t = m * m_;
            CHECK(m_ <= t / m && "Overflow detected.");
            m_ = t;

            x_ %= m_;
        }

        congruences_.emplace_back(modulo, remainder);
    }

    /**
     * Check if the current solution is correct.
     */
    bool IsCorrect() const {
        T x = solution();
        if (x >= m_ || x < 0) return false;

        for (const std::pair<T, T>& c : congruences_) {
            if (x % c.first != c.second) return false;
        }
        return true;
    }

    /**
     * Return the solution of simultaneous congruences.
     */
    T solution() const {
        return x_ < 0 ? x_ + m_ : x_;
    }

    /**
     * Return the least common multiple of modulus.
     *
     * Every y that satisfy (y = x) mod(m) is a solution.
     */
    T modulo() const {
        return m_;
    }

    const Array<std::pair<T, T>>& congruences() const {
        return congruences_;
    }

private:
    // The solution to the equations, it may be negative.
    T x_ = 0;

    // The least common multiple of modulus: m_1 * ... * m_k.
    T m_ = 0;

    // Input simultaneous congruence equations.
    Array<std::pair<T, T>> congruences_;
};

} // namespace cl

#endif // CODELIBRARY_MATH_MODULAR_SIMULTANEOUS_CONGRUENCES_SOLVER_H_
