//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_PRIME_FERMAT_TEST_H_
#define CODELIBRARY_MATH_PRIME_FERMAT_TEST_H_

#include "codelibrary/math/modular/modular.h"

namespace cl {
namespace prime {

/**
 * Test if a given number is a 'probable' prime number by Fermat test.
 *
 * Fermat's little theorem states that if 'p' is prime and 'a' is not divisible
 * by 'p', then
 *   a ^ (p-1) = 1 (mod p)
 */
template <typename T>
inline bool FermatTest(const T& p, const T& a) {
    return ModPow(a, p - 1, p) == 1;
}

} // namespace prime
} // namespace cl

#endif // CODELIBRARY_MATH_PRIME_FERMAT_TEST_H_
