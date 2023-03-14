//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_GEMM_H_
#define CODELIBRARY_MATH_MATRIX_GEMM_H_

#include <algorithm>
#include <cstring>

namespace cl {
namespace blas {

/**
 * Computes a matrix-matrix product with general matrices, which is defined as:
 *
 *   c = a * b
 *
 * Input:
 *  m - The number of rows of the matrix a.
 *  n - The number of columns of the matrix a.
 *  k - The number of columns of the matrix b.
 *  a - Array, size m * n.
 *  b - Array, size n * k.
 *  c - Array, size m * k.
 *
 * Output:
 *  c - Update matrix c.
 */
template <typename T>
void GEMM(int m, int n, int k, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(c != a && c != b);

    const int block_size = 128;
    int ii, jj, pp;
    T sum;

    std::memset(c, 0, sizeof(T) * m * k);
    for (int i = 0; i < m; i += block_size) {
        for (int j = 0; j < n; j += block_size) {
            for (int p = 0; p < k; p += block_size) {
                int block_m = (i + block_size <= m) ? block_size : (m - i);
                int block_n = (j + block_size <= n) ? block_size : (n - j);
                int block_k = (p + block_size <= k) ? block_size : (k - p);
                for (ii = i; ii < i + block_m; ii++) {
                    for (jj = j; jj < j + block_n; jj++) {
                        sum = 0.0;
                        for (pp = p; pp + 4 < p + block_k; pp += 4) {
                            sum += a[ii * k + pp] * b[pp * n + jj];
                            sum += a[ii * k + pp + 1] * b[(pp + 1) * n + jj];
                            sum += a[ii * k + pp + 2] * b[(pp + 2) * n + jj];
                            sum += a[ii * k + pp + 3] * b[(pp + 3) * n + jj];
                        }
                        for (; pp < p + block_k; ++pp) {
                            sum += a[ii * k + pp] * b[pp * n + jj];
                        }
                        c[ii * n + jj] += sum;
                    }
                }
            }
        }
    }
}

} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_GEMM_H_
