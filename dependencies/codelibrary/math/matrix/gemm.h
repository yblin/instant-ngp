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
    T sum;

    std::memset(c, 0, sizeof(T) * m * k);
    if (m <= block_size && n <= block_size && k <= block_size) {
        int p;
        for (int i = 0; i < m; ++i) {
            for (int j = 0; j < k; ++j) {
                sum = T(0);
                for (p = 0; p + 4 < n; p += 4) {
                    sum += a[i * n + p]     * b[ p      * k + j];
                    sum += a[i * n + p + 1] * b[(p + 1) * k + j];
                    sum += a[i * n + p + 2] * b[(p + 2) * k + j];
                    sum += a[i * n + p + 3] * b[(p + 3) * k + j];
                }
                for (; p < n; ++p) {
                    sum += a[i * n + p] * b[p * k + j];
                }
                c[i * k + j] += sum;
            }
        }
    } else {
        int i, j, p, block_m, block_n, block_k;
        #pragma omp parallel for private(sum)
        for (int ii = 0; ii < m; ii += block_size) {
            for (int jj = 0; jj < k; jj += block_size) {
                for (int pp = 0; pp < n; pp += block_size) {
                    block_m = (ii + block_size <= m) ? block_size : (m - ii);
                    block_n = (pp + block_size <= n) ? block_size : (n - pp);
                    block_k = (jj + block_size <= k) ? block_size : (k - jj);
                    for (i = ii; i < ii + block_m; ++i) {
                        for (j = jj; j < jj + block_k; ++j) {
                            sum = T(0);
                            for (p = pp; p + 4 < pp + block_n; p += 4) {
                                sum += a[i * n + p]     * b[ p      * k + j];
                                sum += a[i * n + p + 1] * b[(p + 1) * k + j];
                                sum += a[i * n + p + 2] * b[(p + 2) * k + j];
                                sum += a[i * n + p + 3] * b[(p + 3) * k + j];
                            }
                            for (; p < pp + block_n; ++p) {
                                sum += a[i * n + p] * b[p * k + j];
                            }
                            c[i * k + j] += sum;
                        }
                    }
                }
            }
        }
    }
}

} // namespace blas
} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_GEMM_H_
