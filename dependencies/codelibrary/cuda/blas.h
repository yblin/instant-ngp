//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_BLAS_H_
#define CODELIBRARY_CUDA_BLAS_H_

#include <cublas_v2.h>

#include "codelibrary/cuda/check.h"

/**
 * Check the result of a cuBLAS call.
 */
#define CUBLAS_CHECK(x)                                                 \
    do {                                                                \
        cublasStatus_t s = x;                                           \
        CHECK(s == CUBLAS_STATUS_SUCCESS) << cl::cuda::BLASGetError(s); \
    } while(0)

namespace cl {
namespace cuda {

/**
 * Return string name of cublasStatus.
 */
inline std::string BLASGetError(cublasStatus_t error) {
    switch (error) {
    case CUBLAS_STATUS_SUCCESS:
        return "CUBLAS_STATUS_SUCCESS";
    case CUBLAS_STATUS_NOT_INITIALIZED:
        return "CUBLAS_STATUS_NOT_INITIALIZED";
    case CUBLAS_STATUS_ALLOC_FAILED:
        return "CUBLAS_STATUS_ALLOC_FAILED";
    case CUBLAS_STATUS_INVALID_VALUE:
        return "CUBLAS_STATUS_INVALID_VALUE";
    case CUBLAS_STATUS_ARCH_MISMATCH:
        return "CUBLAS_STATUS_ARCH_MISMATCH";
    case CUBLAS_STATUS_MAPPING_ERROR:
        return "CUBLAS_STATUS_MAPPING_ERROR";
    case CUBLAS_STATUS_EXECUTION_FAILED:
        return "CUBLAS_STATUS_EXECUTION_FAILED";
    case CUBLAS_STATUS_INTERNAL_ERROR:
        return "CUBLAS_STATUS_INTERNAL_ERROR";
    case CUBLAS_STATUS_NOT_SUPPORTED:
        return "CUBLAS_STATUS_NOT_SUPPORTED";
    default: return "<unknown>";
    }
}

/**
 * A Wrapper for cuBLAS.
 */
class BLAS {
    BLAS() {
        CUBLAS_CHECK(cublasCreate(&handle_));
    }

    ~BLAS() {
        CUBLAS_CHECK(cublasDestroy(handle_));
    }

public:
    static BLAS* get_instance() {
        static BLAS cudablas;
        return &cudablas;
    }

    /**
     * Return the version number of the cuBLAS library.
     */
    int get_version() {
        int version;
        CUBLAS_CHECK(cublasGetVersion(handle_, &version));
        return version;
    }

    /**
     * This function scales the vector 'x' by the scalar 'a' and overwrites it
     * with the result. Hence, the performed operation is x[j] = a * x[j] for
     * i = 0,...,n-1 and j = i * incx.
     */
    void Scale(int n, float a, float* x, int incx) {
        CUBLAS_CHECK(cublasSscal(handle_, n, &a, x, incx));
    }
    void Scale(int n, double a, double* x, int incx) {
        CUBLAS_CHECK(cublasDscal(handle_, n, &a, x, incx));
    }

    /**
     * This function multiplies the vector 'x' by the scalar 'a' and adds it to
     * the vector 'y' overwriting the latest vector with the result. Hence, the
     * performed operation is y[j] = a * x[k] + y[j] for i = 0,...,n-1,
     * k = i * incx and j = i * incy.
     */
    void Axpy(int n, float a, const float *x, int incx, float* y, int incy) {
        CUBLAS_CHECK(cublasSaxpy(handle_, n, &a, x, incx, y, incy));
    }
    void Axpy(int n, double a, const double *x, int incx, double* y, int incy) {
        CUBLAS_CHECK(cublasDaxpy(handle_, n, &a, x, incx, y, incy));
    }

    /**
     * This function computes the dot product of vectors 'x' and 'y'. Hence, the
     * result is Sum(x[k] * y[j]) where k = i * incx and j = i * incy.
     */
    float Dot(int n, const float* x, int incx, const float* y, int incy) {
        float result = 0.0f;
        CUBLAS_CHECK(cublasSdot(handle_, n, x, incx, y, incy, &result));
        return result;
    }
    double Dot(int n, const double* x, int incx, const double* y, int incy) {
        double result = 0.0;
        CUBLAS_CHECK(cublasDdot(handle_, n, x, incx, y, incy, &result));
        return result;
    }

    /**
     * This function computes the Euclidean norm of the vector x. The code uses
     * a multiphase model of accumulation to avoid intermediate underflow and
     * overflow, with the result being equivalent to Sqrt(Sum(x[j] * x[j]) where
     * j = i * incx in exact arithmetic.
     */
    float Normal(int n, const float* x, int incx) {
        float result = 0.0f;
        CUBLAS_CHECK(cublasSnrm2(handle_, n, x, incx, &result));
        return result;
    }
    double Normal(int n, const double* x, int incx) {
        double result = 0.0;
        CUBLAS_CHECK(cublasDnrm2(handle_, n, x, incx, &result));
        return result;
    }

    /**
     * This function performs the matrix-vector multiplication
     *
     *  y = a * op(A) * x + b * y
     *
     * where 'A' is a 'm' x 'n' matrix stored in column-major format, 'x' and
     * 'y' are vectors, and 'a' and 'b' are scalars. Also, for matrix 'A'
     *
     *           A            if transa = CUBLAS_OP_N
     *  op(A) =  A^T          if transa = CUBLAS_OP_T
     *           A^H          if transa = CUBLAS_OP_H
     */
    void Gemv(cublasOperation_t trans, int m, int n, float alpha,
              const float* mat_a, int lda, const float* x, int incx,
              float beta, float *y, int incy) {
        CUBLAS_CHECK(cublasSgemv(handle_, trans, m, n, &alpha, mat_a, lda,
                                 x, incx, &beta, y, incy));
    }
    void Gemv(cublasOperation_t trans, int m, int n, double alpha,
              const double* mat_a, int lda, const double* x, int incx,
              double beta, double *y, int incy) {
        CUBLAS_CHECK(cublasDgemv(handle_, trans, m, n, &alpha, mat_a, lda,
                                 x, incx, &beta, y, incy));
    }

    /**
     * This function performs the matrix-matrix multiplication
     *
     *  C = alpha * op(A) * op(B) + beta * C
     *
     * where 'alpha' and 'beta' are scalars, and 'A', 'B', and 'C' are matrices
     * stored in column-major format with dimensions op(A) m x k, op(B) k x n
     * and C m x n, respectively. Also, for matrix
     *
     *           A            if transa = CUBLAS_OP_N
     *  op(A) =  A^T          if transa = CUBLAS_OP_T
     *           A^H          if transa = CUBLAS_OP_H
     *
     * and op(B) is defined similarly for matrix B.
     */
    void GEMM(cublasOperation_t transa, cublasOperation_t transb,
              int m, int n, int k,
              float alpha,
              const float* a, int lda,
              const float* b, int ldb,
              float beta,
              float* c, int ldc) {
        CUBLAS_CHECK(cublasSgemm(handle_, transa, transb, m, n, k, &alpha,
                                 a, lda, b, ldb, &beta, c, ldc));
    }
    void GEMM(cublasOperation_t transa, cublasOperation_t transb,
              int m, int n, int k,
              double alpha,
              const double* a, int lda,
              const double* b, int ldb,
              double beta,
              double* c, int ldc) {
        CUBLAS_CHECK(cublasDgemm(handle_, transa, transb, m, n, k, &alpha,
                                 a, lda, b, ldb, &beta, c, ldc));
    }

private:
    cublasHandle_t handle_;
};

} // namespace cuda
} // namespace cl

#define CUDA_BLAS cl::cuda::BLAS::get_instance()

#endif // CODELIBRARY_CUDA_BLAS_H_
