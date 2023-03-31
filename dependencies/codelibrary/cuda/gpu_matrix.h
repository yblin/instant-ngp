//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_GPU_MATRIX_H_
#define CODELIBRARY_CUDA_GPU_MATRIX_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/cuda/blas.h"
#include "codelibrary/cuda/gpu_vector.h"

namespace cl {
namespace cuda {

/**
 * The row-major GPU matrix.
 *
 * Performance (test on GPU 3090, CPU I9 3070HZ)
 *
 * --------------------------------------------------
 * Matrix and vector multiplication
 * --------------------------------------------------
 * N     GPUMatrix   Matrix (CPU)   Eigen (CPU)
 * 4     19us        24ns           190ns
 * 16    16us        88ns           479ns
 * 32    22us        274ns          14us
 * 64    35us        1us            52us
 * 128   9us         4us            186us
 * 256   10us        18us           546us
 * 512   13us        94us           77us
 * 1024  23us        305us          10ms
 * 2048  223us       1ms            26ms
 * 4096  203us       7ms            203ms
 *
 * --------------------------------------------------
 * Matrix and matrix multiplication
 * --------------------------------------------------
 * N     GPUMatrix   Matrix (CPU)   Eigen (CPU)
 * 4     19us        77ns           51ns
 * 16    23us        1us            709ns
 * 32    22us        17us           4us
 * 64    34us        142us          30us
 * 128   59us        1ms            231us
 * 256   160us       12ms           1ms
 * 512   824us       152ms          14ms
 * 1024  5ms         1.2s           107ms
 * 2048  38ms        9.9s           885ms
 * 4096  281ms       1.3min         7s
 */
template <class T>
class GPUMatrix {
    static_assert(std::is_floating_point<T>::value, "");

public:
    GPUMatrix() = default;

    GPUMatrix(int n_rows, int n_columns, T v = 0)
        : n_rows_(n_rows), n_columns_(n_columns) {
        CHECK(n_rows_ >= 0);
        CHECK(n_columns_ >= 0);
        CHECK(n_rows_ <= INT_MAX / n_columns_);

        memory_.Resize(n_rows_ * n_columns_);
        memory_.Fill(v);
    }

    GPUMatrix(GPUMatrix<T>&& m) noexcept = default;

    /**
     * Assign GPUMatrix from device data.
     */
    explicit GPUMatrix(const GPUMatrix<T>& m)
        : n_rows_(m.rows()), n_columns_(m.columns()), memory_(m.memory_) {}

    /**
     * Assign GPUMatrix from host data.
     */
    explicit GPUMatrix(const ArrayND<T>& data)
        : memory_(data.size()) {
        CHECK(data.n_dimension() == 2);

        n_rows_ = data.shape(0);
        n_columns_ = data.shape(1);
        memory_.CopyFromHost(data.data(), data.size());
    }

    /**
     * Assign GPUMatrix from host data.
     */
    GPUMatrix(int n_rows, int n_columns, const Array<T>& data)
        : n_rows_(n_rows), n_columns_(n_columns), memory_(data) {
        CHECK(n_rows_ * n_columns_ == data.size());
    }

    /**
     * Copy vector to the host.
     */
    void ToHost(ArrayND<T>* data) {
        CHECK(data);

        data->reshape(n_rows_, n_columns_);
        memory_.CopyToHost(data->data());
    }

    GPUMatrix& operator =(const GPUMatrix& rhs) {
        n_rows_ = rhs.n_rows_;
        n_columns_ = rhs.n_columns_;
        memory_.Resize(n_rows_ * n_columns_);
        memory_.CopyFromDevice(rhs.memory_);
        return *this;
    }

    GPUMatrix& operator=(GPUMatrix&& rhs) noexcept {
        std::swap(rhs.n_rows_, this->n_rows_);
        std::swap(rhs.n_columns_, this->n_columns_);
        memory_ = std::move(rhs.memory_);
        return *this;
    }

    GPUMatrix& operator =(const ArrayND<T>& rhs) {
        CHECK(rhs.n_dimension() == 2);

        n_rows_ = rhs.shape(0);
        n_columns_ = rhs.shape(1);
        memory_.Resize(rhs.size());
        memory_.CopyFromHost(rhs.data(), rhs.size());
        return *this;
    }

    GPUMatrix& operator *=(const T& rhs) {
        CUDA_BLAS->Scale(this->size(), rhs, this->data(), 1);
        return *this;
    }

    GPUMatrix& operator +=(const GPUMatrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_ && n_columns_ == rhs.n_columns_);

        CUDA_BLAS->Axpy(this->size(), T(1.0), rhs.data(), 1, this->data(), 1);
        return *this;
    }

    GPUMatrix& operator -=(const GPUMatrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_ && n_columns_ == rhs.n_columns_);

        CUDA_BLAS->Axpy(this->size(), T(-1.0), rhs.data(), 1, this->data(), 1);
        return *this;
    }

    friend const GPUMatrix& operator+(const GPUMatrix& rhs) {
        return rhs;
    }

    friend GPUMatrix operator -(const GPUMatrix& rhs) {
        return rhs * T(-1.0);
    }

    friend GPUMatrix operator *(const GPUMatrix& lhs, T rhs) {
        GPUMatrix res = lhs;
        res *= rhs;
        return res;
    }

    friend GPUMatrix operator *(T lhs, const GPUMatrix& rhs) {
        GPUMatrix res = rhs;
        res *= lhs;
        return res;
    }

    friend GPUMatrix operator +(const GPUMatrix& a, const GPUMatrix& b) {
        CHECK(a.n_rows_ == b.n_rows_ && a.n_columns_ == b.n_columns_);

        GPUMatrix c = a;
        c += b;
        return c;
    }

    friend GPUMatrix operator -(const GPUMatrix& a, const GPUMatrix& b) {
        CHECK(a.n_rows_ == b.n_rows_ && a.n_columns_ == b.n_columns_);

        GPUMatrix c = a;
        c -= b;
        return c;
    }

    friend GPUVector<T> operator *(const GPUMatrix& a, const GPUVector<T>& b) {
        CHECK(a.n_columns() == b.size());

        // Note that the cublas's matrix is column-major, but here we consider
        // the row-marjor matrix of 'a'.
        GPUVector<T> c(a.n_rows());
        CUDA_BLAS->Gemv(CUBLAS_OP_T, a.n_columns(), a.n_rows(), T(1), a.data(),
                        a.n_columns(), b.data(), 1, T(0), c.data(), 1);
        return c;
    }

    friend GPUVector<T> operator *(const GPUVector<T>& b, const GPUMatrix& a) {
        CHECK(a.n_rows() == b.size());

        // Note that the cublas's matrix is column-major, but here we consider
        // the row-marjor matrix of 'a'.
        GPUVector<T> c(a.n_columns());
        CUDA_BLAS->Gemv(CUBLAS_OP_N, a.n_columns(), a.n_rows(), T(1), a.data(),
                        a.n_columns(), b.data(), 1, T(0), c.data(), 1);
        return c;
    }

    friend GPUMatrix operator *(const GPUMatrix& a, const GPUMatrix& b) {
        CHECK(a.n_columns_ == b.n_rows_);

        GPUMatrix c(a.n_rows(), b.n_columns());
        CUDA_BLAS->GEMM(CUBLAS_OP_N, CUBLAS_OP_N,
                        b.n_columns(), a.n_rows(), b.n_rows(),
                        T(1),
                        b.data(), b.n_columns(),
                        a.data(), b.n_rows(),
                        T(0),
                        c.data(), b.n_columns());
        return c;
    }

    int n_rows() const {
        return n_rows_;
    }

    int n_columns() const {
        return n_columns_;
    }

    int size() const {
        return n_rows_ * n_columns_;
    }

    const T* data() const {
        return memory_.data();
    }

    T* data() {
        return memory_.data();
    }

private:
    // The number of rows of the matrix.
    int n_rows_ = 0;

    // The number of columns of the matrix.
    int n_columns_ = 0;

    // GPU memory for matrix.
    GPUMemory<T> memory_;
};

using FGPUMatrix = GPUMatrix<float>;
using RGPUMatrix = GPUMatrix<double>;

} // namespace cuda
} // namespace cl

#endif // CODELIBRARY_CUDA_GPU_MATRIX_H_
