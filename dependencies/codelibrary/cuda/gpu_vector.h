//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_GPU_VECTOR_H_
#define CODELIBRARY_CUDA_GPU_VECTOR_H_

#include "codelibrary/base/array.h"
#include "codelibrary/cuda/blas.h"
#include "codelibrary/cuda/gpu_memory.h"

namespace cl {
namespace cuda {

/**
 * GPU vector.
 *
 * Performance (test on GPU 3090, CPU I9 3070HZ)
 *
 * Vector addition
 * -----------------------------------------
 * N          GPUVector   Vector (CPU)   Eigen (CPU)
 * 100        10us        43ns           16ns
 * 1000       10us        389ns          202ns
 * 10000      10us        4us            2us
 * 100000     12us        117us          24us
 * 1000000    325us       1ms            197us
 * 10000000   1ms         33ms           4ms
 * 100000000  7ms         326ms          66ms
 */
template <class T>
class GPUVector {
    static_assert(std::is_floating_point<T>::value, "");

public:
    GPUVector() = default;

    GPUVector(int n, T v = 0)
        : size_(n), memory_(n) {
        memory_.Fill(v);
    }

    GPUVector(GPUVector<T>&& v) noexcept = default;

    /**
     * Assign GPUVector from device data.
     */
    explicit GPUVector(const GPUVector<T>& v)
        : size_(v.size()), memory_(v.memory_) {}

    /**
     * Assign GPUVector from host data.
     */
    explicit GPUVector(const Array<T>& data)
        : size_(data.size()), memory_(data.size()) {
        memory_.CopyFromHost(data.data(), data.size());
    }

    /**
     * Copy vector to the host.
     */
    void ToHost(Array<T>* data) {
        CHECK(data);

        data->resize(size_);
        memory_.CopyToHost(data->data());
    }

    GPUVector& operator =(GPUVector&& rhs) noexcept = default;

    GPUVector& operator =(const GPUVector& rhs) {
        size_ = rhs.size_;
        memory_.Resize(size_);
        memory_.CopyFromDevice(rhs.memory_);
        return *this;
    }

    GPUVector& operator =(const Array<T>& rhs) {
        size_ = rhs.size();
        memory_.Resize(size_);
        memory_.CopyFromHost(rhs.data(), rhs.size());
        return *this;
    }

    GPUVector& operator *=(const T& rhs) {
        CUDA_BLAS->Scale(this->size_, rhs, this->data(), 1);
        return *this;
    }

    GPUVector& operator +=(const GPUVector& rhs) {
        CHECK(size_ == rhs.size_);

        CUDA_BLAS->Axpy(size_, T(1.0), rhs.data(), 1, this->data(), 1);
        return *this;
    }

    GPUVector& operator -=(const GPUVector& rhs) {
        CHECK(size_ == rhs.size_);

        CUDA_BLAS->Axpy(size_, T(-1.0), rhs.data(), 1, this->data(), 1);
        return *this;
    }

    /**
     * Return euclidean norm.
     */
    T norm() const {
        return CUDA_BLAS->Normal(size_, data(), 1);
    }

    friend const GPUVector& operator+(const GPUVector& rhs) {
        return rhs;
    }

    friend GPUVector operator -(const GPUVector& rhs) {
        return rhs * T(-1.0);
    }

    friend GPUVector operator *(const GPUVector& lhs, T rhs) {
        GPUVector res = lhs;
        res *= rhs;
        return res;
    }

    friend GPUVector operator *(T lhs, const GPUVector& rhs) {
        GPUVector res = rhs;
        res *= lhs;
        return res;
    }

    friend GPUVector operator +(const GPUVector& a, const GPUVector& b) {
        CHECK(a.size_ == b.size_);

        GPUVector c(a);
        c += b;
        return c;
    }

    friend GPUVector operator -(const GPUVector& a, const GPUVector& b) {
        CHECK(a.size_ == b.size_);

        GPUVector c = a;
        c -= b;
        return c;
    }

    int size() const {
        return size_;
    }

    const T* data() const {
        return memory_.data();
    }

    T* data() {
        return memory_.data();
    }

private:
    // The number of elements in the vector.
    int size_ = 0;

    // GPU memory.
    GPUMemory<T> memory_;
};

/**
 * Dot product of two vectors.
 *
 * The main purpose of using lowercase letters 'dot' is to make it similar to
 * python function.
 */
template <typename T>
T dot(const GPUVector<T>& a, const GPUVector<T>& b) {
    CHECK(a.size() == b.size());

    return Dot(a.size(), a.data(), 1, b.data(), 1);
}

using FGPUVector = GPUVector<float>;
using RGPUVector = GPUVector<double>;

} // namespace cuda
} // namespace cl

#endif // CODELIBRARY_CUDA_GPU_VECTOR_H_
