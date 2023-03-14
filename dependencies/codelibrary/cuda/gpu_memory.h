//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CUDA_GPU_MEMORY_H_
#define CODELIBRARY_CUDA_GPU_MEMORY_H_

#include <algorithm>

#include <cuda_runtime.h>

#include "codelibrary/cuda/check.h"

namespace cl {
namespace cuda {

/**
 * Managed memory on GPU Device.
 */
template <class T>
class GPUMemory {
public:
    GPUMemory() = default;

    GPUMemory(int size) {
        this->Allocate(size);
    }

    GPUMemory(GPUMemory<T>&& other) noexcept = default;

    explicit GPUMemory(const GPUMemory<T>& other)
        : size_(other.size_) {
        this->Allocate(size_);

        CUDA_CHECK(cudaMemcpy(data_, other.data_, size_t(size_) * sizeof(T),
                              cudaMemcpyDeviceToDevice));
    }

    virtual ~GPUMemory() {
        this->clear();
    }

    GPUMemory<T>& operator=(const GPUMemory<T>& other) = delete;

    GPUMemory<T>& operator=(GPUMemory<T>&& rhs) noexcept {
        std::swap(size_, rhs.size_);
        std::swap(data_, rhs.data_);
        return *this;
    }

    /**
     * Resize the memory to the exact new size, even if it is already larger.
     */
    void Resize(int size) {
        CHECK(size >= 0);

        if (size_ == size) return;

        this->clear();
        this->Allocate(size);
    }

    /**
     * Fill all the memory of the value 'v'.
     */
    void Fill(const T& v) {
        CUDA_CHECK(cudaMemset(data_, size_, v));
    }

    /**
     * Copy 'n' data from the host.
     */
    void CopyFromHost(const T* data, int n) {
        CHECK(n <= size_);
        CHECK(n >= 0);

        CUDA_CHECK(cudaMemcpy(data_, data, size_t(n) * sizeof(T),
                              cudaMemcpyHostToDevice));
    }

    /**
     * Copy data from the device.
     */
    void CopyFromDevice(const GPUMemory& rhs) {
        CHECK(rhs.size() <= size_);

        CUDA_CHECK(cudaMemcpy(data_, rhs.data_, size_t(size_) * sizeof(T),
                              cudaMemcpyDeviceToDevice));
    }

    /**
     * Copy data from device to host.
     */
    void CopyToHost(T* data) {
        CUDA_CHECK(cudaMemcpy(data, data_, size_t(size_) * sizeof(T),
                              cudaMemcpyDeviceToHost));
    }

    /**
     * Free the memory.
     */
    void clear() {
        if (!data_) return;

        CUDA_CHECK(cudaFree(data_));
        size_ = 0;
        data_ = nullptr;
    }

    /**
     * Return true if the memory is not allocated.
     */
    bool empty() const {
        return size_ == 0;
    }

    /**
     * Return the number of allocated elements.
     */
    int size() const {
        return size_;
    }

    /**
     * Return the allocated data.
     */
    const T* data() const {
        return data_;
    }

    /**
     * Return the allocated data.
     */
    T* data() {
        return data_;
    }

private:
    /**
     * Allocate the memory for 'n' elements.
     *
     * Note that cudaMalloc can automatically align the memory.
     */
    void Allocate(int n) {
        CHECK(n >= 0);

        size_ = n;
        if (size_ == 0) return;

        CUDA_CHECK(cudaMalloc(&data_, size_ * sizeof(T)));
    }

    // The number of allocated elements in data.
    int size_ = 0;

    // GPU pointer of data.
    T* data_ = nullptr;
};

} // namespace cuda
} // namespace cl

#endif // CODELIBRARY_CUDA_GPU_MEMORY_H_
