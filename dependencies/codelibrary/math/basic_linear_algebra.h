//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Implement BLAS level 1 routines:
//   [1]. Scale, x = a*x
//   [2]. AXPY, y = a*x + y
//   [3]. Dot product
//   [4]. Euclidean norm
//   [5]. ASum, sum of absolute values
//   [6]. Matrix-vector product.
//
// And BLAS-like routines:
//   [1]. Unary negate for vector
//   [2]. Add two vectors.
//   [3]. Subtract two vectors.
//   [4]. Element multiplication of two vectors.
//

#ifndef CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_
#define CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <memory>

#include "codelibrary/base/log.h"

namespace cl {
namespace blas {

/**
 * Computes the product of a vector by a scalar, which is defined as:
 *
 *   b *= a.
 *
 * Input:
 *  n - Specify the number of elements in vector b.
 *  a - Specify the scalar a.
 *  b - Array, size at least n.
 *
 * Output:
 *  b - Update vector b.
 */
template <typename T>
void Scale(int n, T a, T* b) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: b[0] *= a;
            return;
    case 2: b[0] *= a;
            b[1] *= a;
            return;
    case 3: b[0] *= a;
            b[1] *= a;
            b[2] *= a;
            return;
    case 4: b[0] *= a;
            b[1] *= a;
            b[2] *= a;
            b[3] *= a;
            return;
    }

    T* p = b;

    int i = 0;
    for (; i + 4 < n; p += 4, i += 4) {
        *(p)     *= a;
        *(p + 1) *= a;
        *(p + 2) *= a;
        *(p + 3) *= a;
    }
    for (; i < n; ++p, ++i)
        *(p) *= a;
}

/**
 * Constant times a vector plus a vector, which is defined as:
 *
 *    c += a * b.
 *
 * Input:
 *  n - Specify the number of elements in vector b.
 *  a - Specify the scalar a.
 *  b - Array, size at least n.
 *
 * Output:
 *  c - Update vector b.
 */
template <typename T>
void AXPY(int n, T a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: c[0] += a * b[0];
            return;
    case 2: c[0] += a * b[0];
            c[1] += a * b[1];
            return;
    case 3: c[0] += a * b[0];
            c[1] += a * b[1];
            c[2] += a * b[2];
            return;
    case 4: c[0] += a * b[0];
            c[1] += a * b[1];
            c[2] += a * b[2];
            c[3] += a * b[3];
            return;
    }

    const T* p1 = b;

    T* p = c;
    int i = 0;
    for (; i + 4 < n; p1 += 4, p += 4, i += 4) {
        *(p)     += a * *(p1);
        *(p + 1) += a * *(p1 + 1);
        *(p + 2) += a * *(p1 + 2);
        *(p + 3) += a * *(p1 + 3);
    }
    for (; i < n; ++p1, ++p, ++i)
        *(p) += a * *(p1);
}

/**
 * Compute a vector-vector dot product.
 *
 * Input:
 *  n - Specify the number of elements in vectors x and y.
 *  a - Array, size at least n.
 *  b - Array, size at least n.
 *
 * Return:
 *  The result of the dot product of a and b, if n is positive.
 *  Otherwise, return 0.
 */
template <typename T>
T DotProduct(int n, const T* a, const T* b) {
    static_assert(std::is_floating_point<T>::value, "");

    if (n <= 0) return 0;

    switch (n) {
    case 1: return a[0] * b[0];
    case 2: return a[0] * b[0] + a[1] * b[1];
    case 3: return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    case 4: return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    }

    T s = 0;
    const T* p1 = a;
    const T* p2 = b;

    int i = 0;
    for (; i + 4 < n; p1 += 4, p2 += 4, i += 4) {
        s += *(p1)     * *(p2);
        s += *(p1 + 1) * *(p2 + 1);
        s += *(p1 + 2) * *(p2 + 2);
        s += *(p1 + 3) * *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++i)
        s += *(p1) * *(p2);
    return s;
}

/**
 * Compute the Euclidean norm of a vector.
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Array, size at least n.
 *
 * Return:
 *  The Euclidean norm of the vector a.
 */
template <typename T>
T EuclideanNorm(int n, const T* a) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return 0;
    case 1: return std::sqrt(a[0] * a[0]);
    case 2: return std::sqrt(a[0] * a[0] + a[1] * a[1]);
    case 3: return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    case 4: return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] +
                             a[3] * a[3]);
    }

    T sum = 0;
    const T* p = a;

    int i = 0;
    for (; i + 4 < n; p += 4, i += 4) {
        sum += *(p)     * *(p);
        sum += *(p + 1) * *(p + 1);
        sum += *(p + 2) * *(p + 2);
        sum += *(p + 3) * *(p + 3);
    }
    for (; i < n; ++p, ++i)
        sum += *(p) * *(p);

    return std::sqrt(sum);
}

/**
 * Compute the sum of absolute values of a vector.
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Array, size at least n.
 *
 * Return:
 *  The sum of absolute values of the vector a.
 */
template <typename T>
T ASum(int n, const T* a) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return 0;
    case 1: return std::fabs(a[0]);
    case 2: return std::fmax(std::fabs(a[0]), std::fabs(a[1]));
    case 3: return std::fmax(std::fabs(a[0]),
                             std::fmax(std::fabs(a[1]), std::fabs(a[2])));
    case 4: return std::fmax(std::fmax(std::fabs(a[0]), std::fabs(a[1])),
                             std::fmax(std::fabs(a[2]), std::fabs(a[3])));
    }

    T max_a = 0;
    const T* p = a;

    int i = 0;
    for (; i + 4 < n; p += 4, i += 4) {
        max_a = std::fmax(std::fabs(*(p)),     max_a);
        max_a = std::fmax(std::fabs(*(p + 1)), max_a);
        max_a = std::fmax(std::fabs(*(p + 2)), max_a);
        max_a = std::fmax(std::fabs(*(p + 3)), max_a);
    }
    for (; i < n; ++p, ++i)
        max_a = std::fmax(std::fabs(*(p)), max_a);

    return max_a;
}

/**
 * In-place negate, which is defined as:
 *
 *  a = -a
 *
 * Input:
 *  n - Specify the number of elements in vector a.
 *  a - Pointer to array that contains the input vector a.
 *
 * Output:
 *  a - Update vector a.
 */
template <typename T>
void Negate(int n, T* a) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: a[0] = -a[0];
            return;
    case 2: a[0] = -a[0];
            a[1] = -a[1];
            return;
    case 3: a[0] = -a[0];
            a[1] = -a[1];
            a[2] = -a[2];
            return;
    case 4: a[0] = -a[0];
            a[1] = -a[1];
            a[2] = -a[2];
            a[3] = -a[3];
            return;
    }

    T* p = a;

    int i = 0;
    for (; i + 4 < n; p += 4, i += 4) {
        *(p)     = -*(p);
        *(p + 1) = -*(p + 1);
        *(p + 2) = -*(p + 2);
        *(p + 3) = -*(p + 3);
    }
    for (; i < n; ++p, ++i)
        *(p) = -*(p);
}

/**
 * Perform element by element addition of two vectors, which is defined as:
 *
 *  c = a + b
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Add(int n, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] + b[0];
            return;
    case 2: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            return;
    case 3: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            c[2] = a[2] + b[2];
            return;
    case 4: c[0] = a[0] + b[0];
            c[1] = a[1] + b[1];
            c[2] = a[2] + b[2];
            c[3] = a[3] + b[3];
            return;
    }

    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 4 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     + *(p2);
        *(p + 1) = *(p1 + 1) + *(p2 + 1);
        *(p + 2) = *(p1 + 2) + *(p2 + 2);
        *(p + 3) = *(p1 + 3) + *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) + *(p2);
}

/**
 * Perform element by element subtraction of vector a and vector b.
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Subtract(int n, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] - b[0];
            return;
    case 2: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            return;
    case 3: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            c[2] = a[2] - b[2];
            return;
    case 4: c[0] = a[0] - b[0];
            c[1] = a[1] - b[1];
            c[2] = a[2] - b[2];
            c[3] = a[3] - b[3];
            return;
    }
    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 4 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     - *(p2);
        *(p + 1) = *(p1 + 1) - *(p2 + 1);
        *(p + 2) = *(p1 + 2) - *(p2 + 2);
        *(p + 3) = *(p1 + 3) - *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) - *(p2);
}

/**
 * Perform element by element multiplication of vector a and vector b.
 *
 * Input:
 *  n   - Specify the number of elements to be calculated.
 *  a,b - Pointers to arrays that contain the input vectors a and b.
 *
 * Output:
 *  c   - Pointer to an array that contains the output vector b.
 */
template <typename T>
void Multiply(int n, const T* a, const T* b, T* c) {
    static_assert(std::is_floating_point<T>::value, "");

    switch (n) {
    case 0: return;
    case 1: c[0] = a[0] * b[0];
            return;
    case 2: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            return;
    case 3: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            c[2] = a[2] * b[2];
            return;
    case 4: c[0] = a[0] * b[0];
            c[1] = a[1] * b[1];
            c[2] = a[2] * b[2];
            c[3] = a[3] * b[3];
            return;
    }

    const T* p1 = a;
    const T* p2 = b;

    T* p = c;
    int i = 0;
    for (; i + 4 < n; p1 += 4, p2 += 4, p += 4, i += 4) {
        *(p)     = *(p1)     * *(p2);
        *(p + 1) = *(p1 + 1) * *(p2 + 1);
        *(p + 2) = *(p1 + 2) * *(p2 + 2);
        *(p + 3) = *(p1 + 3) * *(p2 + 3);
    }
    for (; i < n; ++p1, ++p2, ++p, ++i)
        *(p) = *(p1) * *(p2);
}

} // namespace blas

/**
 * Basic linear algebra object.
 */
template <typename T>
class BasicLinearAlgebra {
    static_assert(std::is_floating_point<T>::value, "");

    static const int ALIGNMENT = 64; // Memory alignment.

public:
    using value_type             = T;
    using pointer                = T*;
    using const_pointer          = const T*;
    using reference              = T&;
    using const_reference        = const T&;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using size_type              = int;
    using difference_type        = int;

    /**
     * Default BasicLinearAlgebra constructor.
     */
    BasicLinearAlgebra() = default;

    explicit BasicLinearAlgebra(int size) {
        Allocate(size);
    }

    /**
     * Construct a BasicLinearAlgebra with initialize value.
     */
    BasicLinearAlgebra(int size, const T& v) {
        Allocate(size);
        std::uninitialized_fill_n(data_, size, v);
    }

    /**
     * Construct BasicLinearAlgebra from data in [first, last).
     * The second template parameter is used to distinguish this function to
     * BasicLinearAlgebra(int, int).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    BasicLinearAlgebra(Iter first, Iter last) {
        auto n = std::distance(first, last);
        assert(n >= 0);
        assert(n <= INT_MAX && "We only accept INT_MAX elements at most.");

        Allocate(static_cast<int>(n));
        std::uninitialized_copy(first, last, data_);
    }

    /**
     * Copy constructor.
     */
    explicit BasicLinearAlgebra(const BasicLinearAlgebra<T>& v) {
        Allocate(v.size_);
        std::uninitialized_copy(v.data_, v.data_ + v.size_, data_);
    }

    /**
     * Move constructor.
     */
    BasicLinearAlgebra(BasicLinearAlgebra<T>&& v) {
        swap(&v);
    }

    /**
     * Build BasicLinearAlgebra from initializer list.
     *
     * Note that this constructor can not be explicit.
     */
    explicit BasicLinearAlgebra(std::initializer_list<T> list)
        : BasicLinearAlgebra(list.begin(), list.end()) {}

    virtual ~BasicLinearAlgebra() {
        Deallocate(data_);
    }

    /**
     * Clear the BasicLinearAlgebra.
     */
    void clear() {
        size_ = 0;
    }

    /**
     * Check if this BasicLinearAlgebra is empty.
     */
    bool empty() const {
        return size_ == 0;
    }

    /**
     * Swap the data with another BasicLinearAlgebra.
     */
    void swap(BasicLinearAlgebra<T>* rhs) {
        std::swap(data_, rhs->data_);
        std::swap(size_, rhs->size_);
        std::swap(allocated_size_, rhs->allocated_size_);
    }

    /**
     * Return the number of elements.
     */
    int size() const {
        return size_;
    }

    T* data() {
        return data_;
    }

    const T* data() const {
        return data_;
    }

    iterator begin() {
        return data_;
    }

    iterator end() {
        return data_ + size_;
    }

    const_iterator begin() const {
        return data_;
    }

    const_iterator end() const {
        return data_ + size_;
    }

    reverse_iterator rbegin() {
        return reverse_iterator(end());
    }

    reverse_iterator rend() {
        return reverse_iterator(begin());
    }

    const_reverse_iterator rbegin() const {
        return const_reverse_iterator(end());
    }

    const_reverse_iterator rend() const {
        return const_reverse_iterator(begin());
    }

    T& operator[](int index) {
        return *(data_ + index);
    }

    const T& operator[](int index) const {
        return *(data_ + index);
    }

    T& at(int index) {
        CHECK(index >= 0 && index < size_);

        return *(data_ + index);
    }

    const T& at(int index) const {
        CHECK(index >= 0 && index < size_);

        return *(data_ + index);
    }

    /**
     * Move assignment.
     */
    BasicLinearAlgebra& operator =(BasicLinearAlgebra<T>&& rhs) noexcept {
        swap(&rhs);
        return *this;
    }

    /**
     * Copy assignment.
     */
    BasicLinearAlgebra& operator =(const BasicLinearAlgebra<T>& rhs) {
        if (this == &rhs) return *this; // This case happens often.

        Reallocate(rhs.size_);
        std::memcpy(data_, rhs.data_, sizeof(T) * size_);
        return *this;
    }

    bool operator ==(const BasicLinearAlgebra& rhs) const {
        return size_ == rhs.size_ &&
               std::equal(data_, data_ + size_, rhs.data_);
    }

    bool operator !=(const BasicLinearAlgebra& rhs) const {
        return !(*this == rhs);
    }

    /**
     * Fill the vector.
     */
    void Fill(const T& v) {
        if (data_) std::fill_n(data_, size_, v);
    }

    /**
     * Return euclidean norm.
     */
    T norm() const {
        return blas::EuclideanNorm(size_, data_);
    }

    /**
     * Return the squared euclidean norm.
     */
    T squared_norm() const {
        return blas::DotProduct(size_, data_, data_);
    }

    /**
     * Infinity norm.
     */
    T infinty_norm() const {
        return blas::ASum(size_, data_);
    }

protected:
    /**
     * Allocate an aligned memory buffer for BasicLinearAlgebra.
     */
    void Allocate(int size) {
        CHECK(size >= 0);

        if (size > 0) {
            int offset = ALIGNMENT - 1 + sizeof(void*);
            void* raw = std::malloc(sizeof(T) * size + offset);
            CHECK(raw) << "Malloc error, maybe memory is not enough.";

            void* buf = reinterpret_cast<void*>((reinterpret_cast<size_t>(raw) +
                                                 offset) & ~(ALIGNMENT - 1));
            (reinterpret_cast<void**>(buf))[-1] = raw;
            data_ = reinterpret_cast<T*>(buf);
            CHECK(data_) << "Malloc error, maybe memory is not enough.";
        }
        allocated_size_ = size_ = size;
    }

    /**
     * Free the memory buffer.
     */
    void Deallocate(T* ptr) {
        if (ptr) free(reinterpret_cast<void*>((reinterpret_cast<void**>
                      (ptr))[-1]));
    }

    /**
     * Reallocate data.
     */
    void Reallocate(int n) {
        CHECK(n >= 0);

        if (n > allocated_size_) {
            Deallocate(data_);
            Allocate(n);
        } else {
            size_ = n;
        }
    }

    /**
     * Reallocate and fill.
     */
    void Reallocate(int n, const T& v) {
        CHECK(n >= 0);

        if (n == 0) {
            clear();
        } else if (n > allocated_size_) {
            // Need relocate the data.
            Reallocate(n);
            std::uninitialized_fill_n(data_, n, v);
        } else {
            size_ = n;
            std::fill_n(data_, n, v);
        }
    }

    /**
     * Reallocate and fill the BasicLinearAlgebra with data [first, last).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    void Reallocate(Iter first, Iter last) {
        auto n = std::distance(first, last);
        CHECK(n >= 0);
        CHECK(n <= INT_MAX) << "We only accept INT_MAX elements at most.";

        Reallocate(static_cast<int>(n));
        std::copy(first, last, data_);
    }

    /**
     * Resize the BLAS without fill the value.
     */
    void Reshape(int n) {
        CHECK(n >= 0);

        if (allocated_size_ == 0) {
            Allocate(n);
        } else {
            if (n > allocated_size_) {
                T* tmp = data_;
                int old_size = size_;

                Allocate(n);
                std::uninitialized_copy(tmp, tmp + old_size, data_);
                Deallocate(tmp);
            } else {
                size_ = n;
            }
        }
    }

    /**
     * Resize the BLAS with the filling value.
     */
    void Reshape(int n, const T& v) {
        CHECK(n >= 0);

        if (allocated_size_ == 0) {
            Allocate(n);
            std::fill(data_, data_ + n, v);
        } else {
            if (n > allocated_size_) {
                T* tmp = data_;
                int old_size = size_;

                Allocate(n);
                std::uninitialized_copy(tmp, tmp + old_size, data_);
                std::uninitialized_fill(data_ + old_size, data_ + n, v);
                Deallocate(tmp);
            } else if (n > size_) {
                std::fill(data_ + size_, data_ + n, v);
                size_ = n;
            } else {
                size_ = n;
            }
        }
    }

    // Number of elements in the BasicLinearAlgebra.
    int size_ = 0;

    // Allocated size of BasicLinearAlgebra.
    int allocated_size_ = 0;

    // Store data.
    T* data_  = nullptr;
};

} // namespace cl

#endif // CODELIBRARY_MATH_BASIC_LINEAR_ALGEBRA_H_
