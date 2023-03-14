//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_VECTOR_H_
#define CODELIBRARY_MATH_VECTOR_H_

#include "codelibrary/math/basic_linear_algebra.h"

namespace cl {

/**
 * N-dimension vector.
 */
template <typename T>
class Vector : public BasicLinearAlgebra<T> {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using value_type             = T;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    /**
     * Default vector constructor.
     */
    Vector() = default;

    /**
     * Construct a vector with initial value.
     */
    Vector(int size, const T& v = T(0)) : BasicLinearAlgebra<T>(size, v) {}

    /**
     * Construct vector from data in [first, last).
     *
     * The second template parameter is used to distinguish this function to
     * Vector(int, int).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Vector(Iter first, Iter last)
        : BasicLinearAlgebra<T>(first, last) {}

    /**
     * Build vector from initializer list.
     *
     * Note that this constructor can not be explicit.
     */
    Vector(std::initializer_list<T> list)
        : BasicLinearAlgebra<T>(list) {}

    explicit Vector(const Vector& rhs) = default;

    Vector(Vector&& rhs) = default;

    virtual ~Vector() = default;

    Vector& operator=(const Vector& rhs) {
        this->Reallocate(rhs.size());
        std::copy(rhs.begin(), rhs.end(), this->data());
        return *this;
    }

    Vector& operator *=(const T& rhs) {
        blas::Scale(this->size_, rhs, this->data_);
        return *this;
    }

    Vector& operator +=(const Vector& rhs) {
        CHECK(this->size_ == rhs.size_);

        blas::Add(this->size_, this->data_, rhs.data(), this->data_);
        return *this;
    }

    Vector& operator -=(const Vector& rhs) {
        CHECK(this->size_ == rhs.size_);

        blas::Subtract(this->size_, this->data_, rhs.data_, this->data_);
        return *this;
    }

    Vector& operator *=(const Vector& rhs) {
        CHECK(this->size_ == rhs.size_);

        blas::Multiply(this->size_, this->data_, rhs.data_, this->data_);
        return *this;
    }

    /**
     * Assign the vector.
     */
    void Assign(int n, const T& v = T(0)) {
        CHECK(n >= 0);

        this->Reallocate(n, v);
    }

    /**
     * Assign the vector to [first, last).
     */
    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    void Assign(Iter first, Iter last) {
        this->Reallocate(first, last);
    }

    /**
     * Resize the vector with the filling value.
     */
    void Resize(int n, const T& v = T(0)) {
        CHECK(n >= 0);

        this->Reshape(n, v);
    }

    /**
     * Compute the uary positive of a vector.
     */
    friend const Vector& operator+(const Vector& rhs) {
        return rhs;
    }

    /**
     * Compute the negative of a vector.
     */
    friend Vector operator -(const Vector& rhs) {
        Vector res = rhs;
        blas::Negate(res.size(), res.data());
        return res;
    }

    /**
     * Compute the product of a vector by a scalar.
     */
    friend Vector operator *(const Vector& lhs, const T& rhs) {
        Vector res = lhs;
        res *= rhs;
        return res;
    }

    /**
     * Compute the product of a scale by a vector.
     */
    friend Vector operator *(const T& lhs, const Vector& rhs) {
        Vector res = rhs;
        res *= lhs;
        return res;
    }

    /**
     * Perform element by element addition of vector a and vector b.
     */
    friend Vector operator +(const Vector& a, const Vector& b) {
        Vector c(a);
        c += b;
        return c;
    }

    /**
     * Perform element by element subtraction of vector b from vector a.
     */
    friend Vector operator -(const Vector& a, const Vector& b) {
        Vector c = a;
        c -= b;
        return c;
    }

    /**
     * Perform element by element multiplication of vector a and vector b.
     */
    friend Vector operator *(const Vector& a, const Vector& b) {
        Vector c = a;
        c *= b;
        return c;
    }

    /**
     * For debug.
     */
    friend std::ostream& operator <<(std::ostream& os, const Vector& rhs) {
        Message msg(rhs.begin(), rhs.end());
        os << msg;
        return os;
    }
};

using FVector = Vector<float>;
using RVector = Vector<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_VECTOR_H_
