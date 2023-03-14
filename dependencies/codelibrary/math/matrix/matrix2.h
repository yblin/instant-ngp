//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_MATRIX2_H_
#define CODELIBRARY_MATH_MATRIX_MATRIX2_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/vector_2d.h"

namespace cl {

/**
 * 2 x 2 matrix.
 */
template <typename T>
class Matrix2 {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using iterator = T*;
    using const_iterator = const T*;

    Matrix2() {
        std::fill(data_ + 0, data_ + 4, T(0));
    }

    explicit Matrix2(const Array<T>& data) {
        CHECK(data_.size() == 4);

        std::copy(data.begin(), data.end(), data_);
    }

    /**
     * Support assign operator:
     *
     *   Matrix2 mat = {
     *       {1, 2},
     *       {3, 4}
     *   };
     */
    Matrix2(const Array<T>& row0, const Array<T>& row1) {
        CHECK(row0.size() == 2);
        CHECK(row1.size() == 2);

        data_[0] = row0[0];
        data_[1] = row0[1];
        data_[2] = row1[0];
        data_[3] = row1[1];
    }

    Matrix2(T m0, T m1, T m2, T m3) {
        data_[0] = m0;
        data_[1] = m1;
        data_[2] = m2;
        data_[3] = m3;
    }

    Matrix2(const Matrix2& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 4, data_);
    }

    static Matrix2 Identity() {
        Matrix2 mat;
        mat[0] = mat[3] = 1;
        return mat;
    }

    /**
     * Return the inverse transform.
     */
    Matrix2 Inverse() const {
        Matrix2 t;
        T* dst = t.data();
        T* src = data_;

        // Compute adjoint:
        dst[0] =  src[3];
        dst[1] = -src[1];
        dst[2] = -src[2];
        dst[3] =  src[0];

        // Compute determinant:
        T det = src[0] * dst[0] + src[1] * dst[2];

        // Multiply adjoint with reciprocal of determinant:
        T norm = T(1) / det;
        dst[0] *= norm;
        dst[1] *= norm;
        dst[2] *= norm;
        dst[3] *= norm;
        return t;
    }

    /**
     * Return the transpose transform.
     */
    Matrix2 Transpose() const {
        Matrix2 t;
        t.data_[1] = data_[2];
        t.data_[2] = data_[1];
        return t;
    }

    // O(1) access the members.
    const T* data()        const { return data_;     }
    T* data()                    { return data_;     }
    T operator[] (int i)   const { return data_[i];  }
    T& operator[] (int i)        { return data_[i];  }
    int n_rows()           const { return 2;         }
    int n_columns()        const { return 2;         }
    iterator begin()             { return data_;     }
    const_iterator begin() const { return data_;     }
    iterator end()               { return data_ + 4; }
    const_iterator end()   const { return data_ + 4; }

    const T& operator() (int i, int j) const {
        return data_[i * 2 + j];
    }

    T& operator() (int i, int j) {
        return data_[i * 2 + j];
    }

    Matrix2& operator =(const Matrix2& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 4, data_);
        return *this;
    }

    Matrix2& operator =(Matrix2&& rhs) noexcept {
        std::swap(data_, rhs.data_);
        return *this;
    }

    Matrix2& operator+=(const Matrix2& rhs) {
        data_[0] += rhs.data_[0];
        data_[1] += rhs.data_[1];
        data_[2] += rhs.data_[2];
        data_[3] += rhs.data_[3];
        return *this;
    }
    Matrix2& operator-=(const Matrix2& rhs) {
        data_[0] -= rhs.data_[0];
        data_[1] -= rhs.data_[1];
        data_[2] -= rhs.data_[2];
        data_[3] -= rhs.data_[3];
        return *this;
    }
    Matrix2& operator*=(T rhs) {
        data_[0] *= rhs;
        data_[1] *= rhs;
        data_[2] *= rhs;
        data_[3] *= rhs;
        return *this;
    }

    friend Matrix2 operator *(const Matrix2& lhs, const T& rhs) {
        Matrix2 res = lhs;
        res *= rhs;
        return res;
    }

    friend Matrix2 operator *(const T& lhs, const Matrix2& rhs) {
        Matrix2 res = rhs;
        res *= lhs;
        return res;
    }

    friend Matrix2 operator +(const Matrix2& a, const Matrix2& b) {
        Matrix2 res = a;
        res += b;
        return res;
    }

    friend Matrix2 operator -(const Matrix2& a, const Matrix2& b) {
        Matrix2 res = a;
        res -= b;
        return res;
    }

    friend Matrix2 operator *(const Matrix2& a, const Matrix2& b) {
        Matrix2 c;
        c.data[0] = a.data[0] * b.data[0] + a.data[2] * b.data[1];
        c.data[1] = a.data[1] * b.data[0] + a.data[3] * b.data[1];
        c.data[2] = a.data[0] * b.data[2] + a.data[2] * b.data[3];
        c.data[3] = a.data[1] * b.data[2] + a.data[3] * b.data[3];
        return c;
    }

    friend Vector2D<T> operator *(const Matrix2& mat, const Vector2D<T>& v) {
        Vector2D<T> res;
        const T* data = mat.data_;
        res.x = data[0] * v.x + data[1] * v.y;
        res.y = data[2] * v.x + data[3] * v.y;
        return res;
    }

    friend Vector2D<T> operator *(const Vector2D<T>& v, const Matrix2& mat) {
        Vector2D<T> res;
        const T* data = mat.data_;
        res.x = data[0] * v.x + data[2] * v.y;
        res.y = data[1] * v.x + data[3] * v.y;
        return res;
    }

    /**
     * Just for debug.
     */
    friend Message& operator <<(Message& msg, const Matrix2& rhs) {
        return msg.Append(rhs.data_, rhs.data_ + 4, {2, 2});
    }

protected:
    T data_[4];
};

using FMatrix2 = Matrix2<float>;
using RMatrix2 = Matrix2<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_MATRIX2_H_
