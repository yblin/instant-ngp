//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_MATRIX4_H_
#define CODELIBRARY_MATH_MATRIX_MATRIX4_H_

#include <cmath>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/vector_4d.h"

// Unrolling the loop can speed up by 2 times.
#define UNROLL_MATRIX4_OPERATION1(a, b, op) \
    a[0] op b;  \
    a[1] op b;  \
    a[2] op b;  \
    a[3] op b;  \
    a[4] op b;  \
    a[5] op b;  \
    a[6] op b;  \
    a[7] op b;  \
    a[8] op b;  \
    a[9] op b;  \
    a[10] op b; \
    a[11] op b; \
    a[12] op b; \
    a[13] op b; \
    a[14] op b; \
    a[15] op b

#define UNROLL_MATRIX4_OPERATION2(a, b, op) \
    a[0] op b[0];   \
    a[1] op b[1];   \
    a[2] op b[2];   \
    a[3] op b[3];   \
    a[4] op b[4];   \
    a[5] op b[5];   \
    a[6] op b[6];   \
    a[7] op b[7];   \
    a[8] op b[8];   \
    a[9] op b[9];   \
    a[10] op b[10]; \
    a[11] op b[11]; \
    a[12] op b[12]; \
    a[13] op b[13]; \
    a[14] op b[14]; \
    a[15] op b[15]

namespace cl {

/**
 * 4 x 4 matrix.
 */
template <typename T>
class Matrix4 {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using iterator = T*;
    using const_iterator = const T*;

    Matrix4() {
        std::fill(data_ + 0, data_ + 16, T(0));
    }

    explicit Matrix4(const Array<T>& data) {
        Assign(data);
    }

    /**
     * Support assign operator:
     *
     *   Matrix4 mat = {
     *       {1,  2,   3,  4},
     *       {5,  6,   7,  8},
     *       {9, 10,  11, 12},
     *       {13, 14, 15, 16}
     *   };
     */
    Matrix4(const Array<T>& row0, const Array<T>& row1,
            const Array<T>& row2, const Array<T>& row3) {
        CHECK(row0.size() == 4);
        CHECK(row1.size() == 4);
        CHECK(row2.size() == 4);
        CHECK(row3.size() == 4);

        data_[0] = row0[0];
        data_[1] = row0[1];
        data_[2] = row0[2];
        data_[3] = row0[3];
        data_[4] = row1[0];
        data_[5] = row1[1];
        data_[6] = row1[2];
        data_[7] = row1[3];
        data_[8] = row2[0];
        data_[9] = row2[1];
        data_[10] = row2[2];
        data_[11] = row2[3];
        data_[12] = row3[0];
        data_[13] = row3[1];
        data_[14] = row3[2];
        data_[15] = row3[3];
    }

    Matrix4(T m0, T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8,
            T m9, T m10, T m11, T m12, T m13, T m14, T m15) {
        Assign(m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13,
               m14, m15);
    }

    Matrix4(const Matrix4& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 16, data_);
    }

    /**
     * Assign matrix.
     */
    void Assign(const Array<T>& data) {
        CHECK(data.size() == 16);

        std::copy(data.begin(), data.end(), data_);
    }

    /**
     * Assign matrix.
     */
    void Assign(T m0, T m1, T m2, T m3, T m4, T m5, T m6, T m7, T m8,
                T m9, T m10, T m11, T m12, T m13, T m14, T m15) {
        data_[0] = m0;
        data_[1] = m1;
        data_[2] = m2;
        data_[3] = m3;
        data_[4] = m4;
        data_[5] = m5;
        data_[6] = m6;
        data_[7] = m7;
        data_[8] = m8;
        data_[9] = m9;
        data_[10] = m10;
        data_[11] = m11;
        data_[12] = m12;
        data_[13] = m13;
        data_[14] = m14;
        data_[15] = m15;
    }

    /**
     * Return a new identity matrix.
     */
    static Matrix4 Identity() {
        Matrix4 mat;
        mat.data_[0] = mat.data_[5] = mat.data_[10] = mat.data_[15] = 1;
        return mat;
    }

    /**
     * Setup an identity matrix.
     */
    void SetIdentity() {
        std::fill(data_ + 0, data_ + 16, T(0));
        data_[0] = data_[5] = data_[10] = data_[15] = 1;
    }

    /**
     * Return the inverse transform.
     */
    Matrix4 Inverse() const {
        Matrix4 t;
        T* dst = t.data();
        const T* src = data_;

        // Compute adjoint:
        dst[0] = src[ 5] * src[10] * src[15] - src[ 5] * src[11] * src[14] -
                 src[ 9] * src[ 6] * src[15] + src[ 9] * src[ 7] * src[14] +
                 src[13] * src[ 6] * src[11] - src[13] * src[ 7] * src[10];

        dst[1] = -src[ 1] * src[10] * src[15] + src[ 1] * src[11] * src[14] +
                  src[ 9] * src[ 2] * src[15] - src[ 9] * src[ 3] * src[14] -
                  src[13] * src[ 2] * src[11] + src[13] * src[ 3] * src[10];

        dst[2] = src[ 1] * src[ 6] * src[15] - src[ 1] * src[ 7] * src[14] -
                 src[ 5] * src[ 2] * src[15] + src[ 5] * src[ 3] * src[14] +
                 src[13] * src[ 2] * src[ 7] - src[13] * src[ 3] * src[ 6];

        dst[3] = -src[ 1] * src[ 6] * src[11] + src[ 1] * src[ 7] * src[10] +
                  src[ 5] * src[ 2] * src[11] - src[ 5] * src[ 3] * src[10] -
                  src[ 9] * src[ 2] * src[ 7] + src[ 9] * src[ 3] * src[ 6];

        dst[4] = -src[ 4] * src[10] * src[15] + src[ 4] * src[11] * src[14] +
                  src[ 8] * src[ 6] * src[15] - src[ 8] * src[ 7] * src[14] -
                  src[12] * src[ 6] * src[11] + src[12] * src[ 7] * src[10];

        dst[5] = src[ 0] * src[10] * src[15] - src[ 0] * src[11] * src[14] -
                 src[ 8] * src[ 2] * src[15] + src[ 8] * src[ 3] * src[14] +
                 src[12] * src[ 2] * src[11] - src[12] * src[ 3] * src[10];

        dst[6] = - src[ 0] * src[ 6] * src[15] + src[ 0] * src[ 7] * src[14] +
                   src[ 4] * src[ 2] * src[15] - src[ 4] * src[ 3] * src[14] -
                   src[12] * src[ 2] * src[ 7] + src[12] * src[ 3] * src[ 6];

        dst[7] = src[ 0] * src[ 6] * src[11] - src[ 0] * src[ 7] * src[10] -
                 src[ 4] * src[ 2] * src[11] + src[ 4] * src[ 3] * src[10] +
                 src[ 8] * src[ 2] * src[ 7] - src[ 8] * src[ 3] * src[ 6];

        dst[8] = src[ 4] * src[ 9] * src[15] - src[ 4] * src[11] * src[13] -
                 src[ 8] * src[ 5] * src[15] + src[ 8] * src[ 7] * src[13] +
                 src[12] * src[ 5] * src[11] - src[12] * src[ 7] * src[ 9];

        dst[9] = -src[ 0] * src[ 9] * src[15] + src[ 0] * src[11] * src[13] +
                  src[ 8] * src[ 1] * src[15] - src[ 8] * src[ 3] * src[13] -
                  src[12] * src[ 1] * src[11] + src[12] * src[ 3] * src[ 9];

        dst[10] = src[ 0] * src[ 5] * src[15] - src[ 0] * src[ 7] * src[13] -
                  src[ 4] * src[ 1] * src[15] + src[ 4] * src[ 3] * src[13] +
                  src[12] * src[ 1] * src[ 7] - src[12] * src[ 3] * src[ 5];

        dst[11] = -src[ 0] * src[ 5] * src[11] + src[ 0] * src[ 7] * src[ 9] +
                   src[ 4] * src[ 1] * src[11] - src[ 4] * src[ 3] * src[ 9] -
                   src[ 8] * src[ 1] * src[ 7] + src[ 8] * src[ 3] * src[ 5];

        dst[12] = -src[ 4] * src[ 9] * src[14] + src[ 4] * src[10] * src[13] +
                   src[ 8] * src[ 5] * src[14] - src[ 8] * src[ 6] * src[13] -
                   src[12] * src[ 5] * src[10] + src[12] * src[ 6] * src[ 9];

        dst[13] = src[ 0] * src[ 9] * src[14] - src[ 0] * src[10] * src[13] -
                  src[ 8] * src[ 1] * src[14] + src[ 8] * src[ 2] * src[13] +
                  src[12] * src[ 1] * src[10] - src[12] * src[ 2] * src[ 9];

        dst[14] = -src[ 0] * src[ 5] * src[14] + src[ 0] * src[ 6] * src[13] +
                   src[ 4] * src[ 1] * src[14] - src[ 4] * src[ 2] * src[13] -
                   src[12] * src[ 1] * src[ 6] + src[12] * src[ 2] * src[ 5];

        dst[15] = src[ 0] * src[ 5] * src[10] - src[ 0] * src[ 6] * src[ 9] -
                  src[ 4] * src[ 1] * src[10] + src[ 4] * src[ 2] * src[ 9] +
                  src[ 8] * src[ 1] * src[ 6] - src[ 8] * src[ 2] * src[ 5];

        // Compute determinant:
        T det = src[0] * dst[0] + src[1] * dst[4] + src[2] * dst[8] +
                src[3] * dst[12];

        // Multiply adjoint with reciprocal of determinant:
        t *= T(1) / det;

        return t;
    }

    /**
     * Return the transpose transform.
     */
    Matrix4 Transpose() const {
        Matrix4 t;
        t.data_[1]  = data_[4];
        t.data_[2]  = data_[8];
        t.data_[3]  = data_[12];
        t.data_[4]  = data_[1];
        t.data_[6]  = data_[9];
        t.data_[7]  = data_[13];
        t.data_[8]  = data_[2];
        t.data_[9]  = data_[6];
        t.data_[11] = data_[14];
        t.data_[12] = data_[3];
        t.data_[13] = data_[7];
        t.data_[14] = data_[11];
        return t;
    }

    // O(1) access the members.
    const T* data()        const { return data_;      }
    T* data()                    { return data_;      }
    T operator[] (int i)   const { return data_[i];   }
    T& operator[] (int i)        { return data_[i];   }
    int n_rows()           const { return 4;          }
    int n_columns()        const { return 4;          }
    iterator begin()             { return data_;      }
    const_iterator begin() const { return data_;      }
    iterator end()               { return data_ + 16; }
    const_iterator end()   const { return data_ + 16; }

    const T& operator() (int i, int j) const {
        return data_[i * 4 + j];
    }

    T& operator() (int i, int j) {
        return data_[i * 4 + j];
    }

    Matrix4& operator =(const Matrix4& rhs) {
        std::copy(rhs.data_ + 0, rhs.data_ + 16, data_);
        return *this;
    }

    Matrix4& operator =(Matrix4&& rhs) noexcept {
        std::swap(data_, rhs.data_);
        return *this;
    }

    Matrix4& operator+=(const Matrix4& rhs) {
        UNROLL_MATRIX4_OPERATION2(data_, rhs, +=);
        return *this;
    }
    Matrix4& operator-=(const Matrix4& rhs) {
        UNROLL_MATRIX4_OPERATION2(data_, rhs, -=);
        return *this;
    }
    Matrix4& operator*=(T rhs) {
        UNROLL_MATRIX4_OPERATION1(data_, rhs, *=);
        return *this;
    }

    friend Matrix4 operator *(const Matrix4& lhs, const T& rhs) {
        Matrix4 res = lhs;
        res *= rhs;
        return res;
    }

    friend Matrix4 operator *(const T& lhs, const Matrix4& rhs) {
        Matrix4 res = rhs;
        res *= lhs;
        return res;
    }

    friend Matrix4 operator +(const Matrix4& a, const Matrix4& b) {
        Matrix4 res = a;
        res += b;
        return res;
    }

    friend Matrix4 operator -(const Matrix4& a, const Matrix4& b) {
        Matrix4 res = a;
        res -= b;
        return res;
    }

    friend Matrix4 operator *(const Matrix4& lhs, const Matrix4& rhs) {
        Matrix4 res;
        T* c = res.data();
        const T* a = lhs.data();
        const T* b = rhs.data();

        c[0] = a[0] * b[0] + a[1] * b[4] + a[2] * b[8]  + a[3] * b[12];
        c[1] = a[0] * b[1] + a[1] * b[5] + a[2] * b[9]  + a[3] * b[13];
        c[2] = a[0] * b[2] + a[1] * b[6] + a[2] * b[10] + a[3] * b[14];
        c[3] = a[0] * b[3] + a[1] * b[7] + a[2] * b[11] + a[3] * b[15];

        c[4] = a[4] * b[0] + a[5] * b[4] + a[6] * b[8]  + a[7] * b[12];
        c[5] = a[4] * b[1] + a[5] * b[5] + a[6] * b[9]  + a[7] * b[13];
        c[6] = a[4] * b[2] + a[5] * b[6] + a[6] * b[10] + a[7] * b[14];
        c[7] = a[4] * b[3] + a[5] * b[7] + a[6] * b[11] + a[7] * b[15];

        c[8]  = a[8] * b[0] + a[9] * b[4] + a[10] * b[8]  + a[11] * b[12];
        c[9]  = a[8] * b[1] + a[9] * b[5] + a[10] * b[9]  + a[11] * b[13];
        c[10] = a[8] * b[2] + a[9] * b[6] + a[10] * b[10] + a[11] * b[14];
        c[11] = a[8] * b[3] + a[9] * b[7] + a[10] * b[11] + a[11] * b[15];

        c[12] = a[12] * b[0] + a[13] * b[4] + a[14] * b[8]  + a[15] * b[12];
        c[13] = a[12] * b[1] + a[13] * b[5] + a[14] * b[9]  + a[15] * b[13];
        c[14] = a[12] * b[2] + a[13] * b[6] + a[14] * b[10] + a[15] * b[14];
        c[15] = a[12] * b[3] + a[13] * b[7] + a[14] * b[11] + a[15] * b[15];

        return res;
    }

    friend Vector4D<T> operator *(const Matrix4& mat, const Vector4D<T>& v) {
        Vector4D<T> res;

        const T* p = mat.data_;
        res.x = p[0]  * v.x + p[1]  * v.y + p[2]  * v.z + p[3]  * v.w;
        res.y = p[4]  * v.x + p[5]  * v.y + p[6]  * v.z + p[7]  * v.w;
        res.z = p[8]  * v.x + p[9]  * v.y + p[10] * v.z + p[11] * v.w;
        res.w = p[12] * v.x + p[13] * v.y + p[14] * v.z + p[15] * v.w;

        return res;
    }

    friend Vector4D<T> operator *(const Vector4D<T>& v, const Matrix4& mat) {
        Vector4D<T> res;
        const T* p = mat.data_;
        res.x = p[0] * v.x + p[4] * v.y + p[8]  * v.z + p[12] * v.w;
        res.y = p[1] * v.x + p[5] * v.y + p[9]  * v.z + p[13] * v.w;
        res.z = p[2] * v.x + p[6] * v.y + p[10] * v.z + p[14] * v.w;
        res.w = p[3] * v.x + p[7] * v.y + p[11] * v.z + p[15] * v.w;
        return res;
    }

    /**
     * Output the matrix, just for debug.
     */
    friend Message& operator <<(Message& msg, const Matrix4& rhs) {
        return msg.Append(rhs.data_, rhs.data_ + 16, {4, 4});
    }

protected:
    T data_[16];
};

using FMatrix4 = Matrix4<float>;
using RMatrix4 = Matrix4<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_MATRIX4_H_
