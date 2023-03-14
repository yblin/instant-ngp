//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_MATRIX_MATRIX_H_
#define CODELIBRARY_MATH_MATRIX_MATRIX_H_

#include "codelibrary/base/array.h"
#include "codelibrary/base/array_nd.h"
#include "codelibrary/math/basic_linear_algebra.h"
#include "codelibrary/math/matrix/gemm.h"
#include "codelibrary/math/matrix/gemv.h"
#include "codelibrary/math/vector.h"

namespace cl {

/**
 * Row-major matrix.
 *
 * For small scale only, for large matrices see GPUMatrix.
 */
template <typename T>
class Matrix : public BasicLinearAlgebra<T> {
    static_assert(std::is_floating_point<T>::value, "");

public:
    using value_type             = T;
    using iterator               = T*;
    using const_iterator         = const T*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    Matrix() = default;

    /**
     * Construct a vector without initialization.
     */
    Matrix(int n_rows, int n_columns, const T& v = 0)
        : BasicLinearAlgebra<T>(n_rows * n_columns, v),
          n_rows_(n_rows), n_columns_(n_columns) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";
    }

    template <typename Iterator>
    Matrix(int n_rows, int n_columns, Iterator first, Iterator last)
        : BasicLinearAlgebra<T>(first, last),
          n_rows_(n_rows), n_columns_(n_columns) {
        CHECK(n_rows_ * n_columns_ == this->size());
    }

    Matrix(int n_rows, int n_columns, const Array<T>& data)
        : BasicLinearAlgebra<T>(data.begin(), data.end()),
          n_rows_(n_rows), n_columns_(n_columns) {
        CHECK(n_rows_ * n_columns_ == this->size());
    }

    explicit Matrix(const ArrayND<T>& data)
        : BasicLinearAlgebra<T>(data.begin(), data.end()) {
        CHECK(data.n_dimension() == 2);

        n_rows_ = data.shape(0);
        n_columns_ = data.shape(1);
    }

    explicit Matrix(const Matrix& rhs) = default;

    Matrix(Matrix&& rhs) = default;

    virtual ~Matrix() = default;

    Matrix& operator =(const Matrix&) = default;

    Matrix& operator =(Matrix&&) = default;

    Matrix& operator *=(const T& rhs) {
        blas::Scale(this->size_, rhs, this->data_);
        return *this;
    }

    Matrix& operator +=(const Matrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_);
        CHECK(n_columns_ == rhs.n_columns_);

        blas::Add(this->size_, this->data_, rhs.data(), this->data_);
        return *this;
    }

    Matrix& operator -=(const Matrix& rhs) {
        CHECK(n_rows_ == rhs.n_rows_);
        CHECK(n_columns_ == rhs.n_columns_);

        blas::Subtract(this->size_, this->data_, rhs.data_, this->data_);
        return *this;
    }

    const T& operator() (int i, int j) const {
        return this->data_[i * n_columns_ + j];
    }

    T& operator() (int i, int j) {
        return this->data_[i * n_columns_ + j];
    }

    /**
     * Assign the matrix.
     */
    void Assign(int n_rows, int n_columns, const T& v = T(0)) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";

        n_rows_ = n_rows;
        n_columns_ = n_columns;
        this->Reallocate(n_rows * n_columns, v);
    }

    /**
     * this = [first, last).
     */
    template <typename Iterator>
    void Assign(int n_rows, int n_columns, Iterator first, Iterator last) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";

        n_rows_ = n_rows;
        n_columns_ = n_columns;
        this->Reallocate(first, last);
    }

    /**
     * Assign the matrix.
     */
    void Assign(const ArrayND<T>& data) {
        CHECK(data.n_dimension() == 2);

        n_rows_ = data.shape(0);
        n_columns_ = data.shape(1);
        this->Reallocate(data.begin(), data.end());
    }

    /**
     * Reshape the matrix with the given value.
     */
    void Resize(int n_rows, int n_columns, const T& value = 0) {
        CHECK(n_rows >= 0);
        CHECK(n_columns >= 0);
        CHECK(n_columns == 0 || n_rows <= INT_MAX / n_columns) <<
              "The given dimensions of matrix are too large.";

        if (n_columns == n_columns_) {
            n_rows_ = n_rows;
            this->Reshape(n_rows_ * n_columns_, value);
        } else {
            Array<T> tmp(this->begin(), this->end());
            this->Assign(n_rows * n_columns, value);

            int min_n_rows = std::min(n_rows_, n_rows);
            int min_n_columns = std::min(n_columns_, n_columns);
            for (int i = 0; i < min_n_rows; ++i) {
                std::copy_n(this->data() + i * n_columns, min_n_columns,
                            tmp.begin() + i * n_columns_);
            }
            n_rows_ = n_rows;
            n_columns_ = n_columns;
        }
    }

    /**
     * Return the transpose matrix.
     */
    Matrix Transpose() const {
        Matrix b(n_columns_, n_rows_);
        int i = 0, j = 0;
        int n = n_rows_ * n_columns_;
        for (int t = 0; t < n; ++t) {
            b[t] = this->operator[](n_rows_ * j + i);
            if (++j == n_columns_) {
                j = 0;
                ++i;
            }
        }
        return b;
    }

    friend const Matrix& operator+(const Matrix& rhs) {
        return rhs;
    }

    friend Matrix operator -(const Matrix& rhs) {
        Matrix res = rhs;
        blas::Negate(res.size(), res.data());
        return res;
    }

    friend Matrix operator *(const Matrix& lhs, const T& rhs) {
        Matrix res = lhs;
        res *= rhs;
        return res;
    }

    friend Matrix operator *(const T& lhs, const Matrix& rhs) {
        Matrix res = rhs;
        res *= lhs;
        return res;
    }

    friend Matrix operator +(const Matrix& a, const Matrix& b) {
        Matrix c = a;
        c += b;
        return c;
    }

    friend Matrix operator -(const Matrix& a, const Matrix& b) {
        Matrix c = a;
        c -= b;
        return c;
    }

    friend Vector<T> operator *(const Matrix<T>& a, const Vector<T>& b) {
        CHECK(a.n_columns_ == b.size());

        Vector<T> c(a.n_rows_);
        blas::GEMV(a.n_rows_, a.n_columns_, a.data_, b.data(), c.data());
        return c;
    }

    friend Vector<T> operator *(const Vector<T>& b, const Matrix<T>& a) {
        CHECK(a.n_rows_ == b.size());

        Vector<T> c(a.n_columns_);
        blas::GEMVTrans(a.n_rows_, a.n_columns_, a.data_, b.data(), c.data());
        return c;
    }

    friend Matrix operator *(const Matrix& a, const Matrix& b) {
        CHECK(a.n_columns_ == b.n_rows_);

        Matrix c(a.n_rows_, b.n_columns_);
        blas::GEMM(a.n_rows_, a.n_columns_, b.n_columns_, a.data_, b.data_,
                   c.data_);
        return c;
    }

    /**
     * For debug.
     */
    friend std::ostream& operator <<(std::ostream& os, const Matrix& rhs) {
        if (rhs.empty()) return os;

        ArrayND<T> t(rhs.n_rows_, rhs.n_columns_);
        t.assign(rhs.begin(), rhs.end());
        Message msg;
        msg << t;

        return os << msg;
    }

    int n_rows() const {
        return n_rows_;
    }

    int n_columns() const {
        return n_columns_;
    }

private:
    // Number of rows in the matrix.
    int n_rows_ = 0;

    // Number of columns in the matrix.
    int n_columns_ = 0;
};

using FMatrix = Matrix<float>;
using RMatrix = Matrix<double>;

} // namespace cl

#endif // CODELIBRARY_MATH_MATRIX_MATRIX_H_
