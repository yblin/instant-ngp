//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_VECTOR_4D_H_
#define CODELIBRARY_GEOMETRY_VECTOR_4D_H_

#include <cmath>
#include <type_traits>

#include "codelibrary/base/equal.h"
#include "codelibrary/base/log.h"

namespace cl {

/**
 * 4D vector.
 */
template <typename T>
class Vector4D {
public:
    using value_type = T;

    Vector4D() = default;

    Vector4D(const T& x1, const T& y1, const T& z1, const T& w1)
        : x(x1), y(y1), z(z1), w(w1) {}

    bool operator ==(const Vector4D& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
    }

    bool operator !=(const Vector4D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Vector4D& rhs) const {
        if (x == rhs.x) {
            if (y == rhs.y) {
                return (z == rhs.z) ? w < rhs.w : z < rhs.z;
            }
            return y < rhs.y;
        }
        return x < rhs.x;
    }

    bool operator <=(const Vector4D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Vector4D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Vector4D& rhs) const {
        return !(*this < rhs);
    }

    const Vector4D& operator +=(const Vector4D& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w += rhs.w;
        return *this;
    }

    const Vector4D& operator -=(const Vector4D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w -= rhs.w;
        return *this;
    }

    const Vector4D& operator *=(const T& rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        w *= rhs;
        return *this;
    }

    const Vector4D& operator *=(const Vector4D& rhs) {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        w *= rhs.w;
        return *this;
    }

    /**
     * Return the squared euclidean norm of the vector.
     */
    T squared_norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return x * x + y * y + z * z + w * w;
    }

    /**
     * Return the euclidean norm of the vector.
     */
    T norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return std::sqrt(squared_norm());
    }

    /**
     * Return the i-th component value of vector.
     */
    const T& operator[] (int i) const {
        switch (i) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return w;
        default:
            CHECK(false);
            return w;
        }
    }

    /**
     * Return the reference value of the i-th component of vector.
     */
    T& operator[] (int i) {
        switch (i) {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return w;
        default:
            CHECK(false);
            return w;
        }
    }

    int size() const { return 4; }

    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector4D& Normalize() {
        static_assert(std::is_floating_point<T>::value, "");

        T len = this->norm();
        if (Equal(len, T(0))) {
            x = 0;
            y = 0;
            z = 0;
            w = 1;
        } else {
            T t = T(1) / len;
            x *= t;
            y *= t;
            z *= t;
            w *= t;
        }

        return *this;
    }

    friend Vector4D operator +(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z,
                        lhs.w + rhs.w);
    }

    friend Vector4D operator -(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z,
                        lhs.w - rhs.w);
    }

    friend Vector4D operator -(const Vector4D& rhs) {
        return Vector4D(-rhs.x, -rhs.y, -rhs.z, -rhs.w);
    }

    friend Vector4D operator *(const T& lhs, const Vector4D& rhs) {
        return Vector4D(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
    }

    friend Vector4D operator *(const Vector4D& lhs, const T& rhs) {
        return Vector4D(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
    }

    friend Vector4D operator *(const Vector4D& lhs, const Vector4D& rhs) {
        return Vector4D(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z,
                        lhs.w * rhs.w);
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector4D& v) {
        Message msg(v.x, v.y, v.z);
        return os << msg;
    }

    T x = 0; // X component.
    T y = 0; // Y component.
    T z = 0; // Z component.
    T w = 0; // W component.
};

using IVector4D = Vector4D<int>;
using FVector4D = Vector4D<float>;
using RVector4D = Vector4D<double>;

/**
 * Return the dot product of two vectors.
 */
template <typename T>
T DotProduct(const Vector4D<T>& v1, const Vector4D<T>& v2) {
    static_assert(std::is_floating_point<T>::value, "");

    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
}

/**
 * Return the normalized vector.
 */
template <typename T>
Vector4D<T> Normalize(const Vector4D<T>& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return Vector4D<T>(v).Normalize();
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_VECTOR_4D_H_
