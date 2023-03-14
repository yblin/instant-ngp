//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_VECTOR_3D_H_
#define CODELIBRARY_GEOMETRY_VECTOR_3D_H_

#include <cmath>
#include <type_traits>

#include "codelibrary/base/equal.h"
#include "codelibrary/base/message.h"

namespace cl {

/**
 * 3D vector.
 */
template<typename T>
class Vector3D {
public:
    using value_type = T;

    Vector3D() = default;

    Vector3D(const T& x1, const T& y1, const T& z1)
        : x(x1), y(y1), z(z1) {}

    bool operator ==(const Vector3D& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator !=(const Vector3D& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Vector3D& rhs) const {
        return x == rhs.x ? (y == rhs.y ? z < rhs.z : y < rhs.y) : x < rhs.x;
    }

    bool operator <=(const Vector3D& rhs) const {
        return !(rhs < *this);
    }

    bool operator > (const Vector3D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Vector3D& rhs) const {
        return !(*this < rhs);
    }

    const Vector3D& operator +=(const Vector3D& rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    const Vector3D& operator -=(const Vector3D& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    const Vector3D& operator *=(const T& rhs) {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        return *this;
    }

    const Vector3D& operator *=(const Vector3D& rhs) {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }

    /**
     * Return the squared euclidean norm of the vector.
     */
    T squared_norm() const {
        static_assert(std::is_floating_point<T>::value, "");

        return x * x + y * y + z * z;
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
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    /**
     * Return the reference value of the i-th component of vector.
     */
    T& operator[] (int i) {
        return (i == 0) ? x : (i == 1 ? y : z);
    }

    int size() const { return 3; }
    
    /**
     * Normalize the vector.
     * The length of the normalized vector is always one.
     */
    Vector3D& Normalize() {
        static_assert(std::is_floating_point<T>::value, "");

        T len = this->norm();
        if (Equal(len, T(0))) {
            x = 0;
            y = 0;
            z = 1;
        } else {
            T t = T(1) / len;
            x *= t;
            y *= t;
            z *= t;
        }

        return *this;
    }

    friend Vector3D operator +(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
    }

    friend Vector3D operator -(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
    }

    friend Vector3D operator -(const Vector3D& rhs) {
        return Vector3D(-rhs.x, -rhs.y, -rhs.z);
    }

    friend Vector3D operator *(const T& lhs, const Vector3D& rhs) {
        return Vector3D(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
    }

    friend Vector3D operator *(const Vector3D& lhs, const T& rhs) {
        return Vector3D(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
    }

    friend Vector3D operator *(const Vector3D& lhs, const Vector3D& rhs) {
        return Vector3D(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z);
    }

    friend std::ostream& operator <<(std::ostream& os, const Vector3D& v) {
        Message msg(v.x, v.y, v.z);
        return os << msg;
    }

    T x = 0; // X component.
    T y = 0; // Y component.
    T z = 0; // Z component.
};

using IVector3D = Vector3D<int>;
using FVector3D = Vector3D<float>;
using RVector3D = Vector3D<double>;

/**
 * Return the dot product of two vectors.
 */
template <typename T>
T DotProduct(const Vector3D<T>& v1, const Vector3D<T>& v2) {
    static_assert(std::is_floating_point<T>::value, "");

    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * Return the cross product of two vectors.
 */
template <typename T>
Vector3D<T> CrossProduct(const Vector3D<T>& v1, const Vector3D<T>& v2) {
    static_assert(std::is_floating_point<T>::value, "");

    return Vector3D<T>(v1.y * v2.z - v1.z * v2.y,
                       v1.z * v2.x - v1.x * v2.z,
                       v1.x * v2.y - v1.y * v2.x);
}

/**
 * Return the normalized vector.
 */
template <typename T>
Vector3D<T> Normalize(const Vector3D<T>& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return Vector3D<T>(v).Normalize();
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_VECTOR_3D_H_
