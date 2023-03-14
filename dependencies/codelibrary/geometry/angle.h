//
// Copyright 2012-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_ANGLE_H_
#define CODELIBRARY_GEOMETRY_ANGLE_H_

// The following code can be replaced by #include <numbers> in C++20
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#else
#include <cmath>
#endif // _USE_MATH_DEFINES

#include <type_traits>

#include "codelibrary/base/clamp.h"
#include "codelibrary/geometry/vector_2d.h"
#include "codelibrary/geometry/vector_3d.h"
#include "codelibrary/geometry/vector_4d.h"
#include "codelibrary/math/number/exact_float.h"

namespace cl {

/**
 * Convert degree to radian.
 */
template <typename T>
inline T DegreeToRadian(const T& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return v * T(M_PI) / 180;
}

/**
 * Convert radian to degree.
 */
template <typename T>
inline T RadianToDegree(const T& v) {
    static_assert(std::is_floating_point<T>::value, "");

    return v * 180 / T(M_PI);
}

/**
 * Get the angle of 2D vector in radians.
 *
 * The return value is in the range: [0, 2 * PI).
 */
template <typename T>
T Radian(const Vector2D<T>& v) {
    static_assert(std::is_floating_point<T>::value, "");

    T radian = std::atan2(v.y, v.x);
    return radian < 0 ? radian + M_PI + M_PI : radian;
}

/**
 * Robust compute the radian angle of 2D vector (x2 - x1, y2 - y1).
 *
 * The return value is in the range: [0, 2 * PI).
 */
template <typename T>
T Radian(T x1, T y1, T x2, T y2) {
    static_assert(std::is_floating_point<T>::value, "");

    ExactFloat a(x1), b(y1), c(x2), d(y2);
    c -= a;
    d -= b;

    if (c.sign() == 0) {
        if (d.sign() == 0) return std::numeric_limits<T>::quiet_NaN();
        return d.sign() > 0 ? M_PI : 1.5 * M_PI;
    }

    int sign = c.sign() * d.sign();
    if (sign == 0) return 0;

    double v = static_cast<double>(d.significand().ToUInt64()) /
               static_cast<double>(c.significand().ToUInt64()) *
               std::pow(2.0, d.exponent() - c.exponent());
    double angle = std::atan(v);
    return sign > 0 ? angle : M_PI + M_PI - angle;
}

/**
 * Get the degree angle of 2D vector.
 *
 * The return value is in the range: [0, 360).
 */
template <typename T>
T Degree(const Vector2D<T>& v) {
    return RadianToDegree(Radian(v));
}

/**
 * Get the angle between two vectors in degrees.
 */
template <typename Vector>
auto Degree(const Vector& v1, const Vector& v2) {
    return RadianToDegree(Radian(v1, v2));
}

/**
 * Numerically stable calculation of the angle between two vectors in radians.
 *
 * Also it is faster than cosine law version.
 *
 * The return value is in the range: [0, PI].
 */
template <typename Vector>
auto Radian(const Vector& v1, const Vector& v2) {
    using T = typename Vector::value_type;
    static_assert(std::is_floating_point<T>::value, "");

    T a = v1.norm();
    T b = v2.norm();
    T c = (v2 - v1).norm();

    T u = b >= c ? c - (a - b) : b - (a - c);
    T t1 = ((a - b) + c) * u;
    T t2 = ((b + c) + a) * ((a - c) + b);
    return T(2) * std::atan(std::sqrt(t1 / t2));
}

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_ANGLE_H_
