//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef MATH_NUMBER_DUAL_NUMBER_H_
#define MATH_NUMBER_DUAL_NUMBER_H_

#include <cmath>

namespace cl {

/**
 * Dual number for automatically computing exact derivatives of functions.
 */
template <typename T>
struct DualNumber {
    DualNumber() = default;

    /**
     * Constructor from scalar: a + 0.
     */
    DualNumber(T a)
        : value(a) {}

    DualNumber(T a, T b)
        : value(a), dual(b) {}

    DualNumber& operator+=(const DualNumber& y) {
        dual += y.dual;
        value += y.value;
        return *this;
    }

    DualNumber& operator+=(T s) {
        value += s;
        return *this;
    }

    DualNumber& operator-=(const DualNumber& y) {
        dual -= y.dual;
        value -= y.value;
        return *this;
    }

    DualNumber& operator-=(T s) {
        value -= s;
        return *this;
    }

    DualNumber& operator*=(const DualNumber& y) {
        dual = y.value * dual + value * y.dual;
        value *= y.value;
        return *this;
    }

    DualNumber& operator*=(T s) {
        dual *= s;
        value *= s;
        return *this;
    }

    DualNumber& operator/=(const DualNumber& y) {
        *this = *this / y;
        return *this;
    }

    DualNumber& operator/=(T s) {
        *this *= T(1) / s;
        return *this;
    }

    bool operator ==(const DualNumber& y) {
        return value == y.value;
    }

    bool operator !=(const DualNumber& y) {
        return value != y.value;
    }

    bool operator <(const DualNumber& y) {
        return value < y.value;
    }

    bool operator <=(const DualNumber& y) {
        return value <= y.value;
    }

    bool operator >(const DualNumber& y) {
        return value > y.value;
    }

    bool operator >=(const DualNumber& y) {
        return value >= y.value;
    }

    friend const DualNumber& operator+(const DualNumber& y) {
        return y;
    }

    friend DualNumber operator-(const DualNumber& y) {
        return DualNumber(-y.value, -y.dual);
    }

    friend DualNumber operator+(const DualNumber& x, const DualNumber& y) {
        return DualNumber(x.value + y.value, x.dual + y.dual);
    }

    friend DualNumber operator+(const DualNumber& f, T s) {
        return DualNumber(f.value + s, f.dual);
    }

    friend DualNumber operator+(T s, const DualNumber& f) {
        return DualNumber(f.value + s, f.dual);
    }

    friend DualNumber operator-(const DualNumber& x, const DualNumber& y) {
        return DualNumber(x.value - y.value, x.dual - y.dual);
    }

    friend DualNumber operator-(const DualNumber& f, T s) {
        return DualNumber(f.value - s, f.dual);
    }

    friend DualNumber operator-(T s, const DualNumber& f) {
        return DualNumber(s - f.value, -f.dual);
    }

    /**
     * (x + e_x)(y + e_y) = xy + ye_x + xe_y
     */
    friend DualNumber operator*(const DualNumber x, const DualNumber& y) {
        return DualNumber(x.value * y.value,
                          y.value * x.dual + x.value * y.dual);
    }

    friend DualNumber operator*(const DualNumber& f, T s) {
        return DualNumber(f.value * s, f.dual * s);
    }

    friend DualNumber operator*(T s, const DualNumber& f) {
        return DualNumber(f.value * s, f.dual * s);
    }

    /**
     * This uses:
     *
     *   a + u   (a + u)(b - v)   (a + u)(b - v)   a   bu - av
     *   ----- = -------------- = -------------- = - + --------
     *   b + v   (b + v)(b - v)        b^2         b     b^2
     *
     * which holds because v*v = 0.
     */
    friend DualNumber operator/(const DualNumber& x, const DualNumber& y) {
        const T b_inverse = T(1) / y.value;
        const T a_b = x.value * b_inverse;
        return DualNumber(a_b, (x.dual - a_b * y.dual) * b_inverse);
    }

    friend DualNumber operator/(const DualNumber& f, T s) {
        const T s_inverse = T(1) / s;
        return DualNumber(f.value * s_inverse, f.dual * s_inverse);
    }

    /**
     *   a        a(b - v)       ab - av     a     a
     * ----- = -------------- = ---------- = - - ---- v
     * b + v   (b + v)(b - v)      b^2       b    b^2
     */
    friend DualNumber operator/(T s, const DualNumber& f) {
        const T a_b2 = -s / (f.value * f.value);
        return DualNumber(s / f.value, a_b2 * f.dual);
    }

    // The scalar part.
    T value = T();

    // The infinitesimal part.
    T dual = T();
};

// Pull some functions from namespace std.
//
// This is necessary because we want to use the same name (e.g. 'sqrt') for
// double-valued and dual number functions, but we are not allowed to put
// dual number functions inside namespace std.
using std::abs;
using std::log;
using std::sqrt;
using std::sin;
using std::cos;
using std::asin;
using std::acos;
using std::tan;
using std::atan;

/**
 * abs(x + h) ~= abs(x) + sgn(x)h
 */
template <typename T>
DualNumber<T> abs(const DualNumber<T>& f) {
    return DualNumber<T>(abs(f.value), std::copysign(T(1), f.value) * f.dual);
}

/**
 * log(a + h) ~= log(a) + h / a
 */
template <typename T>
DualNumber<T> log(const DualNumber<T>& f) {
    const T a_inverse = T(1) / f.a;
    return DualNumber<T>(log(f.value), f.dual * a_inverse);
}

/**
 * exp(a + h) ~= exp(a) + exp(a) h
 */
template <typename T>
DualNumber<T> exp(const DualNumber<T>& f) {
    const T tmp = exp(f.value);
    return DualNumber<T>(tmp, tmp * f.dual);
}

/**
 * sqrt(a + h) ~= sqrt(a) + h / (2 sqrt(a))
 */
template <typename T>
DualNumber<T> sqrt(const DualNumber<T>& f) {
    const T tmp = sqrt(f.value);
    const T two_a_inverse = T(1.0) / (T(2.0) * tmp);
    return DualNumber<T>(tmp, f.dual * two_a_inverse);
}

/**
 * cos(a + h) ~= cos(a) - sin(a) h
 */
template <typename T>
inline DualNumber<T> cos(const DualNumber<T>& f) {
    return DualNumber<T>(cos(f.value), -sin(f.value) * f.dual);
}

/**
 * acos(a + h) ~= acos(a) - 1 / sqrt(1 - a^2) h
 */
template <typename T>
DualNumber<T> acos(const DualNumber<T>& f) {
    const T tmp = -T(1.0) / sqrt(T(1.0) - f.value * f.value);
    return DualNumber<T>(acos(f.value), tmp * f.dual);
}

/**
 * sin(a + h) ~= sin(a) + cos(a) h
 */
template <typename T>
DualNumber<T> sin(const DualNumber<T>& f) {
    return DualNumber<T>(sin(f.value), cos(f.value) * f.dual);
}

/**
 * asin(a + h) ~= asin(a) + 1 / sqrt(1 - a^2) h
 */
template <typename T>
DualNumber<T> asin(const DualNumber<T>& f) {
    const T tmp = T(1.0) / sqrt(T(1.0) - f.value * f.value);
    return DualNumber<T>(asin(f.value), tmp * f.dual);
}

/**
 * tan(a + h) ~= tan(a) + (1 + tan(a)^2) h
 */
template <typename T>
DualNumber<T> tan(const DualNumber<T>& f) {
    const T tan_a = tan(f.value);
    const T tmp = T(1.0) + tan_a * tan_a;
    return DualNumber<T>(tan_a, tmp * f.dual);
}

/**
 * atan(a + h) ~= atan(a) + 1 / (1 + a^2) h
 */
template <typename T>
DualNumber<T> atan(const DualNumber<T>& f) {
    const T tmp = T(1.0) / (T(1.0) + f.value * f.value);
    return DualNumber<T>(atan(f.value), tmp * f.dual);
}

} // namespace cl

#endif // MATH_NUMBER_DUAL_NUMBER_H_
