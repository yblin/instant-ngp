//
// Copyright 2015 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_FLOAT_H_
#define CODELIBRARY_BASE_FLOAT_H_

#include <cstdint>
#include <numeric>

namespace cl {

template <typename T>
class Float {
    static_assert(std::is_floating_point<T>::value, "");

    Float() = delete;
};

template <>
class Float<float> {
    typedef union { float f; int i; } Float32;

public:
    static constexpr int N_MANTISSA_BITS = 23;

    explicit Float(float x) {
        u_.f = x;
    }

    float value() const {
        return u_.f;
    }

    int exponent() const {
        return (u_.i & 0x7F800000) >> 23;
    }

    int mantissa() const {
        return (u_.i & 0x007FFFFF);
    }

    bool is_inf() const {
        return exponent() == 0xFF && mantissa() == 0;
    }

private:
    Float32 u_;
};

template <>
class Float<double> {
    typedef union { double f; int64_t i; } Float64;

public:
    static constexpr int N_MANTISSA_BITS = 52;

    explicit Float(double x) {
        u_.f = x;
    }

    double value() const {
        return u_.f;
    }

    int exponent() const {
        return (u_.i & 0x7FF0000000000000LL) >> 52;
    }

    int64_t mantissa() const {
        return (u_.i & 0x000FFFFFFFFFFFFFLL);
    }

    bool is_inf() const {
        return exponent() == 0x7FF && mantissa() == 0LL;
    }

private:
    Float64 u_;
};

} // namespace cl

#endif // CODELIBRARY_BASE_FLOAT_H_
