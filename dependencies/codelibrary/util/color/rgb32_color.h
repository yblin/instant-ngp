//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_COLOR_RGB32_COLOR_H_
#define CODELIBRARY_UTIL_COLOR_RGB32_COLOR_H_

#include <cstdint>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/message.h"

namespace cl {

/**
 * 32-bits RGB Color.
 */
class RGB32Color {
public:
    static const RGB32Color Red()    { return {255,   0,   0}; }
    static const RGB32Color Green()  { return {  0, 255,   0}; }
    static const RGB32Color Blue()   { return {  0,   0, 255}; }
    static const RGB32Color White()  { return {255, 255, 255}; }
    static const RGB32Color Black()  { return {  0,   0,   0}; }
    static const RGB32Color Gray()   { return {128, 128, 128}; }
    static const RGB32Color Yellow() { return {255, 255, 0};   }

    RGB32Color() = default;

    RGB32Color(int r, int g, int b, int a = 255) {
        red_   = static_cast<uint8_t>(Clamp(r, 0, 255));
        green_ = static_cast<uint8_t>(Clamp(g, 0, 255));
        blue_  = static_cast<uint8_t>(Clamp(b, 0, 255));
        alpha_ = static_cast<uint8_t>(Clamp(a, 0, 255));
    }

    RGB32Color(double r, double g, double b, double a = 1.0) {
        red_   = static_cast<uint8_t>(Clamp(r * 255.0, 0.0, 255.0));
        green_ = static_cast<uint8_t>(Clamp(g * 255.0, 0.0, 255.0));
        blue_  = static_cast<uint8_t>(Clamp(b * 255.0, 0.0, 255.0));
        alpha_ = static_cast<uint8_t>(Clamp(a * 255.0, 0.0, 255.0));
    }

    /**
     * Construct the RGB32Color form a single int.
     */
    explicit RGB32Color(int rgb) {
        alpha_ = static_cast<uint8_t>(rgb >> 24);
        blue_  = (rgb >> 16) & 0xff;
        green_ = (rgb >> 8)  & 0xff;
        red_   =  rgb        & 0xff;
    }

    void set_red(int r)   { red_   = Clamp(r, 0, 255); }
    void set_green(int g) { green_ = Clamp(g, 0, 255); }
    void set_blue(int b)  { blue_  = Clamp(b, 0, 255); }
    void set_alpha(int a) { alpha_ = Clamp(a, 0, 255); }

    uint8_t red()   const { return red_;   }
    uint8_t green() const { return green_; }
    uint8_t blue()  const { return blue_;  }
    uint8_t alpha() const { return alpha_; }

    /**
     * Convert the RGB color to the gray scale.
     * Return a value from 0 to 255.
     */
    int ToGrayScale() const {
        return (static_cast<int>(red_) * 11 + static_cast<int>(green_) * 16 +
                static_cast<int>(blue_) * 5) / 32;
    }

    /**
     * Convert the RGB32Color to a single 32-bit int.
     */
    uint32_t ToUInt() const {
        return (static_cast<uint32_t>(alpha_) << 24) |
               (static_cast<uint32_t>(blue_)  << 16) |
               (static_cast<uint32_t>(green_) << 8) |
               (static_cast<uint32_t>(red_));
    }

    bool operator == (const RGB32Color& color) const {
        return red_ == color.red_   && green_ == color.green_ &&
               blue_ == color.blue_ && alpha_ == color.alpha_;
    }

    bool operator != (const RGB32Color& color) const {
        return !(*this == color);
    }

    friend std::ostream& operator <<(std::ostream& os, const RGB32Color& c) {
        Message msg(c.red_, c.green_, c.blue_, c.alpha_);
        os << msg;
        return os;
    }

private:
    uint8_t red_   = 0; // From 0 to 255.
    uint8_t green_ = 0; // From 0 to 255.
    uint8_t blue_  = 0; // From 0 to 255.
    uint8_t alpha_ = 0; // From 0 to 255.
};

} // namespace cl

#endif // CODELIBRARY_UTIL_COLOR_RGB32_COLOR_H_
