//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_COLOR_XYZ_COLOR_H_
#define CODELIBRARY_UTIL_COLOR_XYZ_COLOR_H_

#include <cmath>

#include "codelibrary/util/color/rgb_color.h"

namespace cl {

/**
 * XYZ color.
 *
 * XYZColor is a color space defined in terms of standard responses to a power
 * spectrum. It was the first standard quantitative color space and allows for
 * objective comparison of colors.
 *
 * The parameters have the following interpretation:
 *   x: color, combination of green and red;
 *   y: approximate luminance;
 *   z: color, approximately blue.
 *
 * XYZColor allows any non-negative real number for x, y and z, negative values
 * will be clamped.
 */
struct XYZColor {
public:
    XYZColor() = default;

    /**
     * Color and opacity levels outside the range 0 to 1 will be clamped.
     */
    XYZColor(double x, double y, double z, double a = 1.0) {
        x_     = x < 0.0 ? 0.0 : x;
        y_     = y < 0.0 ? 0.0 : y;
        z_     = z < 0.0 ? 0.0 : z;
        alpha_ = Clamp(a, 0.0, 1.0);
    }

    /**
     * Construct XYZColor from RGBColor.
     * Uses the standard D65 white point.
     */
    explicit XYZColor(const RGBColor& rgb) {
        double r = Gamma(rgb.red());
        double g = Gamma(rgb.green());
        double b = Gamma(rgb.blue());

        x_ = 0.412453 * r + 0.357580 * g + 0.180423 * b;
        y_ = 0.212671 * r + 0.715160 * g + 0.072169 * b;
        z_ = 0.019334 * r + 0.119193 * g + 0.950227 * b;
        alpha_ = rgb.alpha();
    }

    /**
     * Convert XYZColor to RGBColor.
     */
    RGBColor ToRGBColor() const {
        double r =  3.240479 * x_ - 1.537150 * y_ - 0.498535 * z_;
        double g = -0.969256 * x_ + 1.875992 * y_ + 0.041556 * z_;
        double b =  0.055648 * x_ - 0.204043 * y_ + 1.057311 * z_;

        r = GammaExpand(r);
        g = GammaExpand(g);
        b = GammaExpand(b);

        return {r, g, b, alpha_};
    }

    void set_x(double x)     { x_     = x < 0.0 ? 0.0 : x;       }
    void set_y(double y)     { y_     = y < 0.0 ? 0.0 : y;       }
    void set_z(double z)     { z_     = z < 0.0 ? 0.0 : z;       }
    void set_alpha(double a) { alpha_ = Clamp(a, 0.0, 1.0); }

    double x()     const { return x_;     }
    double y()     const { return y_;     }
    double z()     const { return z_;     }
    double alpha() const { return alpha_; }

private:
    static double GammaExpand(double t) {
        return (t <= 0.0031308) ? t * 12.92
                                : 1.055 * std::pow(t, 1.0 / 2.4) - 0.055;
    }
    static double Gamma(double t) {
        if (t > 0.04045) {
            t = std::pow((t + 0.055) / 1.055, 2.4);
        } else {
            t /= 12.92;
        }
        return t;
    }

    double x_     = 0.0;
    double y_     = 0.0;
    double z_     = 0.0;
    double alpha_ = 0.0;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_COLOR_XYZ_COLOR_H_
