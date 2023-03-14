//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_COLOR_HSL_COLOR_TEST_H_
#define CODELIBRARY_TEST_UTIL_COLOR_HSL_COLOR_TEST_H_

#include <cmath>

#include "codelibrary/base/testing.h"
#include "codelibrary/util/color/hsl_color.h"

namespace cl {
namespace test {

TEST(HSLColorTest, ConvertRGBColorTest) {
    Array<RGBColor> rgb_colors = {
        {1.000, 1.000, 1.000},
        {0.500, 0.500, 0.500},
        {0.000, 0.000, 0.000},
        {1.000, 0.000, 0.000},
        {0.750, 0.750, 0.000},
        {0.000, 0.500, 0.000},
        {0.500, 1.000, 1.000},
        {0.500, 0.500, 1.000},
        {0.750, 0.250, 0.750},
        {0.628, 0.643, 0.142},
        {0.255, 0.104, 0.918},
        {0.116, 0.675, 0.255},
        {0.941, 0.785, 0.053},
        {0.704, 0.187, 0.897},
        {0.931, 0.463, 0.316},
        {0.998, 0.974, 0.532},
        {0.099, 0.795, 0.591},
        {0.211, 0.149, 0.597},
        {0.495, 0.493, 0.721}
    };

    Array<double> hue = {
        0.0,   0.0,   0.0,  0.0,   60.0, 120.0, 180.0, 240.0, 300.0, 61.8,
        251.1, 134.9, 49.5, 283.7, 14.3, 56.9,  162.4, 248.3, 240.5
    };
    Array<double> saturation = {
        0.0,   0.0,   0.0,   1.0,   1.0,   1.0,   1.0,   1.0,   0.5, 0.638,
        0.832, 0.707, 0.893, 0.775, 0.817, 0.991, 0.779, 0.601, 0.29
    };
    Array<double> lightness = {
        1.0,   0.5,   0.0,   0.5,   0.375, 0.25,  0.75,  0.75,  0.5, 0.393,
        0.511, 0.396, 0.497, 0.542, 0.624, 0.765, 0.447, 0.373, 0.607
    };

    for (int i = 0; i < rgb_colors.size(); ++i) {
        HSLColor c(rgb_colors[i]);

        ASSERT_EQ_NEAR(c.hue() * 360.0, hue[i], 1e-1)
                << i << " " << c.hue() * 360.0 << " " << hue[i];

        ASSERT_EQ_NEAR(c.saturation(), saturation[i], 1e-3)
                << i << " " << c.saturation() << " " << saturation[i];

        ASSERT_EQ_NEAR(c.lightness(), lightness[i], 1e-3)
                << i << " " << c.lightness() << " " << lightness[i];

        RGBColor c1 = c.ToRGBColor();
        ASSERT_EQ_NEAR(c1.red(),   rgb_colors[i].red(),   1e-3);
        ASSERT_EQ_NEAR(c1.green(), rgb_colors[i].green(), 1e-3);
        ASSERT_EQ_NEAR(c1.blue(),  rgb_colors[i].blue(),  1e-3);
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_COLOR_HSL_COLOR_TEST_H_
