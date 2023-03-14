//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_COLOR_HSI_COLOR_TEST_H_
#define CODELIBRARY_TEST_UTIL_COLOR_HSI_COLOR_TEST_H_

#include <cmath>

#include "codelibrary/base/testing.h"
#include "codelibrary/util/color/hsi_color.h"

namespace cl {
namespace test {

TEST(HSIColorTest, ConvertRGBColorTest) {
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
        0.0,   0.0,   0.0,  0.0,   60.0, 120.0, 180.0, 240.0, 300.0, 61.5,
        250.0, 133.8, 50.5, 284.8, 13.2, 57.4,  163.4, 247.3, 240.4
    };
    Array<double> saturation = {
        0.0,   0.0,   0.0,   1.0,   1.0,   1.0,   0.4, 0.25,  0.571, 0.699,
        0.756, 0.667, 0.911, 0.686, 0.446, 0.363, 0.8, 0.533, 0.135
    };
    Array<double> intensity = {
        1.0,   0.5,   0.0,   0.333, 0.5,  0.167, 0.833, 0.667, 0.583, 0.471,
        0.426, 0.349, 0.593, 0.596, 0.57, 0.835, 0.495, 0.319, 0.57
    };

    for (int i = 0; i < rgb_colors.size(); ++i) {
        HSIColor c(rgb_colors[i]);

        ASSERT_EQ_NEAR(c.hue() * 360.0, hue[i], 1e-1)
                << i << " " << c.hue() * 360.0 << " " << hue[i];

        ASSERT_EQ_NEAR(c.saturation(), saturation[i], 1e-3)
                << i << " " << c.saturation() << " " << saturation[i];

        ASSERT_EQ_NEAR(c.intensity(), intensity[i], 1e-3)
                << i << " " << c.intensity() << " " << intensity[i];

        RGBColor c1 = c.ToRGBColor();
        ASSERT_EQ_NEAR(c1.red(), rgb_colors[i].red(), 1e-3)
                << i << " " << c1.red() << " " << rgb_colors[i].red();

        ASSERT_EQ_NEAR(c1.green(), rgb_colors[i].green(), 1e-3)
                << i << " " << c1.green() << " " << rgb_colors[i].green();

        ASSERT_EQ_NEAR(c1.blue(), rgb_colors[i].blue(), 1e-3)
                << i << " " << c1.blue() << " " << rgb_colors[i].blue();
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_COLOR_HSI_COLOR_TEST_H_
