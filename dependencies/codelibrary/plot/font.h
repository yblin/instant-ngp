//
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_FONT_H_
#define CODELIBRARY_PLOT_FONT_H_

#include <string>

#include "codelibrary/base/log.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace plot {

// Font for plot library.
struct Font {
    /**
     * The Alignment property is used to align (start-, middle- or
     * end-alignment) a string of text relative to a given point.
     */
    enum Alignment {
        START  = 0,
        MIDDLE = 1,
        END    = 2
    };

    /**
     * Create an empty font, the default font size is 13.
     */
    explicit Font(double s = 13.0) {
        CHECK(s > 0.0);

        size = s;
    }

    Font(const std::string& n, double s = 13.0) {
        CHECK(s > 0.0);

        name = n;
        size = s;
    }

    // Name of font.
    std::string name;

    // Font size. It is the height of font in pixels.
    double size = 0.0;

    // Set font color.
    RGB32Color color = RGB32Color::Black();

    // Alignment for text.
    Alignment alignment = START;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_FONT_H_
