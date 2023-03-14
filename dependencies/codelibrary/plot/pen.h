//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_PEN_H_
#define CODELIBRARY_PLOT_PEN_H_

#include "codelibrary/base/array.h"
#include "codelibrary/util/color/rgb32_color.h"

namespace cl {
namespace plot {

/**
 * The Pen class defines how a Painter should draw lines and outlines of shapes.
 */
struct Pen {
    // Width for drawing lines.
    double line_width = 1.0;

    // Line color.
    RGB32Color line_color = RGB32Color(0, 0, 0, 255);

    // Radius for drawing points.
    double point_radius = 3.0;

    // Is fill the close shape or not.
    bool is_fill = true;

    // Fill color.
    RGB32Color fill_color = RGB32Color(0, 0, 0, 255);

    // Point style.
    char point_style = 'o';

    // Same definition as stroke-dasharray in SVG.
    Array<int> stroke_dash;

    explicit Pen(const RGB32Color& c = RGB32Color(0, 0, 0, 255))
        : line_color(c), fill_color(c) {}

    /**
     * Set line style.
     * The paramter 'style' is a string consist of '-', '.', ' ', and ':'.
     *
     * Example:
     *   "-"    # Solid line:       -----------
     *   "."    # Dot line:         ...........
     *   "--"   # Dash line:        -- -- -- --
     *   "- "   # Dash line2:       --  --  --
     *   ":"    # Short dash line:  - - - - - -
     *   ": "   # Short dash line2: -  -  -  -
     *   "-."   # Dot dash line:    --.--.--.--
     *   "-. "  # Dot dash line2:   --.  --.  --.
     */
    void SetLineStyle(const std::string& style) {
        stroke_dash.clear();
        if (style == "-") return;

        for (size_t i = 0; i < style.length(); ++i) {
            char c = style[i];
            if (c == ' ') {
                if (stroke_dash.size() % 2 == 0) {
                    stroke_dash.push_back(0);
                }
                stroke_dash.push_back(10);
            } else if (c == '.' || c == ':' || c == '-') {
                if (stroke_dash.size() % 2 == 1) {
                    stroke_dash.push_back(2);
                }
                if (c == '.')
                    stroke_dash.push_back(2);
                else if (c == ':')
                    stroke_dash.push_back(5);
                else
                    stroke_dash.push_back(10);
            } else {
                CHECK(false) << "Unknow character in line style: '" << c
                             << "'.";
            }
        }
        if (stroke_dash.size() % 2 == 1) stroke_dash.push_back(2);
    }
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_PEN_H_
