//
// Copyright 2019-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_LEGEND_H_
#define CODELIBRARY_PLOT_LEGEND_H_

#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/plot/object.h"
#include "codelibrary/plot/pen.h"
#include "codelibrary/plot/terminal.h"

namespace cl {
namespace plot {

/**
 * Legend for plot.
 */ 
class Legend {
public:
    // Where to draw the legend in the plot.
    enum Position {
        RIGHT_TOP, LEFT_TOP, LEFT_BOTTOM, RIGHT_BOTTOM
    };

    // Item of Plot.
    struct Item {
        Object::Type type; // Type of this item.
        std::string name;  // Item name.
        Pen pen;           // Pen to draw this item.

        Item() {}

        Item(Object::Type t, const std::string& n, const Pen& p)
            : type(t), name(n), pen(p) {}

        /**
         * Draw this item in the given box.
         */
        void Draw(const RBox2D& box, Terminal* terminal) const {
            switch (type) {
            case Object::LINES:
                terminal->set_pen(pen);
                terminal->DrawLine(box.x_min(), (box.y_min() +
                                                 box.y_max()) * 0.5,
                                   box.x_max(), (box.y_min() +
                                                 box.y_max()) * 0.5);
                break;

            case Object::POINTS:
            {
                terminal->set_pen(pen);
                double radius = 0.2 * box.y_length();

                RPoint2D c1(box.x_min() + radius * 2.0,
                            box.y_min() + radius * 3.0);
                RPoint2D c2((box.x_min() + box.x_max()) * 0.5,
                            box.y_max() - radius);
                RPoint2D c3(box.x_max() - radius * 2.0,
                            box.y_min() + radius * 2.5);

                switch (pen.point_style) {
                case 'o':
                    terminal->DrawCircle(c1.x, c1.y, radius);
                    terminal->DrawCircle(c2.x, c2.y, radius);
                    terminal->DrawCircle(c3.x, c3.y, radius);
                    break;
                case 'r':
                    terminal->DrawRectangle(c1.x - radius, c1.y + radius,
                                            2.0 * radius, 2.0 * radius);
                    terminal->DrawRectangle(c2.x - radius, c2.y + radius,
                                            2.0 * radius, 2.0 * radius);
                    terminal->DrawRectangle(c3.x - radius, c3.y + radius,
                                            2.0 * radius, 2.0 * radius);
                    break;
                default:
                    break;
                }
                break;
            }

            case Object::POLYGON:
            {
                double w = box.x_length(), h = box.y_length();
                terminal->set_pen(pen);
                terminal->DrawRectangle(box.x_min() + 0.1 * w,
                                        box.y_max() - 0.1 * h,
                                        0.8 * w,
                                        0.8 * h);
                break;
            }

            default:
                break;
            }
        }
    };

    /**
     * Add an item into the legend.
     */
    void InsertItem(const Object& o) {
        items_.emplace_back(o.type_, o.name_, o.pen_);
    }

    const Array<Item>& items() const {
        return items_;
    }

    void clear() {
        items_.clear();
    }

    void set_position(const Position& position) {
        position_ = position;
    }

    const Position& position() const {
        return position_;
    }

private:
    Position position_ = RIGHT_TOP;
    Array<Item> items_;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_LEGEND_H_
