//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_OBJECT_H_
#define CODELIBRARY_PLOT_OBJECT_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/plot/pen.h"

namespace cl {
namespace plot {

/**
 * 2D plotable object.
 */
struct Object {
    friend class Legend;
    friend class Plot;

    enum Type {
        POINTS, LINES, POLYGON
    };

    Object() = default;

    Object(Type type, const RGB32Color& color)
        : type_(type) {
        switch (type) {
        case POINTS:
            pen_.line_color = color;
            pen_.fill_color = color;
            pen_.fill_color.set_alpha(color.alpha() * 0.5);
            pen_.is_fill = true;
            pen_.line_width = 1.0;
            pen_.point_radius = 3.0;
            pen_.point_style = 'o';
            break;

        case LINES:
            pen_.line_color = color;
            pen_.fill_color = color;
            pen_.is_fill = false;
            pen_.line_width = 1.0;
            pen_.point_radius = 0.0;
            pen_.point_style = 'o';
            break;

        case POLYGON:
            pen_.line_color = color;
            pen_.fill_color = color;
            pen_.fill_color.set_alpha(color.alpha() * 0.5);
            pen_.is_fill = true;
            pen_.line_width = 1.0;
            pen_.point_radius = 0.0;
            pen_.point_style = 'o';
            break;

        default:
            break;
        }
    }

    bool empty() const {
        return data_.empty();
    }

    Object& name(const std::string& name) {
        name_ = name;
        return *this;
    }

    Object& point_style(char style) {
        CHECK(style == 'o' || style == 'r');

        pen_.point_style = style;
        return *this;
    }

    Object& point_radius(double radius) {
        CHECK(radius >= 0.0);

        pen_.point_radius = radius;
        return *this;
    }

    Object& line_width(double line_width) {
        CHECK(line_width >= 0.0);

        pen_.line_width = line_width;
        return *this;
    }

    Object& line_style(const std::string& style) {
        pen_.SetLineStyle(style);
        return *this;
    }

    Object& line_color(const RGB32Color& color) {
        pen_.line_color = color;
        return *this;
    }

    Object& line_color(int r, int g, int b, int a = 255) {
        return this->line_color(RGB32Color(r, g, b, a));
    }

    Object& color(const RGB32Color& color) {
        pen_.line_color = color;
        pen_.fill_color = color;

        if (type_ == POINTS || type_ == POLYGON) {
            pen_.fill_color.set_alpha(color.alpha() * 0.5);
        }

        return *this;
    }

    Object& color(int r, int g, int b, int a = 255) {
        return this->color(RGB32Color(r, g, b, a));
    }

    Object& fill_color(const RGB32Color& color) {
        pen_.fill_color = color;
        pen_.is_fill = true;
        return *this;
    }

    Object& fill_color(int r, int g, int b, int a = 255) {
        return fill_color(RGB32Color(r, g, b, a));
    }

    Object& no_fill() {
        pen_.is_fill = false;
        return *this;
    }

protected:
    /**
     * Add 1D data: line, lines, point or points.
     */
    template <typename Point>
    void AddData(const Array<Point>& data) {
        Array<RPoint2D> tmp;
        tmp.reserve(data.size());
        for (const Point& p : data) {
            tmp.emplace_back(static_cast<double>(p.x),
                             static_cast<double>(p.y));
        }
        data_.push_back(tmp);
    }
    void AddData(const Array<RPoint2D>& data) {
        data_.push_back(data);
    }

    /**
     * Add 2D data: polygon and multipolygon.
     */
    template <typename T>
    void AddData(const MultiPolygon2D<T>& polygon) {
        for (const auto& b : polygon.boundaries()) {
            this->AddData(b.polygon.vertices());
        }
    }

    std::string name_;
    Type type_;
    Pen pen_;
    Array<Array<RPoint2D>> data_;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_OBJECT_H_
