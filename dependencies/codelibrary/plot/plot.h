//
// Copyright 2016-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_PLOT_PLOT_H_
#define CODELIBRARY_PLOT_PLOT_H_

#include <string>

#include "codelibrary/geometry/polyline_2d.h"
#include "codelibrary/geometry/triangle_2d.h"
#include "codelibrary/plot/base_plot.h"
#include "codelibrary/plot/object.h"
#include "codelibrary/plot/terminal.h"
#include "codelibrary/util/color/color_map.h"

namespace cl {
namespace plot {

/**
 * Plot for geometric objects.
 *
 * Sample usage:
 *
 *   // Input data.
 *   Array<RPoint2D> data1(100), data2(100);
 *   for (int i = 0; i < 100; ++i) {
 *       double x = 2.0 * M_PI * i / 100.0;
 *       data1[i].x = data2[i].x = x;
 *       data1[i].y = std::sin(x);
 *       data2[i].y = std::cos(x);
 *   }
 *
 *   // Plotting.
 *   plot::Plot plot;
 *   plot.Draw(Polyline(data1));
 *   plot.Draw(Polyline(data2));
 *
 *   // Save the plot to the file.
 *   plot.Save("test.svg");
 */
class Plot : public BasePlot {
public:
    explicit Plot(const ColorMap& color_map = ColorMap::Lines())
        : color_map_(color_map) {}

    /**
     * Return true if this plot is empty.
     */
    virtual bool empty() const override {
        return objects_.empty();
    }

    /**
     * Clear this plot.
     */
    virtual void clear() override {
        objects_.clear();
    }

    /**
     * Draw a point.
     */
    template <typename T>
    Object& Draw(const Point2D<T>& p) {
        Object o(Object::POINTS, get_color());
        Array<RPoint2D> points;
        points.emplace_back(p.x, p.y);
        o.AddData(points);
        return DrawObject(o);
    }

    /**
     * Draw a point set.
     */
    template <typename T>
    Object& Draw(const Array<Point2D<T>>& points) {
        Object o(Object::POINTS, get_color());
        o.AddData(points);
        return DrawObject(o);
    }

    /**
     * Draw a line.
     */
    template <typename T>
    Object& Draw(const Segment2D<T>& line) {
        Array<RPoint2D> points(2);
        Object o(Object::LINES, get_color());
        points[0] = line.lower_point();
        points[1] = line.upper_point();
        o.AddData(points);
        return DrawObject(o);
    }

    /**
     * Draw a line segment set.
     */
    template <typename T>
    Object& Draw(const Array<Segment2D<T>>& lines) {
        Array<RPoint2D> points(2);
        Object o(Object::LINES, get_color());
        for (const auto& l : lines) {
            points[0] = l.lower_point();
            points[1] = l.upper_point();
            o.AddData(points);
        }
        return DrawObject(o);
    }

    /**
     * Draw a polyline.
     */
    template <typename T>
    Object& Draw(const Polyline2D<T>& polyline) {
        Object o(Object::LINES, get_color());
        o.AddData(polyline.vertices());
        return DrawObject(o);
    }

    /**
     * Draw a polyline of X and Y array.
     */
    template <typename T>
    Object& Draw(const Array<T>& x, const Array<T>& y) {
        CHECK(x.size() == y.size());

        Polyline2D<T> polyline;
        for (int i = 0; i < x.size(); ++i) {
            polyline.emplace_back(x[i], y[i]);
        }
        return this->Draw(polyline);
    }

    /**
     * Draw a polygon.
     */
    template <typename T>
    Object& Draw(const Polygon2D<T>& polygon) {
        return Draw(MultiPolygon2D<T>(polygon));
    }
    template <typename T>
    Object& Draw(const MultiPolygon2D<T>& polygon) {
        Object o(Object::POLYGON, get_color());
        o.AddData(polygon);
        return DrawObject(o);
    }

    /**
     * Draw a box.
     */
    template <typename T>
    Object& Draw(const Box2D<T>& box) {
        return Draw(Polygon2D<T>(box));
    }

    /**
     * Draw a triangle.
     */
    template <typename T>
    Object& Draw(const Triangle2D<T>& triangle) {
        return Draw(Polygon2D<T>(triangle.vertices()));
    }

    /**
     * Draw plottable data on the terminal.
     */
    virtual void DrawData(Terminal* terminal) override {
        for (const Object& o : objects_) {
            if (o.data_.empty()) continue;
            if (!o.name_.empty()) {
                legend_.InsertItem(o);
            }

            terminal->set_pen(o.pen_);

            switch (o.type_) {
            case Object::POINTS:
                DrawPoints(o, terminal);
                break;
            case Object::LINES:
                DrawLines(o, terminal);
                DrawPoints(o, terminal);
                break;
            case Object::POLYGON:
                DrawPolygon(o, terminal);
                DrawLineLoop(o, terminal);
                DrawPoints(o, terminal);
                break;
            }
        }
    }

protected:
    RGB32Color get_color() const {
        return color_map_[objects_.size()];
    }

    /**
     * Draw points on the terminal.
     */
    void DrawPoints(const Object& o, Terminal* terminal) const {
        if (o.pen_.point_radius <= 0.0) return;

        for (const auto& data : o.data_) {
            for (auto p : data) {
                const RPoint2D q = ToPlotPosition(p);
                terminal->DrawPoint(q.x, q.y);
            }
        }
    }

    /**
     * Draw lines on the terminal.
     */
    void DrawLines(const Object& o, Terminal* terminal) const {
        if (o.pen_.line_width <= 0.0) return;

        for (const auto& data : o.data_) {
            if (data.empty()) continue;

            Array<RPoint2D> polyline;
            for (auto p : data) {
                const RPoint2D q = ToPlotPosition(p);
                polyline.emplace_back(q.x, q.y);
            }
            terminal->DrawPolyline(polyline);
        }
    }

    /**
     * Draw line loop on the terminal.
     */
    void DrawLineLoop(const Object& o, Terminal* terminal) const {
        if (o.pen_.line_width <= 0.0) return;

        for (const auto& data : o.data_) {
            if (data.empty()) continue;

            Array<RPoint2D> polyline;
            for (auto p : data) {
                const RPoint2D q = ToPlotPosition(p);
                polyline.emplace_back(q.x, q.y);
            }
            polyline.push_back(polyline.front());
            terminal->DrawPolyline(polyline);
        }
    }

    /**
     * Draw polygon on the terminal.
     */
    void DrawPolygon(const Object& o, Terminal* terminal) const {
        if (!o.pen_.is_fill) return;

        RMultiPolygon2D poly;
        for (const auto& data : o.data_) {
            Array<RPoint2D> points;
            for (const RPoint2D& p : data) {
                RPoint2D q = ToPlotPosition(p);
                points.push_back(q);
            }
            RPolygon2D polygon(points);
            if (!polygon.empty()) poly.Insert(polygon, true);
        }
        terminal->DrawPolygon(poly);
    }

    /**
     * Draw polyline determined by the given points with specific pen.
     */
    Object& DrawObject(const Object& object) {
        // Update the data range.
        Array<RPoint2D> points;
        for (auto d : object.data_) {
            points.insert(d);
        }
        RBox2D box(points.begin(), points.end());
        if (objects_.empty() && !box.empty())
            data_range_ = box;
        else
            data_range_.Join(box);

        objects_.push_back(object);

        return objects_.back();
    }

    // Color map to draw the objects.
    const ColorMap& color_map_;

    // The input data for plotting.
    Array<Object> objects_;
};

} // namespace plot
} // namespace cl

#endif // CODELIBRARY_PLOT_PLOT_H_
