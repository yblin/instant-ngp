//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Robust intersection test between 2D kernel geometric objects.
//

#ifndef CODELIBRARY_GEOMETRY_INTERSECT_2D_H_
#define CODELIBRARY_GEOMETRY_INTERSECT_2D_H_

#include <limits>

#include "codelibrary/base/clamp.h"
#include "codelibrary/base/index_sort.h"
#include "codelibrary/geometry/circle_2d.h"
#include "codelibrary/geometry/line_2d.h"
#include "codelibrary/geometry/multi_polygon_2d.h"
#include "codelibrary/geometry/point_2d.h"
#include "codelibrary/geometry/point_compare_2d.h"
#include "codelibrary/geometry/polygon_2d.h"
#include "codelibrary/geometry/predicate_2d.h"
#include "codelibrary/geometry/segment_2d.h"

namespace cl {
namespace geometry {

////////////////////////////////////////////////////////////////////////////////
// All Intersect() functions are used to test weather two objects intersect
// (including touch and contain).
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication and construction.
 */
template <typename T>
bool Intersect(const Box2D<T>& box1, const Box2D<T>& box2,
               Box2D<T>* res = nullptr) {
    if (box1.x_max() < box2.x_min() || box1.x_min() > box2.x_max() ||
        box1.y_max() < box2.y_min() || box1.y_min() > box2.y_max())
        return false;

    if (res) {
        *res = Box2D<T>(std::max(box1.x_min(), box2.x_min()),
                        std::min(box1.x_max(), box2.x_max()),
                        std::max(box1.y_min(), box2.y_min()),
                        std::min(box1.y_max(), box2.y_max()));
    }
    return true;
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& point, const Box2D<T>& box) {
    return (point.x >= box.x_min() && point.x <= box.x_max() &&
            point.y >= box.y_min() && point.y <= box.y_max());
}
template <typename T>
bool Intersect(const Box2D<T>& box, const Point2D<T>& point) {
    return Intersect(point, box);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& point, const Segment2D<T>& seg) {
    return point >= seg.lower_point() && point <= seg.upper_point() &&
           Orientation(seg.lower_point(), seg.upper_point(), point) == 0;
}
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Point2D<T>& point) {
    return Intersect(point, seg);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& point, const Line2D<T>& line) {
    return Orientation(line.point1(), line.point2(), point) == 0;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Point2D<T>& point) {
    return Intersect(point, line);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const Polygon2D<T>& polygon) {
    if (!Intersect(p, polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (int i = 0; i < polygon.size(); ++i) {
        const Segment2D<T> seg = polygon.edge(i);
        if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) continue;

        int o = Orientation(seg.lower_point(), seg.upper_point(), p);
        if (o == 0) return true;
        if (o < 0) ++count;
    }
    return count % 2 != 0;
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Point2D<T>& point) {
    return Intersect(point, polygon);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Point2D<T>& p, const MultiPolygon2D<T>& multi_polygon) {
    if (!Intersect(p, multi_polygon.bounding_box())) {
        return false;
    }

    int count = 0;
    for (const auto& b : multi_polygon.boundaries()) {
        const Polygon2D<T>& polygon = b.polygon;
        if (!Intersect(p, polygon.bounding_box())) continue;

        for (int j = 0; j < polygon.size(); ++j) {
            Segment2D<T> seg = polygon.edge(j);
            if (seg.lower_point().x > p.x || seg.upper_point().x <= p.x) {
                continue;
            }

            int o = Orientation(seg.lower_point(), seg.upper_point(), p);
            if (o == 0) return true;
            if (o < 0)  ++count;
        }
    }

    return (count % 2 != 0);
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& multi_polygon, const Point2D<T>& p) {
    return Intersect(p, multi_polygon);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Segment2D<T>& s1, const Segment2D<T>& s2) {
    if (!Intersect(s1.bounding_box(), s2.bounding_box())) return false;

    // Cross test.
    int o1 = Orientation(s1.lower_point(), s1.upper_point(), s2.lower_point());
    int o2 = Orientation(s1.lower_point(), s1.upper_point(), s2.upper_point());
    if (o1 == o2 && o1 != 0) {
        // Two endpoints of s2 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    int o3 = Orientation(s2.lower_point(), s2.upper_point(), s1.lower_point());
    int o4 = Orientation(s2.lower_point(), s2.upper_point(), s1.upper_point());

    // Two endpoints of s1 lay on the same side (left or right) of s2, thus
    // two segments never be intersected.
    return !(o3 == o4 && o3 != 0);
}

/**
 * Exact predication.
 *
 * Note that it is much slower than Cross(line, box).
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const Box2D<T>& box) {
    if (box.empty()) return false;

    Point2D<T> p1(box.x_min(), box.y_min());
    Point2D<T> p2(box.x_max(), box.y_min());
    Point2D<T> p3(box.x_max(), box.y_max());
    Point2D<T> p4(box.x_min(), box.y_max());
    return Intersect(line, Segment2D<T>(p1, p2)) ||
           Intersect(line, Segment2D<T>(p2, p3)) ||
           Intersect(line, Segment2D<T>(p3, p4)) ||
           Intersect(line, Segment2D<T>(p4, p1));
}
template <typename T>
bool Intersect(const Box2D<T>& box, const Line2D<T>& line) {
    return Intersect(line, box);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Line2D<T>& line) {
    int o1 = Orientation(line.point1(), line.point2(), seg.lower_point());
    if (o1 == 0) return true;

    int o2 = Orientation(line.point1(), line.point2(), seg.upper_point());
    if (o2 == 0) return true;

    return o1 != o2;
}
template <typename T>
bool Intersect(const Line2D<T>& line, const Segment2D<T>& seg) {
    return Intersect(seg, line);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const Polygon2D<T>& polygon,
               Array<Point2D<T>>* res = nullptr) {
    using Point = Point2D<T>;

    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!Intersect(line, polygon.bounding_box())) return false;

    // Make sure the polygon is in anti-clockwise.
    int n = polygon.size();
    // Generate 1 to n.
    Array<int> seq(n);
    std::iota(seq.begin(), seq.end(), 0);
    if (polygon.IsClockwise()) std::reverse(seq.begin(), seq.end());

    Point p1 = std::min(line.point1(), line.point2());
    Point p2 = std::max(line.point1(), line.point2());

    // Store the intersection points.
    int cur_o = 0, next_o = 0, prev_o = 0;
    for (int i = 0; i < n; ++i) {
        const Point& cur_p  = polygon.vertices()[seq[i]];
        const Point& next_p = i + 1 < n ? polygon.vertex(seq[i + 1])
                                        : polygon.vertex(seq[0]);
        const Point& prev_p = i == 0 ? polygon.vertex(seq.back())
                                     : polygon.vertex(seq[i - 1]);

        // Cross test.
        if (i == 0) {
            prev_o = Orientation(p1, p2, prev_p);
            cur_o  = Orientation(p1, p2, cur_p);
        } else {
            prev_o = cur_o;
            cur_o = next_o;
        }
        next_o = Orientation(p1, p2, next_p);

        if (cur_o == next_o && cur_o != 0) continue;
        if (cur_o == 0) {
            if (prev_o == next_o) continue;

            if (prev_o == 0) {
                if (cur_p < prev_p && next_o < 0) {
                    if (res)
                        res->emplace_back(cur_p.x, cur_p.y);
                    else
                        return true;
                }
                if (prev_p < cur_p && next_o > 0) {
                    if (res)
                        res->emplace_back(cur_p.x, cur_p.y);
                    else
                        return true;
                }
            } else if (next_o == 0) {
                if (next_p < cur_p && prev_o < 0) {
                    if (res)
                        res->emplace_back(cur_p.x, cur_p.y);
                    else
                        return true;
                }
                if (cur_p < next_p && prev_o > 0) {
                    if (res)
                        res->emplace_back(cur_p.x, cur_p.y);
                    else
                        return true;
                }
            } else {
                if (res)
                    res->emplace_back(cur_p.x, cur_p.y);
                else
                    return true;
            }
        } else if (cur_o != 0 && next_o != 0) {
            if (res) {
                Point2D<T> p;
                Cross(Segment2D<T>(p1, p2), Segment2D<T>(cur_p, next_p), &p);
                res->push_back(p);
            } else {
                return true;
            }
        }
    }

    return (res) ? (!res->empty()) : false;
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Line2D<T>& line,
               Array<Point2D<T>>* res = nullptr) {
    return Cross(line, polygon, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Line2D<T>& line, const MultiPolygon2D<T>& polygon,
               Array<Point2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!Intersect(line, polygon.bounding_box())) return false;

    for (const auto& b : polygon.boundaries()) {
        if (res) {
            Array<Point2D<T>> tmp;
            if (Intersect(line, b.polygon, &tmp)) res->insert(tmp);
        } else {
            if (Intersect(line, b.polygon)) return true;
        }
    }

    return (res) ? (!res->empty()) : false;
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon, const Line2D<T>& line,
               Array<Point2D<T>>* res = nullptr) {
    return Intersect(line, polygon, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const MultiPolygon2D<T>& polygon,
               Array<Segment2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    if (!res) {
        if (!Intersect(seg, polygon.bounding_box())) return false;

        if (Intersect(seg.lower_point(), polygon) ||
            Intersect(seg.upper_point(), polygon)) return true;

        for (int i = 0; i < polygon.size_boundaries(); ++i) {
            const Polygon2D<T>& poly = polygon.boundaries()[i].polygon;
            for (int j = 0; j < poly.size(); ++j) {
                if (Intersect(seg, poly.edge(i))) return true;
            }
        }

        return false;
    } else {
        Line2D<T> line(seg.lower_point(), seg.upper_point());

        Array<Point2D<T>> ps;
        if (!Intersect(line, polygon, &ps)) return false;
        CHECK(ps.size() % 2 == 0);

        PointDotCompare2D<T> compare(seg);
        std::sort(ps.begin(), ps.end(), compare);
        for (int i = 0; i < ps.size(); i += 2) {
            if (compare(seg.upper_point(), ps[i]) ||
                compare(ps[i + 1], seg.lower_point())) continue;

            res->emplace_back(std::max(ps[i], seg.lower_point(), compare),
                              std::min(ps[i + 1], seg.upper_point(), compare));
        }

        return !res->empty();
    }
}
template <typename T>
bool Intersect(const MultiPolygon2D<T>& polygon, const Segment2D<T>& seg,
               Array<Segment2D<T>>* res = nullptr) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Intersect(const Segment2D<T>& seg, const Polygon2D<T>& polygon,
               Array<Segment2D<T>>* res = nullptr) {
    MultiPolygon2D<T> poly(polygon);
    return Intersect(seg, poly, res);
}
template <typename T>
bool Intersect(const Polygon2D<T>& polygon, const Segment2D<T>& seg,
               Array<Segment2D<T>>* res = nullptr) {
    return Intersect(seg, polygon, res);
}

/**
 * Exact predication.
 */
template <typename T>
bool Intersect(const Polygon2D<T>& polygon1, const Polygon2D<T>& polygon2) {
    if (!Intersect(polygon1.bounding_box(), polygon2.bounding_box()))
        return false;

    for (int i = 0; i < polygon1.size(); ++i) {
        if (Intersect(polygon1.points()[i], polygon2)) return true;
        if (Intersect(polygon1.edge(i), polygon2)) return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////
// All Cross() functions are used to test weather two objects are cross
// (excluding touch and contain).
//
// Exact construction requires high-precision computation, which can greatly
// slow down the speed. In this library, we completely avoid such operations.
////////////////////////////////////////////////////////////////////////////////
/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Point2D<T>& point, const Box2D<T>& box) {
    return (point.x > box.x_min() && point.x < box.x_max() &&
            point.y > box.y_min() && point.y < box.y_max());
}
template <typename T>
bool Cross(const Box2D<T>& box, const Point2D<T>& point) {
    return Cross(point, box);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Segment2D<T>& seg) {
    // Cross test.
    int o1 = Orientation(line.point1(), line.point2(), seg.lower_point());
    if (o1 == 0) return false;
    int o2 = Orientation(line.point1(), line.point2(), seg.upper_point());
    if (o2 == 0) return false;
    return o1 != o2;
}
template <typename T>
bool Cross(const Segment2D<T>& seg, const Line2D<T>& line) {
    return Cross(line, seg);
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Line2D<T>& line, const Segment2D<T>& seg,
           Point2D<T>* res) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(res);

    if (!Cross(line, seg)) return false;

    Cross(line, Line2D<T>(seg.lower_point(), seg.upper_point()), res);
    const Box2D<T>& box = seg.bounding_box();
    res->x = Clamp(res->x, box.x_min(), box.x_max());
    res->y = Clamp(res->y, box.y_min(), box.y_max());
    return true;
}
template <typename T>
bool Cross(const Segment2D<T>& seg, const Line2D<T>& line,
           Point2D<T>* res) {
    return Cross(line, seg, res);
}

/**
 * Exact predication.
 */
template <typename T>
bool Cross(const Segment2D<T>& s1, const Segment2D<T>& s2) {
    if (!Intersect(s1.bounding_box(), s2.bounding_box())) return false;

    // Cross test.
    int o1 = Orientation(s1.lower_point(), s1.upper_point(), s2.lower_point());
    int o2 = Orientation(s1.lower_point(), s1.upper_point(), s2.upper_point());
    if (o1 == o2 || o1 == 0 || o2 == 0) {
        // Two endpoints of s2 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    int o3 = Orientation(s2.lower_point(), s2.upper_point(), s1.lower_point());
    int o4 = Orientation(s2.lower_point(), s2.upper_point(), s1.upper_point());
    if (o3 == o4 || o3 == 0 || o4 == 0) {
        // Two endpoints of s1 lay on the same side (left or right) of s1, thus
        // two segments never be intersected.
        return false;
    }

    return true;
}

/**
 * Inexact predication and construction.
 *
 * Exactly predicate whether two lines cross is unnecessary in most cases.
 * Because it is very rare that two lines are strictly parallel. If there is a
 * real need, it is better to convert lines into long line segments.
 */
template <typename T>
bool Cross(const Line2D<T>& line1, const Line2D<T>& line2,
           Point2D<T>* res = nullptr) {
    RPoint2D p1(line1.point1().x, line1.point1().y);
    RPoint2D p2(line1.point2().x, line1.point2().y);
    RPoint2D p3(line2.point1().x, line2.point1().y);
    RPoint2D p4(line2.point2().x, line2.point2().y);
    double t1 = p1.x * p2.y - p1.y * p2.x;
    double t2 = p3.x * p4.y - p3.y * p4.x;
    double t  = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

    double x = (t1 * (p3.x - p4.x) - (p1.x - p2.x) * t2) / t;
    double y = (t1 * (p3.y - p4.y) - (p1.y - p2.y) * t2) / t;
    if (!std::isfinite(x) || !std::isfinite(y)) return false;

    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->x = static_cast<T>(x);
        res->y = static_cast<T>(y);
    }
    return true;
}

/**
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Segment2D<T>& s1, const Segment2D<T>& s2, Point2D<T>* res) {
    static_assert(std::is_floating_point<T>::value, "");
    CHECK(res);

    if (!Cross(s1, s2)) return false;

    // There must be an intersection between two segments.
    Point2D<T> p;
    Line2D<T> line1(s1.lower_point(), s1.upper_point());
    Line2D<T> line2(s2.lower_point(), s2.upper_point());
    Cross(line1, line2, &p);

    // Ensure the intersection is inside the bounding box.
    Box2D<T> box;
    Intersect(s1.bounding_box(), s2.bounding_box(), &box);
    res->x = Clamp(p.x, box.x_min(), box.x_max());
    res->y = Clamp(p.y, box.y_min(), box.y_max());
    return true;
}

/**
 * Inexact predication and construction.
 *
 * Distance comparison is robust for predication. It is no need tos use exact
 * predication.
 */
template <typename T>
bool Cross(const Circle2D<T>& circle1, const Circle2D<T>& circle2,
           Segment2D<T>* res = nullptr) {
    double dx = static_cast<double>(circle2.center().x) - circle1.center().x;
    double dy = static_cast<double>(circle2.center().y) - circle1.center().y;

    double dis = std::sqrt(dx * dx + dy * dy);
    if (dis == 0.0) return false;

    double r1 = circle1.radius();
    double r2 = circle2.radius();

    // No intersection.
    if (dis > r1 + r2) return false;

    // One circle is contained in the other.
    if (dis < std::fabs(r1 - r2)) return false;
    if (!res) return true;

    if (res) {
        CHECK(std::is_floating_point<T>::value);

        double a = (r1 * r1 - r2 * r2 + dis * dis) / (2.0 * dis);
        double x2 = dx * a / dis + circle1.center().x;
        double y2 = dy * a / dis + circle1.center().y;

        double dis2 = std::sqrt((r1 * r1) - (a * a));
        double rx = -dy * (dis2 / dis);
        double ry =  dx * (dis2 / dis);
        *res = Segment2D<T>(Point2D<T>(static_cast<T>(x2 + rx),
                                       static_cast<T>(y2 + ry)),
                            Point2D<T>(static_cast<T>(x2 - rx),
                                       static_cast<T>(y2 - ry)));
    }

    return true;
}

/**
 * Check all intersection between line segments.
 *
 * Exact predication, inexact construction.
 */
template <typename T>
bool Cross(const Array<Segment2D<T>>& lines, Array<Point2D<T>>* res = nullptr) {
    if (res) {
        CHECK(std::is_floating_point<T>::value);
        res->clear();
    }

    int n = lines.size();

    // Get the sequence of segment by their lower endpoints' x coordinate.
    Array<T> values(n);
    for (int i = 0; i < n; ++i) {
        values[i] = lines[i].lower_point().x;
    }
    Array<int> seq;
    IndexSort(values.begin(), values.end(), &seq);

    // Get the intersection.
    Point2D<T> p;
    for (int i = 0; i < n; ++i) {
        T t = lines[seq[i]].upper_point().x;
        for (int j = i + 1; j < n && lines[seq[j]].lower_point().x <= t; ++j) {
            if (Cross(lines[seq[i]], lines[seq[j]], &p)) {
                if (!res)
                    return true;
                else
                    res->push_back(p);
            }
        }
    }

    return res ? (!res->empty()) : false;
}

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_INTERSECT_2D_H_
