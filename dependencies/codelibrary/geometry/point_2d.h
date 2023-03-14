//
// Copyright 2011-2021 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POINT_2D_H_
#define CODELIBRARY_GEOMETRY_POINT_2D_H_

#include <ostream>

#include "codelibrary/base/message.h"
#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/vector_2d.h"

namespace cl {

/**
 * 2D point.
 */
template<typename T>
class Point2D {
public:
    using value_type = T;

    Point2D() = default;

    Point2D(const T& x1, const T& y1)
        : x(x1), y(y1) {}

    template <typename Iter,
              typename = typename std::enable_if<std::is_convertible<
                         typename std::iterator_traits<Iter>::iterator_category,
                                  std::input_iterator_tag>::value>::type>
    Point2D(Iter first, Iter last) {
        CHECK(std::distance(first, last) == 2);

        x = *first++;
        y = *first;
    }

    bool operator ==(const Point2D& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    bool operator !=(const Point2D& rhs) const {
        return !(*this == rhs);
    }

    bool operator <(const Point2D& rhs) const {
        return x < rhs.x || (x == rhs.x && y < rhs.y);
    }

    bool operator <=(const Point2D& rhs) const {
        return !(rhs < *this);
    }

    bool operator >(const Point2D& rhs) const {
        return rhs < *this;
    }

    bool operator >=(const Point2D& rhs) const {
        return !(*this < rhs);
    }

    const Point2D& operator +=(const Vector2D<T>& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    const Point2D& operator -=(const Vector2D<T>& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    const Point2D& operator *=(const T& value) {
        x *= value;
        y *= value;
        return *this;
    }

    /**
     * Return the i-th component value of point.
     */
    const T& operator[] (int i) const {
        return i == 0 ? x : y;
    }

    /**
     * Return the reference value of the i-th component of point.
     */
    T& operator[] (int i) {
        return i == 0 ? x : y;
    }

    /**
     * Return the bounding box of this point.
     */
    Box2D<T> bounding_box() const {
        return Box2D<T>(x, x, y, y);
    }

    /**
     * Return the dimension.
     */
    int size() const {
        return 2;
    }

    Vector2D<T> ToVector() const {
        return Vector2D<T>(x, y);
    }

    friend Point2D operator +(const Point2D& lhs, const Vector2D<T>& rhs) {
        return Point2D<T>(lhs.x + rhs.x, lhs.y + rhs.y);
    }

    friend Point2D operator -(const Point2D& lhs, const Vector2D<T>& rhs) {
        return Point2D<T>(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    friend Vector2D<T> operator -(const Point2D& lhs, const Point2D& rhs) {
        return Vector2D<T>(lhs.x - rhs.x, lhs.y - rhs.y);
    }

    friend Point2D operator *(const Point2D& lhs, const T& v) {
        return Point2D<T>(lhs.x * v, lhs.y * v);
    }

    friend std::ostream& operator <<(std::ostream& os, const Point2D& p) {
        Message msg(p.x, p.y);
        return os << msg;
    }

    T x = 0; // X coordinate.
    T y = 0; // Y coordinate.
};

using IPoint2D = Point2D<int>;
using FPoint2D = Point2D<float>;
using RPoint2D = Point2D<double>;

} // namespace cl

namespace std {

template<typename T>
struct hash<cl::Point2D<T>> {
    size_t operator()(const cl::Point2D<T> &p) const {
        return std::hash<T>()(p.x) ^ std::hash<T>()(p.y);
    }
};

} // namespace std

#endif // CODELIBRARY_GEOMETRY_POINT_2D_H_
