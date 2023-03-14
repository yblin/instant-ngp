//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_POLYLINE_2D_H_
#define CODELIBRARY_GEOMETRY_POLYLINE_2D_H_

#include "codelibrary/base/array.h"
#include "codelibrary/geometry/box_2d.h"
#include "codelibrary/geometry/point_2d.h"

namespace cl {

/**
 * 2D polyline. It is implemented like Array<Point2D>.
 */
template <typename T>
class Polyline2D {
public:
    Polyline2D() = default;

    explicit Polyline2D(const Array<Point2D<T>>& points)
        : vertices_(points) {}

    bool empty() const {
        return vertices_.empty();
    }

    void clear() {
        vertices_.clear();
    }

    /**
     * Equal to Array's push_back.
     */
    void push_back(const Point2D<T>& p) {
        vertices_.push_back(p);
    }

    /**
     * Equal to Array's emplace back.
     */
    template <typename... Args>
    void emplace_back(Args&&... args) {
        vertices_.emplace_back(std::forward<Args>(args)...);
    }

    const Array<Point2D<T>>& vertices() const {
        return vertices_;
    }

protected:
    Array<Point2D<T>> vertices_;
};

using IPolyline2D = Polyline2D<int>;
using FPolyline2D = Polyline2D<float>;
using RPolyline2D = Polyline2D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_POLYGON_2D_H_
