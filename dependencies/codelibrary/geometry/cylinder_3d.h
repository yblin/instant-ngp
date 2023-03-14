//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_CYLINDER_3D_H_
#define CODELIBRARY_GEOMETRY_CYLINDER_3D_H_

#include "codelibrary/geometry/point_3d.h"

namespace cl {

template <typename T>
class Cylinder3D {
public:
    Cylinder3D() = default;

    Cylinder3D(const Point3D<T>& p1, const Point3D<T>& p2, const T& radius)
        : point1_(p1), point2_(p2), radius_(radius) {
        CHECK(radius >= 0);
    }

    const Point3D<T>& point1() const {
        return point1_;
    }

    const Point3D<T>& point2() const {
        return point2_;
    }

    const T& radius() const {
        return radius_;
    }

private:
    // Two end points of cylinder.
    Point3D<T> point1_, point2_;

    T radius_ = 0;
};

using ICylinder3D = Cylinder3D<int>;
using FCylinder3D = Cylinder3D<float>;
using RCylinder3D = Cylinder3D<double>;

} // namespace cl

#endif // CODELIBRARY_GEOMETRY_CYLINDER_3D_H_
