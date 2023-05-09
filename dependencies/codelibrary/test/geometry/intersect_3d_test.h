//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_INTERSECT_3D_TEST_H
#define CODELIBRARY_TEST_GEOMETRY_INTERSECT_3D_TEST_H

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/intersect_3d.h"

namespace cl {
namespace test {

TEST(Intersect3DTest, PlaneLineIntersection) {
    RPlane3D plane(RPoint3D(0.0, 1.0, 0.0), RVector3D(0.0, 1.0, 0.0));
    RLine3D line(cl::RPoint3D(1.0, 2.0, 3.0), cl::RPoint3D(4.0, 5.0, 6.0));
    RPoint3D p;
    ASSERT(cl::geometry::Cross(plane, line, &p));
    ASSERT_EQ_NEAR(p.x, 0.0, 1e-12);
    ASSERT_EQ_NEAR(p.y, 1.0, 1e-12);
    ASSERT_EQ_NEAR(p.z, 2.0, 1e-12);

    RPlane3D plane1(RPoint3D(1.0, 1.0, 1.0), RVector3D(1.0, 0.0, 1.0));
    RLine3D line1(cl::RPoint3D(1.0, 2.0, 3.0), cl::RPoint3D(4.0, 5.0, 6.0));
    RPoint3D p1;
    ASSERT(cl::geometry::Cross(plane1, line1, &p1));
    ASSERT_EQ_NEAR(p1.x, 0.0, 1e-12);
    ASSERT_EQ_NEAR(p1.y, 1.0, 1e-12);
    ASSERT_EQ_NEAR(p1.z, 2.0, 1e-12);

    RPlane3D plane2(RPoint3D(1.0, 2.0, 3.0), RVector3D(1.0, 1.0, 1.0));
    RLine3D line2(cl::RPoint3D(1.0, 2.0, 3.0), cl::RPoint3D(4.0, 5.0, 6.0));
    RPoint3D p2;
    ASSERT(cl::geometry::Cross(plane2, line2, &p2));
    ASSERT_EQ(p2, cl::RPoint3D(1.0, 2.0, 3.0));
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_INTERSECT_3D_TEST_H
