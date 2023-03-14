//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_PREDICATE_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_PREDICATE_2D_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/predicate_2d.h"

namespace cl {
namespace test {

TEST(Predicate2DTest, InCircle) {
    RPoint2D p1(-21800.0, -10750.0);
    RPoint2D p2(24500.0,  -10750.0);
    RPoint2D p3(24500.0,  13750.0);
    RPoint2D p4(-21800.0, 13750.0);

    ASSERT_EQ(geometry::InCircle(p1, p3, p2, p4), 0);
    ASSERT_EQ(geometry::InCircle(p1, p2, p4, p3), 0);
    ASSERT_EQ(geometry::InCircle(p1, p2, p3, p2), 0);

    RPoint2D p5(0.0, 0.0);
    RPoint2D p6(1.0, 0.0);
    RPoint2D p7(0.0, 1.0);
    RPoint2D p8(0.0, 7.0);

    ASSERT_EQ(geometry::InCircle(p5, p6, p7, p8), -1);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_PREDICATE_2D_TEST_H_
