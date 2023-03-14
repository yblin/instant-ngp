//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_TEST_H_

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/envelope/convex_hull_2d.h"

namespace cl {
namespace test {

TEST(ConvexHull2DTest, ConvexHullOfNoPoints) {
    Array<RPoint2D> points;
    geometry::ConvexHull2D<double> convex_hull(points);
    ASSERT(convex_hull.empty());
}

TEST(ConvexHull2DTest, ConvexHullOfOnePoint) {
    Array<RPoint2D> points = { {0.0, 0.0} };

    geometry::ConvexHull2D<double> convex_hull(points);

    Array<RPoint2D> results = { {0.0, 0.0} };

    ASSERT_EQ_RANGE(convex_hull.vertices().begin(),
                   convex_hull.vertices().end(),
                   results.begin(),
                   results.end());
}

TEST(ConvexHull2DTest, ConvexHullOfTwoPoints) {
    Array<RPoint2D> points = { {0.0, 0.0}, {0.0, 1.0} };

    geometry::ConvexHull2D<double> convex_hull(points);

    Array<RPoint2D> results = { {0.0, 0.0}, {0.0, 1.0} };

    ASSERT_EQ_RANGE(convex_hull.vertices().begin(),
                   convex_hull.vertices().end(),
                   results.begin(),
                   results.end());
}

TEST(ConvexHull2DTest, ConvexHullOfThreePoints) {
    Array<RPoint2D> points = { {0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0} };
    geometry::ConvexHull2D<double> convex_hull(points);

    Array<RPoint2D> results = { {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0} };

    ASSERT_EQ_RANGE(convex_hull.vertices().begin(),
                   convex_hull.vertices().end(),
                   results.begin(),
                   results.end());
}

TEST(ConvexHull2DTest, GeneralCases) {
    Array<RPoint2D> points1 = {
        {4.4, 14.0}, {6.7, 15.25}, {6.9, 12.8},  {2.1, 11.1},
        {9.5, 14.9}, {13.2, 11.9}, {10.3, 12.3}, {6.8, 9.5},
        {3.3, 7.7},  {0.6, 5.1},   {5.3, 2.4},   {8.45, 4.7},
        {11.5, 9.6}, {13.8, 7.3},  {12.9, 3.1},  {11.0, 1.1}
    };

    geometry::ConvexHull2D<double> convex_hull1(points1);

    Array<RPoint2D> results1 = {
        {0.6, 5.1},  {5.3, 2.4},   {11.0, 1.1}, {12.9, 3.1},
        {13.8, 7.3}, {13.2, 11.9}, {9.5, 14.9}, {6.7, 15.25},
        {4.4, 14.0}, {2.1, 11.1} };

    ASSERT_EQ_RANGE(convex_hull1.vertices().begin(),
                   convex_hull1.vertices().end(),
                   results1.begin(),
                   results1.end());

    Array<RPoint2D> points2 = {
        {0.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {2.0, 0.0}, {1.0, 1.0}
    };
    geometry::ConvexHull2D<double> convex_hull2(points2);

    Array<RPoint2D> results2 = { {0.0, 0.0}, {2.0, 0.0}, {1.0, 1.0} };

    ASSERT_EQ_RANGE(convex_hull2.vertices().begin(),
                   convex_hull2.vertices().end(),
                   results2.begin(),
                   results2.end());
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_ENVELOPE_CONVEX_HULL_2D_TEST_H_
