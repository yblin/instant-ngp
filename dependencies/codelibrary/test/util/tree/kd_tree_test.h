//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_UTIL_TREE_KD_TREE_TEST_H_
#define CODELIBRARY_TEST_UTIL_TREE_KD_TREE_TEST_H_

#include <random>

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/util/metric/euclidean.h"
#include "codelibrary/util/tree/kd_tree.h"

namespace cl {
namespace test {

/**
 * KD Tree Test.
 */
class KDTreeTest : public Test {
protected:
    KDTreeTest() = default;

    virtual void SetUp() override {
        // Generate point set for kd tree test.
        std::uniform_real_distribution<double> uniform(0.0, 1.0);

        points_.resize(100);
        for (auto & point : points_) {
            point.x = uniform(random_engine_);
            point.y = uniform(random_engine_);
            point.z = uniform(random_engine_);
        }

        kd_tree_.ResetPoints(points_);
    }

    // Distance metric.
    metric::Euclidean distance_;

    // KD tree.
    KDTree<RPoint3D, metric::Euclidean> kd_tree_;

    // Test point set.
    Array<RPoint3D> points_;

    // MT random engine for random points.
    std::mt19937 random_engine_;
};

TEST_F(KDTreeTest, FindNearest) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (int n_run = 0; n_run < 100; ++n_run) {
        RPoint3D p(uniform(random_engine_), uniform(random_engine_),
                   uniform(random_engine_));
        RPoint3D q = kd_tree_.FindNearestNeighbor(p);
        double distance = distance_(p, q);

        for (const auto & point : points_) {
            ASSERT_FALSE(distance_(point, p) + DBL_EPSILON < distance);
        }
    }
}

TEST_F(KDTreeTest, FindKNearestNeighbors) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    const int k = 10;

    for (int n_run = 0; n_run < 100; ++n_run) {
        RPoint3D p(uniform(random_engine_), uniform(random_engine_),
                   uniform(random_engine_));
        Array<int> res;
        kd_tree_.FindKNearestNeighbors(p, k, &res);
        double distance = distance_(p, points_[res.back()]);

        for (int i = 0; i < kd_tree_.size(); ++i) {
            int j = 0;
            for (j = 0; j < k; ++j) {
                if (i == res[j]) break;
            }

            if (j == k) {
                ASSERT_FALSE(distance_(points_[i], p) + DBL_EPSILON < distance);
            }
        }
    }
}

TEST_F(KDTreeTest, FindRaiusNeighbors) {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    const double radius = 0.1;

    for (int n_run = 0; n_run < 100; ++n_run) {
        RPoint3D p(uniform(random_engine_), uniform(random_engine_),
                   uniform(random_engine_));
        Array<int> res;
        kd_tree_.FindRadiusNeighbors(p, radius, &res);

        for (int i = 0; i < kd_tree_.size(); ++i) {
            int j = 0;
            for (j = 0; j < res.size(); ++j) {
                if (i == res[j]) break;
            }

            if (j == res.size()) {
                ASSERT_FALSE(distance_(points_[i], p) + DBL_EPSILON < radius);
            }
        }
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_UTIL_TREE_KD_TREE_TEST_H_
