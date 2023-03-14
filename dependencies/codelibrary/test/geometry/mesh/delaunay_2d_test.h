//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_
#define CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_

#include <random>

#include "codelibrary/base/testing.h"
#include "codelibrary/geometry/angle.h"
#include "codelibrary/geometry/mesh/delaunay_2d.h"

namespace cl {
namespace test {

class Delaunay2DTest : public cl::Test {
public:
    /**
     * ASSERT if this Delaunay triangulation is valid.
     * Note the time complexity is O(N^2). It is only used in debug.
     */
    template <typename T>
    bool IsValid(const geometry::Delaunay2D<T>& dt) {
        using Vertex = typename geometry::Delaunay2D<T>::Vertex;
        using Halfedge = typename geometry::Delaunay2D<T>::Halfedge;

        const auto& mesh = dt.mesh();
        if (mesh.n_vertices() < 2) return true;

        for (auto e : mesh) {
            if (dt.is_outer(e)) continue;
            if (!e->next() ||
                !e->next()->next() ||
                e->next()->next()->next() != e ||
                !e->prev() ||
                !e->prev()->prev() ||
                e->prev()->prev()->prev() != e) {
                return false;
            }

            const Halfedge* e_next = e->next();
            const Halfedge* e_prev = e->prev();

            // The triangle should be form a empty circle.
            for (const Vertex* v : mesh.vertices()) {
                if (geometry::InCircle(e->source_point(),
                                       e_next->source_point(),
                                       e_prev->source_point(),
                                       v->point()) > 0) {
                    return false;
                }
            }
        }
        return true;
    }
};

TEST_F(Delaunay2DTest, EmptyInputPoints) {
    Array<RPoint2D> points;
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, OneInputPoint) {
    Array<RPoint2D> points = {{0.0, 0.0}};
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, TwoInputPoints) {
    Array<RPoint2D> points = { {1.0, 1.0}, {0.0, 1.0} };
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, FourInputPoints) {
    Array<RPoint2D> points = { {-24304000.0, -858951.0},
                               {-904000.0,   -858951.0},
                               {-904000.0,   11041049.0},
                               {-24304000.0, 11041049.0} };
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, CollinearInputPoints) {
    Array<RPoint2D> points(100);
    for (int i = 0; i < points.size(); ++i) {
        points[i].x = points[i].y = static_cast<double>(i);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, UniformGridInputPoints) {
    Array<RPoint2D> points(100);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            points[i * 10 + j].x = i;
            points[i * 10 + j].y = j;
        }
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, ConcyclicInputPoints) {
    Array<RPoint2D> points(360);
    for (int i = 0; i < 360; ++i) {
        points[i].x = std::cos(i * M_PI / 180.0);
        points[i].y = std::sin(i * M_PI / 180.0);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, RandomInputPoints) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100);
    for (int i = 0; i < 100; ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);
    ASSERT(IsValid(delaunay));
}

TEST_F(Delaunay2DTest, RandomInsert) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    geometry::Delaunay2D<double> delaunay;
    for (int i = 0; i < 100; ++i) {
        double x = uniform(random);
        double y = uniform(random);
        delaunay.Insert(RPoint2D(x, y));
        ASSERT(IsValid(delaunay));
    }
}


TEST_F(Delaunay2DTest, RandomEraseVertices) {
    std::mt19937 random;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100);
    for (int i = 0; i < points.size(); ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);

    Array<int> seq(points.size());
    for (int i = 0; i < points.size(); ++i) {
        seq[i] = i;
    }
    std::shuffle(seq.begin(), seq.end(), random);
    for (int i : seq) {
        delaunay.Erase(points[i]);
        ASSERT(IsValid(delaunay));
    }
}

TEST_F(Delaunay2DTest, Performance100000) {
    std::mt19937 random;

    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    Array<RPoint2D> points(100000);
    for (int i = 0; i < 100000; ++i) {
        points[i].x = uniform(random);
        points[i].y = uniform(random);
    }
    geometry::Delaunay2D<double> delaunay(points);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GEOMETRY_MESH_DELAUNAY_2D_TEST_H_
