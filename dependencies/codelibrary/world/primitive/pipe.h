//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_PIPE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_PIPE_H_

#include <cfloat>

#include "codelibrary/geometry/intersect_3d.h"
#include "codelibrary/opengl/transform.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * The pipe can be seen as a polyline with a radius.
 */
struct Pipe : public RenderData {
    Pipe()
        : RenderData(GL_TRIANGLES) {}

    Pipe(const Array<FPoint3D>& points, float radius, int n_slices = 32)
        : RenderData(GL_TRIANGLES),
          points_(points),
          radius_(points.size(), radius) {
        CHECK(radius > 0.0f);

        Initialize(n_slices);
    }

    /**
     * Construct pipe with different radius at each vertex.
     */
    Pipe(const Array<FPoint3D>& points, const Array<float>& radius,
         int n_slices = 32)
        : RenderData(GL_TRIANGLES),
          points_(points),
          radius_(radius) {
        CHECK(points.size() == radius.size());

        Initialize(n_slices);
    }

    /**
     * Construct a cylinder.
     */
    Pipe(const FPoint3D& p1, const FPoint3D& p2, float radius,
         int n_slices = 32)
        : RenderData(GL_TRIANGLES),
          points_({p1, p2}),
          radius_(2, radius) {
        CHECK(radius > 0.0f);

        Initialize(n_slices);
    }

private:
    void Initialize(int n_slices) {
        CHECK(points_.size() == radius_.size());
        CHECK(n_slices >= 3);

        if (points_.size() <= 1) return;

        Array<FPoint3D> ps;
        float angle = 2.0f * float(M_PI) / n_slices;
        float r = 0.0f;
        for (int i = 0; i < n_slices; ++i, r += angle) {
            ps.emplace_back(std::cos(r), std::sin(r), 0.0f);
        }
        ps.push_back(ps.front());

        const float threshold = std::sqrt(FLT_EPSILON);
        FVector3D v = points_[1] - points_[0];

        gl::Transform transform;
        transform.Rotate(FVector3D(0.0f, 0.0f, 1.0f), FVector3D(v.x, v.y, v.z));
        transform.Translate(points_[0].ToVector());
        for (FPoint3D& p : ps) {
            p = transform(p);
        }

        // Set vertices and normals.
        for (int j = 0; j < points_.size(); ++j) {
            if (j > 0) {
                const FPoint3D& p0 = points_[j - 1];
                const FPoint3D& p1 = points_[j];

                FVector3D v1 = p0 - p1;
                float norm = v1.norm();
                if (norm <= FLT_EPSILON) continue;
                v1 *= 1.0f / norm;
                FVector3D v = v1;

                if (j + 1 < points_.size()) {
                    const FPoint3D& p2 = points_[j + 1];
                    FVector3D v2 = p2 - p1;
                    float norm = v2.norm();
                    if (norm > FLT_EPSILON) {
                        v2 *= 1.0f / norm;

                        FVector3D v3 = CrossProduct(v1, v2);
                        FVector3D v4 = v1 + v2;
                        v = CrossProduct(v3, v4);
                        if (v3.norm() < threshold || v4.norm() < threshold ||
                            v.norm() < threshold)
                            v = v1;
                    }
                }
                FPlane3D plane(FPoint3D(p1.x, p1.y, p1.z),
                               FVector3D(v.x, v.y, v.z));
                for (int i = 0; i < ps.size(); ++i) {
                    FLine3D line(ps[i], FVector3D(v1.x, v1.y, v1.z));
                    geometry::Cross(line, plane, &ps[i]);
                }
            }

            for (int i = 0; i < ps.size(); ++i) {
                vertices.push_back(points_[j] + (ps[i] - points_[j]) *
                                   radius_[j]);

                int next = i + 1 == ps.size() ? 1 : i + 1;
                int prev = i == 0 ? ps.size() - 2 : i - 1;
                FVector3D v1 = ps[prev] - ps[i];
                FVector3D v2 = ps[next] - ps[i];
                FVector3D v = -(v1 + v2);
                normals.push_back(v.Normalize());
            }

            if (j > 0 && j + 1 < points_.size()) {
                int offset = vertices.size() - ps.size();
                vertices.insert(vertices.begin() + offset,
                                vertices.begin() + offset + ps.size());
                normals.insert(normals.begin() + offset,
                               normals.begin() + offset + ps.size());
            }
        }

        // Set texture coordinates.
        for (int j = 0; j < points_.size(); ++j) {
            if (j == 0) {
                for (int i = 0; i < ps.size(); ++i) {
                    texture_coords.emplace_back(float(i) / n_slices, 0.0f);
                }
            } else if (j + 1 == points_.size()) {
                for (int i = 0; i < ps.size(); ++i) {
                    texture_coords.emplace_back(float(i) / n_slices, 1.0f);
                }
            } else {
                for (int i = 0; i < ps.size(); ++i) {
                    texture_coords.emplace_back(float(i) / n_slices, 1.0f);
                }
                for (int i = 0; i < ps.size(); ++i) {
                    texture_coords.emplace_back(float(i) / n_slices, 0.0f);
                }
            }

            if (j > 0) {
                int offset = 2 * ps.size() * (j - 1);
                for (int i = 0; i < n_slices; ++i) {
                    int next = i + 1;
                    indices.push_back(offset + i);
                    indices.push_back(offset + next);
                    indices.push_back(offset + i + ps.size());

                    indices.push_back(offset + i + ps.size());
                    indices.push_back(offset + next);
                    indices.push_back(offset + next + ps.size());
                }
            }
        }
    }

    // Vertices of the pipe.
    Array<FPoint3D> points_;

    // Radius of each vertex of the pipe.
    Array<float> radius_;

    // Number of slices of each circle.
    int n_slices = 0;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_PIPE_H_
