//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_CUBE_SPHERE_H_
#define CODELIBRARY_WORLD_PRIMITIVE_CUBE_SPHERE_H_

#include "codelibrary/geometry/angle.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Similar to subdividing an icosahedron, a cubesphere is subdividing a cube N
 * times to construct a sphere geometry. The characteristic of a cubesphere is
 * that the spherical surface of the sphere is decomposed into 6 equal-area
 * regions (+X, -X, +Y, -Y, +Z and -Z faces). It is somewhat related to the cube
 * map, which is a method of environment mapping in computer graphics.
 */
struct CubeSphere : public RenderData {
    CubeSphere()
        : RenderData(GL_TRIANGLES) {}

    explicit CubeSphere(int subdivision)
        : RenderData(GL_TRIANGLES) {
        CHECK(subdivision >= 0);
        CHECK(subdivision <= 8);

        // Generate unit-length verties in +X face;
        Array<FPoint3D> unit_vertices;
        int vertex_count_per_row = (2 << subdivision) + 1;
        GetUnitPositiveX(vertex_count_per_row, &unit_vertices);

        Array<int> line_indices;

        // +X face.
        int k = 0;
        for (int i = 0; i < vertex_count_per_row; ++i) {
            int k1 = i * vertex_count_per_row;  // Index at current row.
            int k2 = k1 + vertex_count_per_row; // Index at next row.

            // Texture coords.
            float t = float(i) / (vertex_count_per_row - 1);
            for (int j = 0; j < vertex_count_per_row; ++j, ++k1, ++k2, ++k) {
                float s = float(j) / (vertex_count_per_row - 1);
                vertices.push_back(unit_vertices[k]);
                normals.emplace_back(unit_vertices[k].ToVector());
                texture_coords.emplace_back(s, t);

                if (i + 1 < vertex_count_per_row &&
                    j + 1 < vertex_count_per_row) {
                    indices.push_back(k1);
                    indices.push_back(k2);
                    indices.push_back(k1 + 1);
                    indices.push_back(k1 + 1);
                    indices.push_back(k2);
                    indices.push_back(k2 + 1);

                    // Add line indices; top and left.
                    line_indices.push_back(k1); // left
                    line_indices.push_back(k2);
                    line_indices.push_back(k1); // top
                    line_indices.push_back(k1 + 1);
                }
            }
        }

        int vertex_size = vertices.size();
        int line_index_size = line_indices.size();
        int start_index = vertices.size();

        // Build -X face by negating x and z values.
        for (int i = 0; i < vertex_size; ++i) {
            const FPoint3D& p = vertices[i];
            const FVector3D& n = normals[i];

            vertices.emplace_back(-p.x, p.y, -p.z);
            normals.emplace_back(-n.x, n.y, -n.z);
            texture_coords.push_back(texture_coords[i]);
        }
        for (int i = 0; i < line_index_size; i += 4) {
            // Left and bottom lines.
            line_indices.push_back(start_index + line_indices[i]);     // left
            line_indices.push_back(start_index + line_indices[i + 1]);
            line_indices.push_back(start_index + line_indices[i + 1]); // bottom
            line_indices.push_back(start_index + line_indices[i + 1] + 1);
        }

        // Build +Y face by swapping x=>y, y=>-z, z=>-x.
        start_index = vertices.size();
        for (int i = 0; i < vertex_size; ++i) {
            const FPoint3D& p = vertices[i];
            const FVector3D& n = normals[i];

            vertices.emplace_back(-p.z, p.x, -p.y);
            normals.emplace_back(-n.z, n.x, -n.y);
            texture_coords.push_back(texture_coords[i]);
        }
        for (int i = 0; i < line_index_size; ++i) {
            // Top and left lines (same as +X)
            line_indices.push_back(start_index + line_indices[i]);
        }

        // Build -Y face by swapping x=>-y, y=>z, z=>-x.
        start_index = vertices.size();
        for (int i = 0; i < vertex_size; ++i) {
            const FPoint3D& p = vertices[i];
            const FVector3D& n = normals[i];

            vertices.emplace_back(-p.z, -p.x, p.y);
            normals.emplace_back(-n.z, -n.x, n.y);
            texture_coords.push_back(texture_coords[i]);
        }
        for (int i = 0; i < line_index_size; i += 4) {
            // Top and right lines
            line_indices.push_back(start_index + line_indices[i]);     // top
            line_indices.push_back(start_index + line_indices[i + 3]);
            line_indices.push_back(start_index + line_indices[i] + 1); // right
            line_indices.push_back(start_index + line_indices[i + 1] + 1);
        }

        // Build +Z face by swapping x=>z, z=>-x.
        start_index = vertices.size();
        for (int i = 0; i < vertex_size; ++i) {
            const FPoint3D& p = vertices[i];
            const FVector3D& n = normals[i];

            vertices.emplace_back(-p.z, p.y, p.x);
            normals.emplace_back(-n.z, n.y, n.x);
            texture_coords.push_back(texture_coords[i]);
        }
        for (int i = 0; i < line_index_size; ++i) {
            // Top and left lines (same as +X)
            line_indices.push_back(start_index + line_indices[i]);
        }

        // Build -Z face by swapping x=>-z, z=>x.
        start_index = vertices.size();
        for (int i = 0; i < vertex_size; ++i) {
            const FPoint3D& p = vertices[i];
            const FVector3D& n = normals[i];

            vertices.emplace_back(p.z, p.y, -p.x);
            normals.emplace_back(n.z, n.y, -n.x);
            texture_coords.push_back(texture_coords[i]);
        }

        // Compute indices.
        int indice_size = indices.size();
        start_index = vertex_size;
        for (int j = 0; j < 5; ++j) {
            for (int i = 0; i < indice_size; ++i) {
                indices.push_back(start_index + indices[i]);
            }
            start_index += vertex_size;
        }
    }

protected:
    /**
     * Generate vertices for +X face only by intersecting 2 circular planes
     * (longitudinal and latitudinal) at the longitude/latitude angles.
     */
    static void GetUnitPositiveX(int points_per_row,
                                 Array<FPoint3D>* unit_vertices) {
        unit_vertices->clear();

        FVector3D n1; // Normal of longitudinal plane rotating along Y-axis.
        FVector3D n2; // Normal of latitudinal plane rotating along Z-axis.

        // Rotate latitudinal plane from 45 to -45 degrees along Z-axis.
        for (int i = 0; i < points_per_row; ++i) {
            // Normal for latitudinal plane;
            float a2 = DegreeToRadian(45.0f - 90.0f * i / (points_per_row - 1));
            n2.x = -std::sin(a2);
            n2.y = std::cos(a2);
            n2.z = 0.0f;

            // Rotate longitudinal plane from -45 to 45 along Y-axis.
            for (int j = 0; j < points_per_row; ++j) {
                // Normal for longitudinal plane.
                float a1 = DegreeToRadian(-45.0f + 90.0f * j /
                                          (points_per_row - 1));
                n1.x = -std::sin(a1);
                n1.y = 0.0f;
                n1.z = -std::cos(a1);

                // Find direction vector of intersected line, n1 x n2.
                FVector3D v = Normalize(CrossProduct(n1, n2));

                unit_vertices->emplace_back(v.x, v.y, v.z);
            }
        }
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_CUBE_SPHERE_H_
