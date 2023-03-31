//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_
#define CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_

#include "codelibrary/geometry/mesh/obj_io.h"
#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/util/io/file_util.h"
#include "codelibrary/world/kernel/render_data.h"
#include "codelibrary/world/primitive/quad.h"

namespace cl {
namespace world {

/**
 * Mesh render data.
 */
struct MeshData : public RenderData {
    MeshData() : RenderData(GL_TRIANGLES) {}

    /**
     * Load mesh from SurfaceMesh.
     */
    bool Load(const geometry::SurfaceMesh<FPoint3D>& mesh) {
        if (mesh.empty()) return false;

        int id = 0;
        std::unordered_map<FPoint3D, int> hash;
        this->clear();

        vertices.reserve(mesh.n_vertices());
        normals.reserve(mesh.n_vertices());
        for (auto v : mesh.vertices()) {
            auto pair = hash.insert({v->point(), id});
            if (pair.second) {
                ++id;
                vertices.push_back(v->point());
            }
        }

        Array<int> temp;
        for (auto f : mesh.faces()) {
            // We assume that the face is convex.
            temp.clear();
            for (auto e : mesh.circular_list(f->edge())) {
                temp.push_back(hash[e->source_point()]);
            }
            if (temp.size() == 3)
                indices.insert(temp);
            else {
                for (int i = 1; i + 1 < temp.size(); ++i) {
                    indices.push_back(temp[0]);
                    indices.push_back(temp[i]);
                    indices.push_back(temp[i + 1]);
                }
            }
        }

        auto normal_map = mesh.GetVertexNormals();
        normals.resize(vertices.size());
        for (auto v : mesh.vertices()) {
            normals[hash[v->point()]] = normal_map[v];
        }

        return true;
    }

    /**
     * Load mesh data from local file.
     */
    bool Load(const std::string& filename) {
        std::string suffix = file_util::GetSuffix(filename);

        geometry::SurfaceMesh<FPoint3D> surface_mesh;
        if (suffix == "obj") {
            geometry::OBJLoader obj;
            if (!obj.Open(filename)) return false;
            if (!obj.Load(&surface_mesh)) return false;
        } else {
            LOG(WRONG) << "Unsupport mesh format: " << filename;
            return false;
        }

        return this->Load(surface_mesh);
    }
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_MESH_DATA_H_
