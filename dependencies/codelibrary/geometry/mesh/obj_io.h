//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_
#define CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_

#include "codelibrary/geometry/mesh/surface_mesh.h"
#include "codelibrary/geometry/point_3d.h"
#include "codelibrary/string/string_split.h"
#include "codelibrary/util/io/line_reader.h"

namespace cl {
namespace geometry {

/**
 * Load OBJ format file.
 */
class OBJLoader {
public:
    OBJLoader() = default;

    OBJLoader(const std::string& filename) {
        Open(filename);
    }

    OBJLoader(const OBJLoader&) = delete;

    OBJLoader& operator=(const OBJLoader&) = delete;

    /**
     * Open file for loading.
     */
    bool Open(const std::string& filename) {
        return line_reader_.Open(filename);
    }

    /**
     * Check if a OBJ file is open for loading.
     */
    bool is_open() const {
        return line_reader_.is_open();
    }

    /**
     * Close the open file.
     */
    void Close() {
        line_reader_.Close();
    }

    /**
     * Load OBJ into surface mesh.
     *
     * Return false if error occurs.
     */
    template <typename T>
    bool Load(SurfaceMesh<Point3D<T>>* mesh, bool swap_y_and_z = false) {
        static_assert(std::is_floating_point<T>::value, "");

        CHECK(mesh);

        mesh->clear();

        if (!is_open()) {
            LOG(INFO) << "No open file for loading. Forget to call Open()?";
            return false;
        }

        Array<std::string> elements;
        Array<std::string> values;
        double x, y, z, w;
        int id;

        using Vertex = typename SurfaceMesh<Point3D<T>>::Vertex;
        Array<int> indices;
        Array<Vertex*> vertices;
        Array<Vertex*> polygon;
        Point3D<T> p;

        char* line = nullptr;
        while ((line = line_reader_.ReadLine())) {
            if (line[0] == '#') continue;
            if (line[0] == 'v' && line[1] == ' ') {
                if (std::sscanf(line + 2, "%lf %lf %lf %lf",
                                &x, &y, &z, &w) < 3) {
                    LOG(WRONG) << ErrorMessage();
                    mesh->clear();
                    return false;
                }

                if (swap_y_and_z) {
                    p = Point3D<T>(x, z, y);
                } else {
                    p = Point3D<T>(x, y, z);
                }
                vertices.push_back(mesh->AddVertex(p));
            } else if (line[0] == 'f' && line[1] == ' ') {
                StringSplit(line, ' ', &elements);
                indices.clear();
                for (int i = 1; i < elements.size(); ++i) {
                    if (elements[i].empty()) continue;
                    StringSplit(elements[i], '/', &values);
                    if (values.empty()) {
                        LOG(WRONG) << ErrorMessage();
                        mesh->clear();
                        return false;
                    }

                    if (std::sscanf(values[0].c_str(), "%d", &id) != 0) {
                        if (id <= 0) {
                            LOG(WRONG) << ErrorMessage("Index must be greater "
                                                       "than 1.");
                            mesh->clear();
                            return false;
                        }
                        indices.push_back(id);
                    }
                }

                if (indices.size() < 3) {
                    LOG(WRONG) << ErrorMessage("The number of indices of the "
                                               "face should not smaller than "
                                               "3.");
                    mesh->clear();
                    return false;
                }

                if (swap_y_and_z) std::reverse(indices.begin(), indices.end());

                polygon.resize(indices.size());
                for (int i = 0; i < indices.size(); ++i) {
                    if (indices[i] > vertices.size()) {
                        LOG(WRONG) << ErrorMessage("Invalid index detected.");
                        mesh->clear();
                        return false;
                    }
                    polygon[i] = vertices[indices[i] - 1];
                }
                mesh->AddFace(polygon);
            }
        }

        return !mesh->empty();
    }

private:
    /**
     * Return common error message.
     */
    std::string ErrorMessage(const std::string& message = "") const {
        return "Invalid OBJ format at line " +
               std::to_string(line_reader_.n_line()) + ". " + message;
    }

    io::LineReader line_reader_;
};

} // namespace geometry
} // namespace cl

#endif // CODELIBRARY_GEOMETRY_MESH_OBJ_IO_H_
