//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_
#define CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_

#include <unordered_map>

#include "codelibrary/base/pool.h"
#include "codelibrary/geometry/mesh/obj_io.h"
#include "codelibrary/util/io/file_util.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/node.h"
#include "codelibrary/world/primitive/cube.h"
#include "codelibrary/world/primitive/cube_sphere.h"
#include "codelibrary/world/primitive/cylinder.h"
#include "codelibrary/world/primitive/mesh_data.h"
#include "codelibrary/world/primitive/pipe.h"
#include "codelibrary/world/primitive/quad.h"
#include "codelibrary/world/primitive/sphere.h"

namespace cl {
namespace world {

/**
 * Factory to manager the life of primitive mesh nodes.
 */
class PrimitiveFactory {
    PrimitiveFactory() = default;

public:
    static PrimitiveFactory* GetInstance() {
        static PrimitiveFactory factory;
        return &factory;
    }

    /**
     * Create a node that contain given render object.
     */
    Node* CreateNode(const std::string& name, RenderObject* object) {
        std::string type;
        switch (object->type()) {
        case GL_POINTS:
            type = "Points";
            break;
        case GL_LINES:
            type = "Lines";
            break;
        case GL_TRIANGLES:
            type = "Mesh";
            break;
        default:
            CHECK(false) << "Unknown type.";
        }

        Node* node = nodes_.Allocate();
        node->set_name(GetID(name, type));
        node->AddRenderObject(object);
        return node;
    }
    Node* CreateNode(RenderObject* object) {
        return CreateNode("", object);
    }

    /**
     * Create an unit cube.
     */
    Node* CreateCube(const std::string& name = "") {
        return CreateNode(name, "Cube", Cube());
    }

    /**
     * Create an unit cube sphere with quality (the value range is [2, 128]).
     */
    Node* CreateCubeSphere(const std::string& name, int subdivision = 4) {
        CubeSphere sphere(subdivision);
        return CreateNode(name, "Cubesphere", sphere);
    }
    Node* CreateCubeSphere(int subdivision = 4) {
        return CreateCubeSphere("", subdivision);
    }

    /**
     * Create a pipe with n_slices (the value range is [2, 128]).
     */
    Node* CreatePipe(const std::string& name, const Array<FPoint3D>& points,
                     float radius = 1.0f, int n_slices = 32) {
        Pipe pipe(points, radius, n_slices);
        return CreateNode(name, "Pipe", pipe);
    }
    Node* CreatePipe(const Array<FPoint3D>& points, float radius = 1.0f,
                     int n_slices = 32) {
        return CreatePipe("", points, radius, n_slices);
    }
    Node* CreatePipe(const std::string& name,
                     const Array<FPoint3D>& points,
                     const Array<float>& radius,
                     int n_slices = 32) {
        CHECK(points.size() == radius.size());

        Pipe pipe(points, radius, n_slices);
        return CreateNode(name, "Pipe", pipe);
    }
    Node* CreatePipe(const Array<FPoint3D>& points,
                     const Array<float>& radius,
                     int n_slices = 32) {
        CHECK(points.size() == radius.size());

        return CreatePipe("", points, radius, n_slices);
    }

    /**
     * Create a unit sphere with n_slices (the value range is [2, 128]).
     */
    Node* CreateSphere(const std::string& name, int n_slices = 32) {
        Sphere sphere(n_slices);
        return CreateNode(name, "Sphere", sphere);
    }
    Node* CreateSphere(int n_slices = 32) {
        return CreateSphere("", n_slices);
    }

    /**
     * Create a unit cylinder with n_slices (the value range is [2, 128]).
     */
    Node* CreateCylinder(const std::string& name, int n_slices = 32) {
        Cylinder cylinder(n_slices);
        return CreateNode(name, "Cylinder", cylinder);
    }
    Node* CreateCylinder(int n_slices = 32) {
        return CreateCylinder("", n_slices);
    }

    /**
     * Create a mesh node from SurfaceMesh.
     */
    Node* CreateMesh(const std::string& name,
                     const geometry::SurfaceMesh<FPoint3D>& surface_mesh) {
        MeshData mesh;
        mesh.Load(surface_mesh);

        return CreateNode(name, "Mesh", mesh);
    }
    Node* CreateMesh(const geometry::SurfaceMesh<FPoint3D>& surface_mesh) {
        return CreateMesh("", surface_mesh);
    }

    /**
     * Create a mesh from local file.
     */
    Node* CreateMesh(const std::string& filename) {
        std::string suffix = file_util::GetSuffix(filename);

        geometry::SurfaceMesh<FPoint3D> surface_mesh;
        if (suffix == "obj") {
            geometry::OBJLoader obj;
            if (!obj.Open(filename)) return nullptr;
            if (!obj.Load(&surface_mesh)) return nullptr;
        } else {
            LOG(WRONG) << "Unsupport mesh format: " << filename;
            return nullptr;
        }

        if (surface_mesh.empty()) return nullptr;

        MeshData mesh;
        mesh.Load(surface_mesh);
        return CreateNode(file_util::GetBasename(filename), "Mesh", mesh);
    }

protected:
    /**
     * Create a mesh node.
     */
    template <typename Data>
    Node* CreateNode(const std::string& name,
                     const std::string& type,
                     const Data& data) {
        RenderObject* object = objects_.Allocate();
        object->SetRenderData(data);

        return CreateNode(name, type, object);
    }
    Node* CreateNode(const std::string& name,
                     const std::string& type,
                     RenderObject* object) {
        Node* node = nodes_.Allocate();
        node->set_name(GetID(name, type));
        node->AddRenderObject(object);
        return node;
    }

    /**
     * Automatic create names for different types.
     */
    std::string GetID(const std::string& name, const std::string& type) {
        return name.empty() ? type + std::to_string(counts_[type]++) : name;
    }

    // Allocated nodes.
    Pool<Node> nodes_;

    // Mesh objects.
    Pool<RenderObject> objects_;

    // Count for each object type.
    std::unordered_map<std::string, int> counts_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_PRIMITIVE_PRIMITIVE_FACTORY_H_
