//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_INSTANCE_NODE_H_
#define CODELIBRARY_WORLD_INSTANCE_NODE_H_

#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/node.h"

namespace cl {
namespace world {

/**
 * Instance node is a lot of models where each model contain the same set of
 * vertex data.
 */
class InstanceNode : public Node {
public:
    InstanceNode(const std::string& name = "")
        : Node(name) {
        glGenBuffers(1, &transform_buffer_);

        glBindBuffer(GL_ARRAY_BUFFER, transform_buffer_);

        // Set attribute pointers for transform matrix (4 times vec4).
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, 16 * sizeof(float),
                              (void*)0);
        glEnableVertexAttribArray(5);
        glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 16 * sizeof(float),
                              (void*)(1 * 4 * sizeof(float)));
        glEnableVertexAttribArray(6);
        glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, 16 * sizeof(float),
                              (void*)(2 * 4 * sizeof(float)));
        glEnableVertexAttribArray(7);
        glVertexAttribPointer(7, 4, GL_FLOAT, GL_FALSE, 16 * sizeof(float),
                              (void*)(3 * 4 * sizeof(float)));

        glVertexAttribDivisor(4, 1);
        glVertexAttribDivisor(5, 1);
        glVertexAttribDivisor(6, 1);
        glVertexAttribDivisor(7, 1);
    }

    virtual ~InstanceNode() {
        glDeleteBuffers(1, &transform_buffer_);
    }

    /**
     * Clear the instances.
     */
    void ClearInstances() {
        n_instances_ = 0;
        transforms_.clear();
        bounding_box_ = FBox3D();
        modified_ = true;
    }

    /**
     * Reset the instance model.
     */
    void Reset(const RenderData& instance) {
        CHECK(instance.type == GL_TRIANGLES);

        n_instances_ = 0;
        transforms_.clear();
        instance_.SetRenderData(instance);
        modified_ = true;
    }

    /**
     * Add an instance.
     */
    void AddInstance(const gl::Transform& transform) {
        CHECK(!instance_.empty());

        ++n_instances_;
        transforms_.insert(transform.begin(), transform.end());

        FBox3D box = instance_.bounding_box();
        FPoint3D p1(box.x_min(), box.y_min(), box.z_min());
        FPoint3D p2(box.x_max(), box.y_max(), box.z_max());
        p1 = transform * p1;
        p2 = transform * p2;
        Array<FPoint3D> points = {p1, p2};
        bounding_box_.Join(FBox3D(points.begin(), points.end()));
        modified_ = true;
    }

    virtual void Render() override {
        if (!is_visible_ || instance_.empty() || n_instances_ == 0) return;

        if (modified_) {
            modified_ = false;
            this->SetupInstance();
        }

        glBindVertexArray(instance_.vertex_array());
        glDrawElementsInstanced(GL_TRIANGLES,
                                instance_.render_data().indices.size(),
                                GL_UNSIGNED_INT,
                                0,
                                n_instances_);
    }

    virtual FBox3D GetBoundingBox() const override {
        return bounding_box_;
    }

    /**
     * Return the number of instances.
     */
    int n_instances() const {
        return n_instances_;
    }

    virtual bool is_instance() const override {
        return true;
    }

    virtual int n_vertices() const override {
        return instance_.n_vertices() * n_instances_;
    }

    /**
     * Count the number of triangles in the render list.
     */
    virtual int n_faces() const override {
        return instance_.n_faces() * n_instances_;
    }

private:
    /**
     * Call when instance changed.
     */
    void SetupInstance() {
        // Setup transforms buffer.
        glBindBuffer(GL_ARRAY_BUFFER, transform_buffer_);
        glBufferData(GL_ARRAY_BUFFER, transforms_.size() * sizeof(float),
                     transforms_.data(), GL_STATIC_DRAW);
        instance_.SetupGL();
    }

    // Instance and transforms modified or not.
    bool modified_ = true;

    // Number of instances.
    int n_instances_ = 0;

    // Instance model.
    RenderObject instance_;

    // Buffer for transforms.
    GLuint transform_buffer_ = 0;

    // Transform for each instance.
    Array<float> transforms_;

    // Bounding box of this node.
    FBox3D bounding_box_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_INSTANCE_NODE_H_
