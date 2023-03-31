//
// Copyright 2022-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_RENDER_OBJECT_H_
#define CODELIBRARY_WORLD_KERNEL_RENDER_OBJECT_H_

#include "codelibrary/base/array.h"
#include "codelibrary/world/kernel/render_data.h"

namespace cl {
namespace world {

/**
 * Used to draw RenderData.
 */
class RenderObject {
public:
    explicit RenderObject() {
        Initialize();
    }

    explicit RenderObject(const RenderData& data)
        : data_(data) {
        Initialize();
        bounding_box_ = FBox3D(data.vertices.begin(), data.vertices.end());
    }

    RenderObject(const RenderObject&) = delete;

    virtual ~RenderObject() {
        glDeleteVertexArrays(1, &vertex_array_);
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteBuffers(1, &normal_buffer_);
        glDeleteBuffers(1, &color_buffer_);
        glDeleteBuffers(1, &texture_coord_buffer_);
        glDeleteBuffers(1, &element_buffer_);
    }

    RenderObject& operator=(const RenderObject&) = delete;

    /**
     * Render the list.
     */
    virtual void Render() {
        SetupGL();

        glBindVertexArray(vertex_array_);
        glDrawElements(data_.type, data_.indices.size(), GL_UNSIGNED_INT, 0);
    }

    /**
     * Set render data.
     */
    void SetRenderData(const RenderData& data) {
        data_ = data;
        bounding_box_ = FBox3D(data_.vertices.begin(), data_.vertices.end());
        modified_ = true;
    }

    /**
     * Swap render data to reduce memory copy.
     */
    void SwapRenderData(RenderData* data) {
        data_.swap(data);
        bounding_box_ = FBox3D(data_.vertices.begin(), data_.vertices.end());
        modified_ = true;
    }

    /**
     * Setup render data to GL.
     */
    /**
     * Setup render data.
     */
    virtual void SetupGL() {
        if (!modified_) return;
        modified_ = false;

        if (data_.empty()) return;

        // Upload vertex / index buffers.
        glBindVertexArray(vertex_array_);

        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(FPoint3D) * data_.vertices.size(),
                     data_.vertices.data(), GL_STATIC_DRAW);

        // Color attribute.
        if (!data_.colors.empty()) {
            glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
            glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, 0);
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(RGB32Color) * data_.colors.size(),
                         data_.colors.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(1);
        }

        // Normal vector attribute.
        if (!data_.normals.empty()) {
            glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(FVector3D) * data_.normals.size(),
                         data_.normals.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(2);
        }

        // Texture coordinate attribute.
        if (!data_.texture_coords.empty()) {
            glBindBuffer(GL_ARRAY_BUFFER, texture_coord_buffer_);
            glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, 0, 0);
            glBufferData(GL_ARRAY_BUFFER,
                         sizeof(FPoint2D) * data_.texture_coords.size(),
                         data_.texture_coords.data(),
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(3);
        }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     sizeof(GLuint) * data_.indices.size(),
                     data_.indices.data(),
                     GL_STATIC_DRAW);
    }

    /**
     * Append a render data to this object.
     */
    void Add(const RenderData& data) {
        CHECK(data_.empty() || data_.type == data.type)
            << "The appended data must be the same type as the existing data.";

        if (data.empty()) return;

        modified_ = true;
        bounding_box_.Join(FBox3D(data.vertices.begin(), data.vertices.end()));

        if (data_.empty()) {
            data_ = data;
            return;
        }

        // Check if the input data is valid.
        CHECK(data.colors.empty() ||
              data.colors.size() == data.vertices.size());
        CHECK(data.normals.empty() ||
              data.normals.size() == data.vertices.size());
        CHECK(data.texture_coords.empty() ||
              data.texture_coords.size() == data.vertices.size());

        if (data_.type == GL_TRIANGLES) {
            CHECK(data.indices.size() % 3 == 0);
        } else if (data_.type == GL_LINES) {
            CHECK(data.indices.size() % 2 == 0);
        }

        // Merge data.
        int n1 = data_.vertices.size();
        int n2 = data.vertices.size();
        data_.vertices.insert(data.vertices);
        for (int t : data.indices) {
            data_.indices.push_back(n1 + t);
        }

        Merge(data_.colors, n1, data.colors, n2, default_color_, &data_.colors);
        Merge(data_.normals, n1, data.normals, n2, default_normal_,
              &data_.normals);
        Merge(data_.texture_coords, n1, data.texture_coords, n2,
              FPoint2D(0.0f, 0.0f), &data_.texture_coords);
    }

    /**
     * Set a uniform color for all data vertices.
     */
    void SetUniformColor(const RGB32Color& c) {
        data_.colors.assign(data_.vertices.size(), c);
        this->modified_ = true;
    }

    void clear() {
        data_.clear();
        bounding_box_ = FBox3D();
        modified_ = true;
    }

    bool empty() const {
        return data_.empty();
    }

    /**
     * Return the number of vertices of this object.
     */
    int n_vertices() const {
        return data_.vertices.size();
    }

    /**
     * Return the number of faces of this object.
     */
    int n_faces() const {
        return (data_.type == GL_TRIANGLES) ? data_.indices.size() / 3 : 0;
    }

    const RenderData& render_data() const {
        return data_;
    }

    const FBox3D& bounding_box() const {
        return bounding_box_;
    }

    GLuint vertex_array() const {
        return vertex_array_;
    }

    void set_default_color(const RGB32Color& color) {
        default_color_ = color;
    }

    int type() const {
        return data_.type;
    }

protected:
    /**
     * This function will only be called once.
     */
    void Initialize() {
        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &color_buffer_);
        glGenBuffers(1, &normal_buffer_);
        glGenBuffers(1, &texture_coord_buffer_);
        glGenBuffers(1, &element_buffer_);

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, element_buffer_);

        // Position attribute.
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    }

    /**
     * Merge two lists with the default value.
     */
    template <typename T>
    static void Merge(const Array<T>& data1, int n1,
                      const Array<T>& data2, int n2,
                      const T& default_value,
                      Array<T>* data) {
        if (data1.empty() && !data2.empty()) {
            data->assign(n1, default_value);
            data->insert(data2);
        } else if (!data1.empty() && data2.empty()) {
            data->resize(n1 + n2, default_value);
        } else {
            data->insert(data2);
        }
    }

    // Vertex array object and rendering buffers.
    GLuint vertex_array_         = 0;
    GLuint vertex_buffer_        = 0;
    GLuint color_buffer_         = 0;
    GLuint normal_buffer_        = 0;
    GLuint texture_coord_buffer_ = 0;
    GLuint element_buffer_       = 0;

    // The render data of this object is modified or not.
    bool modified_ = true;

    // Default color if color is not set.
    RGB32Color default_color_ = RGB32Color::Black();

    // Defualt normal vector if normal vector is not set.
    FVector3D default_normal_ = {0.0f, 0.0f, 0.0f};

    // Bounding box of this object.
    FBox3D bounding_box_;

    // Render data.
    RenderData data_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_RENDER_OBJECT_H_
