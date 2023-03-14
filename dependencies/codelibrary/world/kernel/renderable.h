//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_
#define CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_

#include "codelibrary/opengl/camera.h"
#include "codelibrary/world/kernel/render_object.h"

namespace cl {
namespace world {

/**
 * Class to implement rendering.
 *
 * It defines the basic rendering framework for 'world::Node'.
 */
struct Renderable {
public:
    using RenderList = Array<RenderObject*>;

    Renderable() = default;

    /**
     * Construction with a specific render object.
     */
    explicit Renderable(RenderObject* object) {
        CHECK(object);

        render_list_.push_back(object);
    }

    Renderable(const Renderable&) = delete;

    Renderable& operator=(const Renderable&) = delete;

    /**
     * Render this node.
     */
    virtual void Render() {
        if (is_visible_) {
            for (RenderObject* o : render_list_) {
                o->Render();
            }
        }
    }

    /**
     * Render parts that don't need lighting.
     */
    virtual void RenderWithoutLight(const gl::Camera&) {}

    /**
     * Clear the render list.
     */
    void ClearRenderList() {
        render_list_.clear();
    }

    /**
     * Add a render object to render list.
     */
    void AddRenderObject(RenderObject* render_object) {
        render_list_.push_back(render_object);
    }

    virtual int n_vertices() const {
        int n_vertices = 0;
        for (RenderObject* o : render_list_) {
            n_vertices += o->n_vertices();
        }
        return n_vertices;
    }

    virtual int n_faces() const {
        int n_faces = 0;
        for (RenderObject* o : render_list_) {
            n_faces += o->n_faces();
        }
        return n_faces;
    }

    // O(1) functions to access the members.
    void show(bool is_visible = true)     { is_visible_ = is_visible; }
    void hide()                           { is_visible_ = false;      }
    void set_visible(bool flag)           { is_visible_ = flag;       }
    bool is_visible()               const { return is_visible_;       }
    const RenderList& render_list() const { return render_list_;      }

protected:
    // Node is visible or not.
    bool is_visible_ = true;

    // A render list consists of render objects. Note that it does not own
    // these objects.
    RenderList render_list_;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_KERNEL_RENDERABLE_H_
