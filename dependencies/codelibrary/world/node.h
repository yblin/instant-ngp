 //
// Copyright 2020-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_NODE_H_
#define CODELIBRARY_WORLD_NODE_H_

#include <map>
#include <unordered_set>

#include "codelibrary/world/kernel/collidable.h"
#include "codelibrary/world/kernel/render_object.h"
#include "codelibrary/world/kernel/renderable.h"
#include "codelibrary/world/kernel/transformable.h"
#include "codelibrary/world/material/material.h"

namespace cl {
namespace world {

/**
 * World node.
 */
class Node : public world::Transformable,
             public world::Renderable,
             public world::Collidable {
    using Children = std::unordered_set<Node*>;
    using Transform = gl::Transform;

    friend class Scene;

public:
    Node() = default;

    explicit Node(const std::string& name)
        : name_(name) {}

    Node(const Node& node) = delete;

    Node(const std::string& name, RenderObject* object)
        : Renderable(object), name_(name) {}

    explicit Node(RenderObject* object)
        : Renderable(object) {}

    virtual ~Node() {
        if (parent_) {
            size_t n_erased = parent_->children_.erase(this);
            CHECK(n_erased == 1);
            parent_ = nullptr;
        }
        for (auto c : children_) {
            c->parent_ = nullptr;
        }
    }

    Node& operator=(const Node&) = delete;

    /**
     * Add an existing node as the child of this node.
     *
     * Require that the adding node is an isolate node.
     */
    void AddNode(Node* node) {
        CHECK(!node->parent_) << "Cannot be added a node repeatedly.";
        children_.insert(node);
        node->parent_ = this;
    }

    /**
     * Get the child node according to the given name.
     *
     * If there are several nodes have the same name, return the first one in
     * unordered_set.
     *
     * Note that this code has a time cost of O(N). Should be override in
     * special cases.
     */
    virtual Node* GetNode(const std::string& name) const {
        for (Node* node : children_) {
            if (node->name() == name) return node;
        }
        return nullptr;
    }

    /**
     * Erase a node.
     *
     * Require that the erasing node is a child of this node. Note that it dose
     * not destroy the node.
     */
    void EraseNode(Node* node) {
        CHECK(node->parent_ == this);

        children_.erase(node);
        node->parent_ = nullptr;
    }

    /**
     * Get the bounding box of this node.
     */
    virtual FBox3D GetBoundingBox() const {
        FBox3D box;
        for (RenderObject* o : render_list_) {
            box.Join(o->bounding_box());
        }
        return box;
    }

    /**
     * Check if this node is an instance node or not.
     */
    virtual bool is_instance() const {
        return false;
    }

    /**
     * Returns true if this node is capable for camera frustum culling.
     */
    virtual bool is_cullable() const {
        return true;
    }

    /**
     * Return true if 'node' is the child of this node.
     */
    bool is_child(Node* node) const {
        return children_.find(node) != children_.end();
    }

    // O(1) functions to access the members.
    const std::string& name()              const { return name_;             }
    Node* parent()                               { return parent_;           }
    const Node* parent()                   const { return parent_;           }
    const Children& children()             const { return children_;         }
    void set_name(const std::string& name)       { name_ = name;             }
    const Transform& global_transform()    const { return global_transform_; }
    bool fixed()                           const { return fixed_;            }
    void set_fixed(bool flag)                    { fixed_ = flag;            }
    void set_cast_shadow(bool flag)              { cast_shadow_ = flag;      }
    void set_receive_shadow(bool flag)           { receive_shadow_ = flag;   }
    bool cast_shadow()                     const { return cast_shadow_;      }
    bool receive_shadow()                  const { return receive_shadow_;   }
    bool is_checked()                      const { return is_checked_;       }
    void set_checked(bool flag)                  { is_checked_ = flag;       }
    Material* material()                         { return material_;         }
    const Material* material()             const { return material_;         }
    void set_material(Material* material)        { material_ = material;     }

protected:
    // The name of the node.
    std::string name_;

    // Parent node.
    Node* parent_ = nullptr;

    // Children nodes.
    Children children_;

    // Transform from the local coordinate to global coordinate.
    // It will be automatically set by Scene class.
    Transform global_transform_;

    // When a node is fixed it can not be moved.
    bool fixed_ = false;

    // Cast shadow or not.
    bool cast_shadow_ = true;

    // Receive shadow or not.
    bool receive_shadow_ = true;

    // Check if the node is checked or not.
    bool is_checked_ = false;

    // Material of this node.
    Material* material_ = nullptr;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_NODE_H_
