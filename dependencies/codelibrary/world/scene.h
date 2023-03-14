//
// Copyright 2020-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_WORLD_SCENE_H_
#define CODELIBRARY_WORLD_SCENE_H_

#include "codelibrary/opengl/camera.h"
#include "codelibrary/world/frustum_culler.h"
#include "codelibrary/world/light/light_set.h"
#include "codelibrary/world/node.h"

namespace cl {
namespace world {

/**
 * 3D world scene.
 *
 * It does not hold any node data but just organize them.
 */
class Scene : public Node {
public:
    /**
     * Construct the scene.
     */
    Scene()
        : Node("Scene") {
        // We set parent of scene to itself, so that other nodes cannot use
        // 'Scene' as a child node.
        this->parent_ = this;
    }

    virtual ~Scene() {
        parent_ = nullptr;
    }

    /**
     * Pick a node by ray.
     *
     * It optional returns the intersection point in world coordinate.
     */
    Node* PickNode(const FRay3D& ray, FPoint3D* point = nullptr) const {
        Node* picked_node = nullptr;
        double dis = DBL_MAX;

        for (Node* node : nodes_) {
            FPoint3D p;
            Transform transform = node->global_transform_.Inverse();

            FRay3D r(transform(ray.origin()), transform(ray.direction()));
            if (node->Pick(r, &p)) {
                // Convert the point to the world coordinate.
                p = node->global_transform_(p);
                double d = Distance(ray.origin(), p);
                if (d < dis) {
                    if (point) *point = p;
                    dis = d;
                    picked_node = node;
                }
            }
        }

        return picked_node;
    }

    /**
     * Update nodes and lights. It should be called every frame.
     */
    void Update() {
        // Update global transform for each node.
        nodes_.clear();
        UpdateNodes(this, this->GetTransform());

        // Add all light nodes into light system.
        lights_.clear();
        for (auto node : nodes_) {
            lights_.Add(node);
        }
    }

    /**
     * Cull the current nodes.
     */
    void FrustumCull(const gl::Camera& camera) {
        FrustumCuller culler(camera);
        culler.Cull(&nodes_);
    }

    /**
     * Get checked nodes from current nodes.
     */
    void GetCheckedNodes(Array<Node*>* nodes) {
        CHECK(nodes);

        nodes->clear();
        for (Node* node : nodes_) {
            if (node->is_checked_) nodes->push_back(node);
        }
    }

    /**
     * Return current lights in the scene.
     */
    const LightSet& lights() const {
        return lights_;
    }

    /**
     * Trun on/off the shadow.
     */
    void show_shadow(bool flag) {
        show_shadow_ = flag;
    }

    bool show_shadow() const {
        return show_shadow_;
    }

    /**
     * Return all nodes in the scene tree.
     */
    const Array<Node*>& nodes() const {
        return nodes_;
    }

    virtual int n_vertices() const override {
        int n = 0;
        for (const Node* node : nodes_) {
            n += node->n_vertices();
        }
        return n;
    }

    virtual int n_faces() const override {
        int n = 0;
        for (const Node* node : nodes_) {
            n += node->n_faces();
        }
        return n;
    }

private:
    /**
     * Recursively update all nodes.
     */
    void UpdateNodes(Node* node, const gl::Transform& transform) {
        if (!node || !node->is_visible()) return;
        node->global_transform_ = transform;

        for (Node* child : node->children()) {
            nodes_.push_back(child);
            if (child->fixed())
                UpdateNodes(child, child->GetTransform());
            else
                UpdateNodes(child, transform * child->GetTransform());
        }
    }

    // Current visible nodes in the scene.
    Array<Node*> nodes_;

    // Extract all lights from nodes.
    LightSet lights_;

    // Show shadow or not.
    bool show_shadow_ = false;
};

} // namespace world
} // namespace cl

#endif // CODELIBRARY_WORLD_SCENE_H_
