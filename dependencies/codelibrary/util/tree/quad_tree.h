//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_TREE_QUAD_TREE_H_
#define CODELIBRARY_UTIL_TREE_QUAD_TREE_H_

#include <algorithm>
#include <cstring>

#include "codelibrary/base/bits.h"
#include "codelibrary/base/pool.h"

namespace cl {

/**
 * Quad tree.
 * The class can be used as a 2D array. It has high space efficiency.
 *
 * Sample usage:
 *
 * Quadtree<int> q(4, 4);
 * q(1, 2) = 3;
 */
template <typename T>
class Quadtree {
public:
    // Enum of node type within the Quadtree.
    enum NodeType {
        BRANCH_NODE, LEAF_NODE
    };

    // Basic node of Quadtree for inherit.
    struct Node {
        Node() = default;

        virtual ~Node() = default;

        virtual NodeType get_type() const = 0;
    };

    // Branch Node for Quadtree.
    class BranchNode : Node {
        friend class Quadtree;

    public:
        BranchNode() {
            memset(child_nodes_, 0, sizeof(child_nodes_));
        }

        virtual NodeType get_type() const {
            return BRANCH_NODE;
        }

    protected:
        Node* child_nodes_[4]; // The pointers to the children.
    };

    // Leaf Node for Quadtree.
    class LeafNode : Node {
        friend class Quadtree;

    public:
        LeafNode() = default;

        virtual NodeType get_type() const {
            return LEAF_NODE;
        }

        const T& data() const  {
            return data_;
        }

        T& data() {
            return data_;
        }

        void set_data(const T& v) {
            data_ = v;
        }

    protected:
        T data_; // The data stored in this leaf node.
    };

    Quadtree() {
        Initialize();
    }

    Quadtree(int size1, int size2)
        : size1_(size1), size2_(size2) {
        CHECK(size1_ >= 0 && size2_ >= 0);

        Initialize();
    }

    Quadtree(const Quadtree&) = delete;

    Quadtree& operator=(const Quadtree&) = delete;

    /**
     * Clear the Quadtree.
     */
    void clear() {
        branch_pool_.clear();
        leaf_pool_.clear();

        root_ = branch_pool_.Allocate();
        size_ = 0;
        depth_ = 0;
        size1_ = 0;
        size2_ = 0;
    }

    /**
     * Return the size of elements.
     */
    int size() const {
        return size_;
    }

    /**
     * Return the size of the first dimension.
     */
    int size1() const {
        return size1_;
    }

    /**
     * Return the size of the second dimension.
     */
    int size2() const {
        return size2_;
    }

    /**
     * Return the root of Quadtree.
     */
    BranchNode* root() const {
        return root_;
    }

    /**
     * Return the depth of Quadtree.
     */
    int depth() const {
        return depth_;
    }

    /**
     * Check if the Quadtree is empty.
     */
    bool empty() const {
        return depth_ == 0;
    }

    /**
     * Resize the Quadtree.
     *
     * Note that, this method will clear the data.
     */
    void Resize(int size1, int size2) {
        CHECK(size1 >= 0 && size2 >= 0);

        clear();

        size1_ = size1;
        size2_ = size2;

        SetDepth();
    }

    /**
     * Find the leaf node at (x, y) in Quadtree.
     */
    LeafNode* Find(int x, int y) {
        CHECK(CheckValid(x, y));

        return Find(x, y, DepthMask(), root_);
    }

    /**
     * Find the leaf node at (x, y) in Quadtree.
     */
    const LeafNode* Find(int x, int y) const {
        CHECK(CheckValid(x, y));

        return Find(x, y, DepthMask(), root_);
    }

    /**
     * Insert a leaf node at (x, y) in Quadtree.
     * If that leaf node already exist, directly return it and set the bool to
     * false.
     */
    std::pair<LeafNode*, bool> Insert(int x, int y, const T& v) {
        CHECK(CheckValid(x, y));

        return Insert(x, y, DepthMask(), root_, v);
    }

    /**
     * Erase a leaf node, return false if the node does not exist.
     */
    bool Erase(int x, int y) {
        CHECK(CheckValid(x, y));

        return Erase(x, y, DepthMask(), root_);
    }

    T& operator() (int x, int y) {
        return Insert(x, y, T()).first->data_;
    }

    const T& operator() (int x, int y) const {
        const LeafNode* leaf = Find(x, y);

        return (leaf == nullptr) ? T() : leaf->data;
    }

protected:
    /**
     * Check if (x, y, z) is a valid Quadtree index.
     */
    bool CheckValid(int x, int y) const {
        return x >= 0 && x < size1_ && y >= 0 && y < size2_;
    }

    /**
     * Insert a leaf node and return it.
     * If the leaf already exist, directly return it and set the bool to false.
     */
    std::pair<LeafNode*, bool> Insert(int x, int y, int depth_mask,
                                      BranchNode* branch, const T& data) {
        int child = GetChildIndex(x, y, depth_mask);
        Node* node = branch->child_nodes_[child];

        if (node == nullptr) {
            if (depth_mask > 1) {
                // If required branch does not exist, create it.
                BranchNode* child_branch = branch_pool_.Allocate();
                branch->child_nodes_[child] =
                        reinterpret_cast<Node*>(child_branch);

                return Insert(x, y, depth_mask >> 1, child_branch, data);
            }

            // If leaf node at child_index does exist.
            LeafNode* leaf = leaf_pool_.Allocate();
            leaf->data_ = data;
            branch->child_nodes_[child] = reinterpret_cast<Node*>(leaf);
            ++size_;
            return std::make_pair(leaf, true);
        }

        // Node exists already.
        switch (node->get_type()) {
        case BRANCH_NODE:
            return Insert(x, y, depth_mask >> 1,
                          reinterpret_cast<BranchNode*>(node), data);
        case LEAF_NODE:
            // Found.
            return std::make_pair(reinterpret_cast<LeafNode*>(node), false);
        }

        // Code never go here.
        CHECK(false) << "Unreachable code.";
        return std::make_pair(nullptr, false);
    }

    /**
     * Find a leaf node and return it.
     * If the leaf already exist, directly return it.
     */
    LeafNode* Find(int x, int y, int depth_mask, BranchNode* branch) {
        int child = GetChildIndex(x, y, depth_mask);
        Node* node = branch->child_nodes_[child];
        if (node == nullptr) return nullptr;

        if (node->get_type() == BRANCH_NODE) {
            return Find(x, y, depth_mask >> 1,
                        reinterpret_cast<BranchNode*>(node));
        }

        return reinterpret_cast<LeafNode*>(node);
    }

    /**
     * Find a leaf node and return it.
     * If the leaf already exist, directly return it.
     */
    const LeafNode* Find(int x, int y, int depth_mask,
                         BranchNode* branch) const {
        int child = GetChildIndex(x, y, depth_mask);
        Node* node = branch->child_nodes_[child];
        if (node == nullptr) return nullptr;

        if (node->get_type() == BRANCH_NODE) {
            return Find(x, y, depth_mask >> 1,
                        reinterpret_cast<BranchNode*>(node));
        }

        return reinterpret_cast<LeafNode*>(node);
    }

    /**
     * Erase a leaf node, return false if the node does not exist.
     */
    bool Erase(int x, int y, int depth_mask, BranchNode* branch) {
        int child = GetChildIndex(x, y, depth_mask);
        Node* node = branch->child_nodes_[child];

        if (node == nullptr) {
            return false;
        }

        switch (node->get_type()) {
        case BRANCH_NODE:
        {
            BranchNode* b = reinterpret_cast<BranchNode*>(node);
            if (Erase(x, y, depth_mask >> 1, b)) {
                for (int i = 0; i < 8; ++i) {
                    if (b->child_nodes_[i])
                        return true;
                }
                branch_pool_.Free(b);
                branch->child_nodes_[child] = nullptr;
                return true;
            }

            return false;
        }
        case LEAF_NODE:
        {
            --size_;
            LeafNode* leaf = reinterpret_cast<LeafNode*>(node);
            leaf_pool_.Free(leaf);
            branch->child_nodes_[child] = nullptr;
            return true;
        }
        }

        // Code never go here.
        CHECK(false) << "Unreachable code.";
        return false;
    }

    /**
     * Initialize the Quadtree.
     */
    void Initialize() {
        root_ = branch_pool_.Allocate();

        SetDepth();
    }

    /**
     * Set the depth of Quadtree.
     */
    void SetDepth() {
        // Find maximum amount of keys, require greater than zero.
        int max_pixels = std::max(size1_, size2_);

        // Tree depth = amount of bits of max_pixels.
        depth_ = max_pixels == 0 ? 0 : bits::Log2Floor(max_pixels) + 1;
    }

    /**
     * Get child node index using depth mask.
     */
    static int GetChildIndex(int x, int y, int depth_mask) {
        return (static_cast<int>((x & depth_mask) != 0) << 1) |
               (static_cast<int>((y & depth_mask) != 0));
    }

    /**
     * Return the depth mask for fast searching the child.
     */
    int DepthMask() const {
        return 1 << (depth_ - 1);
    }

    int depth_ = 0;                // Quadtree depth.
    int size_  = 0;                // The number of leaves.
    int size1_ = 0;                // Size of the first dimension.
    int size2_ = 0;                // Size of the second dimension.
    BranchNode* root_ = nullptr;   // The root node of Quadtree.
    Pool<BranchNode> branch_pool_; // Memory pool for branch nodes.
    Pool<LeafNode> leaf_pool_;     // Memory pool for leaf nodes.
};

} // namespace cl

#endif // CODELIBRARY_UTIL_TREE_QUAD_TREE_H_
