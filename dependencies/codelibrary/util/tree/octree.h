//
// Copyright 2014-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_TREE_OCTREE_H_
#define CODELIBRARY_UTIL_TREE_OCTREE_H_

#include <limits>
#include <unordered_map>

#include "codelibrary/base/bits.h"
#include "codelibrary/base/pool.h"

namespace cl {

/**
 * Hash based fast sparse octree. It is much better than the traditional octree
 * in terms of memory utilization.
 */
template <typename T, typename Index = uint32_t>
class Octree {
public:
    static_assert(std::is_integral<Index>::value, "");

    // Octree Node.
    struct Node {
        friend class Octree;

        bool has_child(int i) const {
            return (child_mask_ & (1 << i)) != 0;
        }

        void set_data(const T& data) {
            data_ = data;
        }

        const T& data() const {
            return data_;
        }

        T& data() {
            return data_;
        }

        /**
         * Return the node location.
         */
        Index location() const {
            return location_;
        }

        /**
         * Return the depth of this node in octree.
         */
        int get_depth() const {
            return bits::CountBits(location_) / 3;
        }

        /**
         * Return the location of this node in the octree.
         */
        void get_position(int* x, int* y, int* z, int* d) const {
            CHECK(x && y && z && d);

            *d = this->get_depth();
            Index location = location_ & ((Index(1) << (3 * *d)) - 1);
            *x = MortonDecode(location);
            *y = MortonDecode(location >> 1);
            *z = MortonDecode(location >> 2);
        }

    protected:
        /**
         * 3D Morton decode.
         */
        static int MortonDecode(Index x) {
            x = x & 0x49249249;
            x = (x | (x >> 2)) & 0xc30c30c3;
            x = (x | (x >> 4)) & 0x0f00f00f;
            x = (x | (x >> 8)) & 0xff0000ff;
            x = (x | (x >> 16)) & 0x0000ffff;
            return x;
        }

        /**
         * Return the i-th index child location.
         */
        Index child_location(Index i) const {
            return (location_ << 3) | i;
        }

        T data_;             // The stored data.
        Index location_;     // Hash (location) code of this node.
        uint8_t child_mask_; // Used to check if the child is exist.
    };

    Octree() = default;

    /**
     * Construct an octree by given depth. The resolution of octree will be
     * (1 << depth)^3.
     */
    Octree(int depth)
        : depth_(depth) {
        CHECK(depth_ > 0);
        CHECK(depth_ <= (std::numeric_limits<Index>::digits - 1) / 3);

        root_ = Allocate(1);
        resolution_ = 1 << (depth_ - 1);
    }

    Octree(const Octree&) = delete;

    Octree& operator=(const Octree&) = delete;

    /**
     * Check if the given node is a leaf node.
     */
    bool is_leaf(const Node* node) const {
        return node->get_depth() == depth_ - 1;
    }

    /**
     * Return the i-th child of 'node'.
     */
    const Node* GetChild(const Node* node, int index) const {
        if (!node->has_child(index)) return nullptr;

        return Find(node->child_location(index));
    }
    Node* GetChild(Node* node, int index) {
        if (!node->has_child(index)) return nullptr;

        return Find(node->child_location(index));
    }

    /**
     * Get the parent node of this node.
     */
    const Node* GetParent(const Node* node) const {
        auto itr = nodes_.find(node->location() >> 3);
        return itr == nodes_.end() ? nullptr : itr->second;
    }
    Node* GetParent(const Node* node) {
        auto itr = nodes_.find(node->location() >> 3);
        return itr == nodes_.end() ? nullptr : itr->second;
    }

    /**
     * Insert a leaf node at (x, y, z) in octree and return the new node.
     */
    std::pair<Node*, bool> Insert(int x, int y, int z, const T& data) {
        CHECK(x >= 0 && x < resolution_);
        CHECK(y >= 0 && y < resolution_);
        CHECK(z >= 0 && z < resolution_);

        return Insert(x, y, z, data, resolution_ >> 1, root_);
    }

    /**
     * Return the location code (i.e., morton encode) of leaf node at (x, y, z).
     */
    Index GetLocationCode(int x, int y, int z) const {
        CHECK(x >= 0 && x < resolution_);
        CHECK(y >= 0 && y < resolution_);
        CHECK(z >= 0 && z < resolution_);

        return static_cast<Index>(MortonEncode(x, y, z)) |
               (Index(1) << 3 * (depth_ - 1));
    }

    /**
     * Find the leaf node at (x, y, z) in octree.
     */
    Node* Find(int x, int y, int z) {
        return Find(GetLocationCode(x, y, z));
    }

    /**
     * Find the leaf node at (x, y, z) in octree.
     */
    const Node* Find(int x, int y, int z) const {
        return Find(GetLocationCode(x, y, z));
    }

    /**
     * Return the node of the given location code.
     */
    Node* Find(Index index) {
        auto itr = nodes_.find(index);
        return itr == nodes_.end() ? nullptr : itr->second;
    }
    const Node* Find(Index index) const {
        auto itr = nodes_.find(index);
        return itr == nodes_.end() ? nullptr : itr->second;
    }

    /**
     * Erase a leaf node, return false if the node does not exist.
     */
    bool Erase(int x, int y, int z) {
        CHECK(x >= 0 && x < resolution_);
        CHECK(y >= 0 && y < resolution_);
        CHECK(z >= 0 && z < resolution_);

        return Erase(x, y, z, resolution_ >> 1, root_);
    }

    T& operator() (int x, int y, int z) {
        return Insert(x, y, z, T()).first->data_;
    }

    const T& operator() (int x, int y, int z) const {
        const Node* leaf = Find(x, y, z);

        return (leaf == nullptr) ? T() : leaf->data_;
    }

    /**
     * Reset depth for the octree.
     *
     * Note that, this method will clear the data.
     */
    void Reset(int depth) {
        CHECK(depth > 0);
        CHECK(depth <= (std::numeric_limits<Index>::digits - 1) / 3);

        clear();
        depth_ = depth;
        root_ = Allocate(1);
        resolution_ = 1 << (depth - 1);
    }

    /**
     * Return the number of octree nodes, including branch nodes.
     */
    int size() const {
        return node_pool_.n_available();
    }

    bool empty() const {
        return root_ == nullptr;
    }

    void clear() {
        root_ = nullptr;
        nodes_.clear();
        node_pool_.clear();
    }

    /**
     * Return the root node of the octree.
     */
    Node* root() const {
        return root_;
    }

    /**
     * Return the depth of octree.
     */
    int depth() const {
        return depth_;
    }

    /**
     * Return the resolution of the octree.
     */
    int resolution() const {
        return resolution_;
    }

    /**
     * Return hashed nodes stored in octree.
     */
    const std::unordered_map<Index, Node*> nodes() const {
        return nodes_;
    }

protected:
    /**
     * Get child node index using depth mask.
     */
    static int GetChildIndex(int x, int y, int z, int depth_mask) {
        return ((static_cast<int>((z & depth_mask) != 0) << 2) |
                (static_cast<int>((y & depth_mask) != 0) << 1) |
                 static_cast<int>((x & depth_mask) != 0));
    }

    /**
     * Method to seperate bits from a given integer 3 positions apart.
     */
    static uint64_t ExpandBits(int a) {
        uint64_t x = a    & 0x00000000001fffff;
        x = (x | x << 32) & 0x001f00000000ffff;
        x = (x | x << 16) & 0x001f0000ff0000ff;
        x = (x | x << 8)  & 0x100f00f00f00f00f;
        x = (x | x << 4)  & 0x10c30c30c30c30c3;
        x = (x | x << 2)  & 0x1249249249249249;
        return x;
    }

    /**
     * 3D Morton encode.
     */
    static uint64_t MortonEncode(int x, int y, int z) {
        uint64_t answer = 0;
        answer |= ExpandBits(x) | ExpandBits(y) << 1 | ExpandBits(z) << 2;
        return answer;
    }

    /**
     * Insert a leaf node.
     *
     * If the leaf node already exist, we do nothing and return a 'false' label.
     */
    std::pair<Node*, bool> Insert(int x, int y, int z, const T& data,
                                  int depth_mask, Node* node) {
        if (depth_mask == 0) {
            // Root node.
            return {node, false};
        }

        int index = GetChildIndex(x, y, z, depth_mask);
        Index location = node->child_location(index);
        if (node->has_child(index)) {
            Node* child = nodes_[location];
            if (depth_mask > 1) {
                return Insert(x, y, z, data, depth_mask >> 1, child);
            }

            // Reach leaf.
            return {child, false};
        } else {
            Node* child = Allocate(location);
            node->child_mask_ |= (1 << index);
            if (depth_mask > 1) {
                return Insert(x, y, z, data, depth_mask >> 1, child);
            }

            // Reach leaf.
            child->data_ = data;
            return {child, true};
        }
    }

    /**
     * Erase a leaf node, return false if the node does not exist.
     */
    bool Erase(int x, int y, int z, int depth_mask, Node* node) {
        if (depth_mask == 0) {
            // The root node, do nothing.
            return true;
        }

        int index = GetChildIndex(x, y, z, depth_mask);
        if (!node->has_child(index)) return false;

        Index location = node->child_location(index);
        Node* child = nodes_[location];

        if (depth_mask > 1) {
            return Erase(x, y, z, depth_mask >> 1, child);
        }

        // Reach leaf.
        node->child_mask_ &= ~(1 << index);
        nodes_.erase(child);
        node_pool_.Deallocate(child);
        return true;
    }

    /**
     * Allocate a new octree node at given location.
     */
    Node* Allocate(Index location) {
        Node* node = node_pool_.Allocate();
        node->location_ = location;
        node->child_mask_ = 0;
        node->data_ = T();
        nodes_[location] = node;
        return node;
    }

    // Depth of octree.
    int depth_ = 0;

    // Finest resolution of octree.
    int resolution_ = 0;

    // Octree root.
    Node* root_ = nullptr;

    // All nodes are stored in hash.
    std::unordered_map<Index, Node*> nodes_;

    // Allocator for octree nodes.
    Pool<Node> node_pool_;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_TREE_OCTREE_H_
