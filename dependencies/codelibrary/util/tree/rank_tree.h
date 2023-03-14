//
// Copyright 2016 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_TREE_RANK_TREE_H_
#define CODELIBRARY_UTIL_TREE_RANK_TREE_H_

#include <algorithm>
#include <functional>

#include "codelibrary/base/pool.h"

namespace cl {

/**
 * Rank tree is a red-black tree which dynamic organizes the rank of elements.
 *
 * Note that if two or more elements have the same key, all of them will be
 * stored.
 *
 * The typical usage is given below:
 *
 *  RankTree<int> rt;
 *  rt.Insert(3);      // 3 -> rank 0
 *  rt.Insert(1);      // 1 -> rank 0, 3 -> rank 1
 *  rt[1];             // 3 (Find the element of rank 1)
 *  rt[0];             // 1 (Find the element of rank 0)
 *  rt.Erase(1);       // 3 -> rank 0
 *  rt[0];             // 3 (Find the element of rank 0)
 *  rt.Insert(4);      // 3 -> rank 0, 4 -> rank 1
 *  rt.Insert(4);      // 3 -> rank 0, 4 -> rank 1, 4 -> rank 2
 *  rt.LowerRank(4);   // 1 (The number of elements less than 4)
 *  rt.UpperRank(4);   // 3 (The number of elements no greater than 4)
 */
template <class KeyType, class Less = std::less<KeyType>>
class RankTree {
    // Color enum for red-black.
    enum Color { BLACK = 0x00, RED = 0x01 };

    // Node for RankTree.
    struct Node {
        KeyType key;       // The key of node.
        Color color;       // The color of node.
        Node* left_child;  // The left child's link of this node.
        Node* right_child; // The right child's link of this node.
        Node* parent;      // The parent's link of this node.
        int left_count;    // The number of left sub-tree's nodes.
        int right_count;   // The number of right sub-tree's nodes.
    };

    /**
     * Return the in-order successor node in the rank tree.
     */
    template <typename NodePointer>
    static NodePointer Successor(NodePointer node) {
        if (node->right_child == nullptr) {
            while (node->parent && node == node->parent->right_child) {
                node = node->parent;
            }
            return node->parent;
        }

        node = node->right_child;
        while (node->left_child) {
            node = node->left_child;
        }
        return node;
    }

    /**
     * Return the in-order predecessor node in the rank tree.
     */
    template <typename NodePointer>
    static NodePointer Predecessor(NodePointer node) {
        if (node->left_child == nullptr) {
            while (node->parent && node == node->parent->left_child) {
                node = node->parent;
            }
            return node->parent;
        }

        node = node->left_child;
        while (node->right_child) {
            node = node->right_child;
        }
        return node;
    }

public:
    // Iterator for RankTree.
    class Iterator {
        friend class ConstIterator;
        friend class RankTree;

    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = Node*;
        using difference_type   = int;
        using pointer           = const value_type*;
        using reference         = const value_type&;

        explicit Iterator(Node* node = nullptr)
            : node_(node) {}

        bool operator == (const Iterator& rhs) const {
            return node_ == rhs.node_;
        }

        bool operator != (const Iterator& rhs) const {
            return !(*this == rhs);
        }

        const KeyType& operator*()  const { return node_->key; }
        const KeyType& operator->() const { return node_->key; }

        Iterator& operator ++() {
            node_ = Successor(node_);
            return *this;
        }

        Iterator& operator --() {
            node_ = Predecessor(node_);
            return *this;
        }

    private:
        Node* node_ = nullptr;
    };

    // ConstIterator for RankTree.
    class ConstIterator {
    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type        = const Node*;
        using difference_type   = int;
        using pointer           = const value_type*;
        using reference         = const value_type&;

        explicit ConstIterator(const Iterator& i)
            : node_(i.node_) {}

        explicit ConstIterator(const Node* node = nullptr)
            : node_(node) {}

        bool operator == (const ConstIterator& rhs) const {
            return node_ == rhs.node_;
        }

        bool operator != (const ConstIterator& rhs) const {
            return !(*this == rhs);
        }

        const KeyType& operator*()  const { return node_->key; }
        const KeyType& operator->() const { return node_->key; }

        ConstIterator& operator ++() {
            node_ = Successor(node_);
            return *this;
        }

        ConstIterator& operator --() {
            node_ = Predecessor(node_);
            return *this;
        }

    private:
        const Node* node_;
    };

    RankTree() = default;

    RankTree(const RankTree&) = delete;

    RankTree& operator=(const RankTree&) = delete;

    /**
     * Clear the RankTree.
     */
    void clear() {
        size_ = 0;
        root_ = nullptr;
        pool_.clear();
    }

    /**
     * Check if the RankTree is empty.
     */
    bool empty() const {
        return size_ == 0;
    }

    /**
     * Return the number of keys in the RankTree.
     */
    int size() const {
        return size_;
    }

    Iterator begin() {
        if (!root_) return Iterator(nullptr);

        Node* node = root_;
        while (node->left_child) {
            node = node->left_child;
        }
        return Iterator(node);
    }

    ConstIterator begin() const {
        if (!root_) return ConstIterator(nullptr);

        const Node* node = root_;
        while (node->left_child) {
            node = node->left_child;
        }
        return ConstIterator(node);
    }

    Iterator end() {
        return Iterator(nullptr);
    }

    ConstIterator end() const {
        return ConstIterator(nullptr);
    }

    /**
     * Insert a key into RankTree, return the rank of the inserted key.
     */
    Iterator Insert(const KeyType& key) {
        Node* current = root_;
        Node* parent = nullptr;

        int rank = 0;
        while (current) {
            parent = current;
            if (!less_(current->key, key)) {
                ++current->left_count;
                current = current->left_child;
            } else {
                ++current->right_count;
                rank += current->left_count + 1;
                current = current->right_child;
            }
        }

        // Setup new node.
        Node* node = pool_.Allocate();
        node->color = RED;
        node->key = key;
        node->parent = parent;
        node->left_child = node->right_child = nullptr;
        node->left_count = node->right_count = 0;

        // Insert node into tree.
        if (parent) {
            if (!less_(parent->key, key)) {
                parent->left_child = node;
            } else {
                parent->right_child = node;
            }
        } else {
            root_ = node;
        }

        ++size_;
        InsertFixup(node);

        return Iterator(node);
    }

    /**
     * Return the lower bound rank of the given key, i.e., the number of the
     * elements less than (<) 'key'.
     */
    int LowerRank(const KeyType& key) const {
        const Node* p = root_;
        int rank = 0;
        while (p) {
            if (!less_(p->key, key)) {
                p = p->left_child;
            } else {
                rank += p->left_count + 1;
                p = p->right_child;
            }
        }
        return rank;
    }

    /**
     * Return the upper bound rank of the given key, i.e., the number of the 
     * elements no greater than (<=) 'key'.
     */
    int UpperRank(const KeyType& key) const {
        const Node* p = root_;
        int rank = 0;
        while (p) {
            if (less_(key, p->key)) {
                p = p->left_child;
            } else {
                rank += p->left_count + 1;
                p = p->right_child;
            }
        }
        return rank;
    }

    /**
     * Erase the element with the given key.
     */
    void Erase(const KeyType& key) {
        Erase(Find(key));
    }

    /**
     * Erase the element with the given iterator.
     */
    void Erase(const Iterator& iter) {
        Node* node = iter.node_;
        if (!node) return;

        // Update node count.
        Node* tmp = node;
        while (tmp->parent) {
            if (tmp->parent->left_child == tmp) {
                --tmp->parent->left_count;
            } else {
                --tmp->parent->right_count;
            }
            tmp = tmp->parent;
        }

        Node* y = node;
        Node* successor = nullptr;
        if (node->left_child == nullptr) {
            successor = y->right_child;
        } else if (node->right_child == nullptr) {
            successor = y->left_child;
        } else {
            y = node->right_child;
            --node->right_count;
            while (y->left_child) {
                --y->left_count;
                y = y->left_child;
            }
            successor = y->right_child;
        }

        Node* successor_parent = nullptr;
        if (y != node) {
            node->left_child->parent = y;
            y->left_child = node->left_child;
            y->left_count = node->left_count;

            if (y != node->right_child) {
                successor_parent = y->parent;
                if (successor) successor->parent = y->parent;
                y->parent->left_child = successor;
                y->right_child = node->right_child;
                y->right_count = node->right_count;
                node->right_child->parent = y;
            } else {
                successor_parent = y;
            }

            if (root_ == node) {
                root_ = y;
            } else if (node->parent->left_child == node) {
                node->parent->left_child = y;
            } else {
                node->parent->right_child = y;
            }
            y->parent = node->parent;
            std::swap(y->color, node->color);
            y = node;
        } else {
            successor_parent = y->parent;
            if (successor) successor->parent = y->parent;

            if (root_ == node) {
                root_ = successor;
            } else if (node->parent->left_child == node) {
                node->parent->left_child = successor;
            } else {
                node->parent->right_child = successor;
            }
        }

        if (y->color == BLACK) {
            EraseFixup(successor, successor_parent);
        }

        --size_;
        pool_.Deallocate(y);
    }

    /**
     * Return the element which has a rank k (i.e. the k-th smallest element).
     */
    const KeyType& operator[](int k) const {
        CHECK(k >= 0 && k < size_);

        Node* p = root_;
        while (p) {
            if (p->left_count < k) {
                k -= p->left_count + 1;
                p = p->right_child;
            } else if (p->left_count > k) {
                p = p->left_child;
            } else {
                return p->key;
            }
        }

        CHECK(false) << "Unreachable code!";
        return root_->key;
    }

    /**
     * Return the left-most const_iterator to the given key.
     */
    ConstIterator Find(const KeyType& key) const {
        return ConstIterator(FindNode(key));
    }

    /**
     * Return the left-most iterator to the given key.
     */
    Iterator Find(const KeyType& key) {
        return Iterator(FindNode(key));
    }

private:
    /**
     * Return the left-most node whose key is equal to the given 'key'.
     */
    Node* FindNode(const KeyType& key) const {
        Node* p = root_;
        Node* lower_bound = nullptr;
        while (p) {
            if (!less_(p->key, key)) {
                lower_bound = p;
                p = p->left_child;
            } else {
                p = p->right_child;
            }
        }

        return (lower_bound == nullptr || less_(key, lower_bound->key)) ?
               nullptr : lower_bound;
    }

    /**
     * Fixup the insertion by recoloring nodes and performing rotations.
     */
    void InsertFixup(Node* node) {
        while (node->parent && node->parent->color == RED) {
            Node* parent = node->parent;
            Node* grandparent = parent->parent;
            if (parent == grandparent->left_child) {
                Node* uncle = grandparent->right_child;

                if (uncle && uncle->color == RED) {
                    parent->color = BLACK;
                    uncle->color = BLACK;
                    grandparent->color = RED;
                    node = grandparent;
                } else {
                    if (node == parent->right_child) {
                        RotateLeft(parent);
                        node = parent;
                        parent = node->parent;
                    }
                    parent->color = BLACK;
                    parent->parent->color = RED;
                    RotateRight(parent->parent);
                }
            } else {
                Node* uncle = grandparent->left_child;

                if (uncle && uncle->color == RED) {
                    parent->color = BLACK;
                    uncle->color = BLACK;
                    grandparent->color = RED;
                    node = grandparent;
                } else {
                    if (node == parent->left_child) {
                        RotateRight(parent);
                        node = parent;
                        parent = node->parent;
                    }
                    parent->color = BLACK;
                    parent->parent->color = RED;
                    RotateLeft(parent->parent);
                }
            }
        }
        root_->color = BLACK;
    }

    /**
     * Fixup the deletion by recoloring nodes and performing rotations.
     */
    void EraseFixup(Node* node, Node* parent) {
        while ((node == nullptr || node->color == BLACK) && node != root_) {
            if (node == parent->left_child) {
                Node* sibling = parent->right_child;

                if (sibling->color == RED) {
                    sibling->color = BLACK;
                    parent->color = RED;
                    RotateLeft(parent);
                    sibling = parent->right_child;
                }

                if ((sibling->left_child == nullptr ||
                     sibling->left_child->color == BLACK) &&
                    (sibling->right_child == nullptr ||
                     sibling->right_child->color == BLACK)) {
                    sibling->color = RED;
                    node = parent;
                    parent = parent->parent;
                    continue;
                }

                if (sibling->right_child == nullptr ||
                    sibling->right_child->color == BLACK) {
                    if (sibling->left_child)
                        sibling->left_child->color = BLACK;
                    sibling->color = RED;
                    RotateRight(sibling);
                    sibling = parent->right_child;
                }

                sibling->color = parent->color;
                parent->color = BLACK;
                if (sibling->right_child)
                    sibling->right_child->color = BLACK;
                RotateLeft(parent);
                node = root_;
                break;
            }

            Node* sibling = parent->left_child;

            if (sibling->color == RED) {
                sibling->color = BLACK;
                parent->color = RED;
                RotateRight(parent);
                sibling = parent->left_child;
            }

            if ((!sibling->left_child ||
                 sibling->left_child->color == BLACK) &&
                (!sibling->right_child ||
                 sibling->right_child->color == BLACK)) {
                sibling->color = RED;
                node = parent;
                parent = parent->parent;
                continue;
            }

            if (!sibling->left_child ||
                sibling->left_child->color == BLACK) {
                if (sibling->right_child)
                    sibling->right_child->color = BLACK;
                sibling->color = RED;
                RotateLeft(sibling);
                sibling = parent->left_child;
            }

            sibling->color = parent->color;
            parent->color = BLACK;
            if (sibling->left_child)
                sibling->left_child->color = BLACK;
            RotateRight(parent);
            node = root_;
            break;
        }
        if (node) node->color = BLACK;
    }

    /**
     * Left Rotate operation.
     */
    void RotateLeft(Node* node) {
        Node* y = node->right_child;  // Can't be nullptr.
        node->right_child = y->left_child;
        node->right_count = y->left_count;

        if (y->left_child) {
            y->left_child->parent = node;
        }
        y->parent = node->parent;

        if (node == root_) {
            root_ = y;
        } else if (node == node->parent->left_child) {
            node->parent->left_child = y;
        } else {
            node->parent->right_child = y;
        }

        y->left_child = node;
        y->left_count += 1 + node->left_count;
        node->parent = y;
    }

    /**
     * Right Rotate operation.
     */
    void RotateRight(Node* node) {
        Node* y = node->left_child;  // Can't be nullptr.
        node->left_child = y->right_child;
        node->left_count = y->right_count;

        if (y->right_child) {
            y->right_child->parent = node;
        }
        y->parent = node->parent;

        if (node == root_) {
            root_ = y;
        } else if (node == node->parent->right_child) {
            node->parent->right_child = y;
        } else {
            node->parent->left_child = y;
        }

        y->right_child = node;
        y->right_count += 1 + node->right_count;
        node->parent = y;
    }

    int size_ = 0;          // The number of keys stored in RankTree.
    Node* root_ = nullptr;  // Root of RankTree.
    Less less_;             // Less comparison.
    Pool<Node> pool_;       // Pool for RankTree node.
};

} // namespace cl

#endif // CODELIBRARY_UTIL_TREE_RANK_TREE_H_
