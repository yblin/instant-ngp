//
// Copyright 2021-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin.xmu@qq.com (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_LIST_CIRCULAR_LIST_VIEW_H_
#define CODELIBRARY_UTIL_LIST_CIRCULAR_LIST_VIEW_H_

#include <iterator>

#include "codelibrary/base/log.h"

namespace cl {

/**
 * CircularListView is a view of circular list. It does not hold any object.
 *
 * The template parameter 'Node' must have member 'next()'.
 *
 * Sample usage:
 *
 *   // 'p' is A pointer to the node in a circular list.
 *   for (auto node : CircularListView(p)) {
 *       // Do something...
 *   }
 */
template <class Node>
class CircularListView {
public:
    /**
     * Iterator for CircularListView.
     */
    class Iterator {
    public:
        explicit Iterator(Node* node)
            : head_(node), node_(node) {}

        bool operator == (const Iterator& rhs) const {
            return head_ == rhs.head_ && node_ == rhs.node_;
        }

        bool operator != (const Iterator& rhs) const {
            return !(*this == rhs);
        }

        Node* operator*()  const { return node_; }
        Node* operator->() const { return node_; }

        Iterator& operator++() {
            CHECK(node_);
            node_ = node_->next();
            CHECK(node_);

            if (node_ == head_) {
                head_ = nullptr;
                node_ = nullptr;
            }
            return *this;
        }

    protected:
        Node* head_ = nullptr;
        Node* node_ = nullptr;
    };

    explicit CircularListView(Node* head)
        : head_(head) {}

    Iterator begin() const {
        return Iterator(head_);
    }

    Iterator end() const {
        return Iterator(nullptr);
    }

protected:
    Node* head_ = nullptr;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_LIST_CIRCULAR_LIST_VIEW_H_
