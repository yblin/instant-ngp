//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_INTERVAL_INTERVAL_SET_H_
#define CODELIBRARY_UTIL_INTERVAL_INTERVAL_SET_H_

#include <set>

#include "codelibrary/util/interval/interval.h"

namespace cl {

/**
 * An interval set is used to store the intervals, and perform the interval
 * operations, like: union, difference and intersection.
 */
template <typename T>
class IntervalSet {
    using BoundType = typename Interval<T>::BoundType;
    using It = Interval<T>;

public:
    using Iterator = typename std::set<It>::iterator;
    using ConstIterator = typename std::set<It>::const_iterator;
    using ReverseIterator = typename std::set<It>::reverse_iterator;
    using ConstReverseIterator = typename std::set<It>::const_reverse_iterator;

    /**
     * Construct an empty interval set.
     */
    IntervalSet() = default;

    /**
     * Insert an interval into interval set.
     *
     * Example:
     *   [1 3] + [2 4) -> [1 4).
     *   [1 3] + [4 5] -> [1 3] U [4 5].
     *   [1 3] U [4 6] + [2 5] -> [1 6].
     */
    void Insert(const It& interval) {
        CHECK(!interval.empty());

        It union_interval = interval;

        ConstIterator left_it = Lower(interval);
        ConstIterator right_it = Upper(interval);

        if (left_it != end()) {
            union_interval.Join(*left_it);
        }
        if (right_it != end()) {
            union_interval.Join(*right_it);
            ++right_it;
        }

        set_.erase(left_it, right_it);
        set_.insert(union_interval);
    }

    /**
     * Erase an interval from interval set.
     *
     * Example:
     *   [1 3) U [4 5) - [2 4] -> [1 2) U (4 5).
     */
    void Erase(const It& interval) {
        CHECK(!interval.empty());

        It new_interval1, new_interval2;
        ConstIterator left_it = Lower(interval);
        if (left_it != end()) {
            if (interval.Touch(*left_it)) ++left_it;
            if (left_it != end()) {
                const T& l = left_it->lower_bound();
                const T& r = interval.lower_bound();
                BoundType t1 = left_it->lower_bound_type();
                BoundType t2 = interval.lower_bound_type() == It::OPEN ?
                               It::CLOSED : It::OPEN;
                if (It::IsValid(l, r, t1, t2)) {
                    new_interval1 = It(l, r, t1, t2);
                }
            }
        }

        ConstIterator right_it = Upper(interval);
        if (right_it != end()) {
            ConstIterator i = right_it;
            if (interval.Touch(*right_it) &&
                right_it->lower_bound() == interval.upper_bound()) {
                --i;
            } else {
                ++right_it;
            }
            if (i != end()) {
                const T& l = interval.upper_bound();
                const T& r = i->upper_bound();
                BoundType t1 = interval.upper_bound_type() == It::OPEN ?
                               It::CLOSED : It::OPEN;
                BoundType t2 = i->upper_bound_type();
                if (It::IsValid(l, r, t1, t2)) {
                    new_interval2 = It(l, r, t1, t2);
                }
            }
        }

        if (left_it != end()) set_.erase(left_it, right_it);
        if (!new_interval1.empty()) set_.insert(new_interval1);
        if (!new_interval2.empty()) set_.insert(new_interval2);
    }

    /**
     * Return the iterator to the left-most interval that touches or overlap
     * with the given interval.
     */
    ConstIterator Lower(const It& interval) const {
        CHECK(!interval.empty());

        if (set_.empty()) return end();

        ConstIterator lower_it = set_.lower_bound(interval);
        if (lower_it == set_.end() || --lower_it == set_.end()) {
            for (ConstIterator i = begin(); i != end(); ++i) {
                if (i->lower_bound() > interval.upper_bound()) break;
                if (interval.Overlap(*i) ||
                    (interval.Touch(*i) &&
                     i->lower_bound() != interval.upper_bound())) {
                    return i;
                }
            }
        } else {
            for (ConstIterator i = lower_it; i != end(); ++i) {
                if (i->lower_bound() > interval.upper_bound()) break;
                if (interval.Overlap(*i) ||
                    (interval.Touch(*i) &&
                     i->lower_bound() != interval.upper_bound())) {
                    return i;
                }
            }
        }

        return end();
    }

    /**
     * Return the iterator to the right-most interval that touch or overlap
     * with the given interval.
     */
    ConstIterator Upper(const It& interval) const {
        CHECK(!interval.empty());

        if (set_.empty()) return end();

        It tmp(interval.upper_bound(), interval.upper_bound(), It::CLOSED, 
               It::CLOSED);
        ConstIterator it = set_.lower_bound(tmp);
        if (it == end()) {
            it = (++set_.rbegin()).base();
        }

        if (!interval.Overlap(*it) && !interval.Touch(*it)) {
            --it;
            if (it != end() &&
                !interval.Overlap(*it) && !interval.Touch(*it)) {
                return end();
            }
        }

        return it;
    }

    /**
     * Clear the interval set.
     */
    void clear() {
        set_.clear();
    }

    /**
     * Return true if the interval set is empty.
     */
    bool empty() const {
        return set_.empty();
    }

    /**
     * Return the current number of intervals in the interval set.
     */
    int size() const {
        return set_.size();
    }

    // Iterators begin and end.
    Iterator begin()                    { return set_.begin();  }
    ConstIterator begin()         const { return set_.begin();  }
    ReverseIterator rbegin()            { return set_.rbegin(); }
    ConstReverseIterator rbegin() const { return set_.rbegin(); }
    Iterator end()                      { return set_.end();    }
    ConstIterator end()           const { return set_.end();    }
    ReverseIterator rend()              { return set_.rend();   }
    ConstReverseIterator rend()   const { return set_.rend();   }

private:
    // We store the intervals in the STL set.
    std::set<It> set_;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_INTERVAL_INTERVAL_SET_H_
