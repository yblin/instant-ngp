//
// Copyright 2016-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_INTERVAL_INTERVAL_H_
#define CODELIBRARY_UTIL_INTERVAL_INTERVAL_H_

#include <limits>
#include <ostream>

#include "codelibrary/base/message.h"

namespace cl {

/**
 * An interval is a pair of numbers which represents all the numbers between
 * the pair.
 */
template <typename T>
class Interval {
public:
    // Bound type.
    enum BoundType {
        CLOSED = 0,
        OPEN   = 1
    };

    /**
     * The default interval is an invalid interval: [inf, -inf].
     */
    Interval()
        : lower_bound_(std::numeric_limits<T>::max()),
          upper_bound_(std::numeric_limits<T>::lowest()),
          lower_bound_type_(CLOSED),
          upper_bound_type_(CLOSED) {}

    /**
     * Note that the interval may be empty.
     */
    Interval(const T& lower_bound, const T& upper_bound,
             BoundType lower_bound_type, BoundType upper_bound_type)
        : lower_bound_(lower_bound),
          upper_bound_(upper_bound),
          lower_bound_type_(lower_bound_type),
          upper_bound_type_(upper_bound_type) {
        CHECK(!empty());
    }

    /**
     * Check if a given interval is valid.
     */
    static bool IsValid(const T& lower_bound,
                        const T& upper_bound,
                        BoundType lower_bound_type,
                        BoundType upper_bound_type) {
        return (lower_bound_type == CLOSED && upper_bound_type == CLOSED &&
                lower_bound <= upper_bound) || lower_bound < upper_bound;
    }

    /**
     * Create a left open interval: [l, r).
     */
    static Interval LeftOpen(const T& lower_bound, const T& upper_bound) {
        return Interval(lower_bound, upper_bound, CLOSED, OPEN);
    }

    /**
     * Create a right open interval: (l, r].
     */
    static Interval RightOpen(const T& lower_bound, const T& upper_bound) {
        return Interval(lower_bound, upper_bound, OPEN, CLOSED);
    }

    /**
     * Create a closed interval: [l, r].
     */
    static Interval Closed(const T& lower_bound, const T& upper_bound) {
        return Interval(lower_bound, upper_bound, CLOSED, CLOSED);
    }

    /**
     * Create a open interval: (l, r).
     */
    static Interval Open(const T& lower_bound, const T& upper_bound) {
        return Interval(lower_bound, upper_bound, OPEN, OPEN);
    }

    /**
     * Return true if the interval is empty.
     */
    bool empty() const {
        return !IsValid(lower_bound_, upper_bound_,
                        lower_bound_type_, upper_bound_type_);
    }

    /**
     * Return the lower bound of the interval.
     */
    const T& lower_bound() const {
        return lower_bound_;
    }

    /**
     * Return the upper bound of the interval.
     */
    const T& upper_bound() const {
        return upper_bound_;
    }

    /**
     * Return the type of lower bound.
     */
    BoundType lower_bound_type() const {
        return lower_bound_type_;
    }

    /**
     * Return the type of upper bound.
     */
    BoundType upper_bound_type() const {
        return upper_bound_type_;
    }

    bool operator ==(const Interval& rhs) const {
        // We do not want to compare the empty set.
        CHECK(!empty() && !rhs.empty());

        return lower_bound_ == rhs.lower_bound_ &&
               upper_bound_ == rhs.upper_bound_ &&
               lower_bound_type_ == rhs.lower_bound_type_ &&
               upper_bound_type_ == rhs.upper_bound_type_;
    }

    bool operator !=(const Interval& rhs) const {
        return !(*this == rhs);
    }

    bool operator <(const Interval& rhs) const {
        if (lower_bound_ != rhs.lower_bound_)
            return lower_bound_ < rhs.lower_bound_;
        if (lower_bound_type_ != rhs.lower_bound_type_)
            return lower_bound_type_ < rhs.lower_bound_type_;
        if (upper_bound_ != rhs.upper_bound_)
            return upper_bound_ < rhs.upper_bound_;
        return rhs.upper_bound_type_ < upper_bound_type_;
    }

    /**
     * Check if this interval is overlap with another interval.
     */
    bool Overlap(const Interval& y) const {
        if (empty() || y.empty()) return false;

        if (upper_bound_ < y.lower_bound_ ||
            lower_bound_ > y.upper_bound_) return false;

        if (upper_bound_ == y.lower_bound_) {
            return upper_bound_type_ == CLOSED && y.lower_bound_type_ == CLOSED;
        }

        if (lower_bound_ == y.upper_bound_) {
            return lower_bound_type_ == CLOSED && y.upper_bound_type_ == CLOSED;
        }

        return true;
    }

    /**
     * Check if this interval touches (but not intersect with) another interval.
     *
     * For example: [3, 4) and [4, 5] are touched, but [3, 4] and [4, 5] is not
     * touched.
     */
    bool Touch(const Interval& y) const {
        if (empty() || y.empty()) return false;

        if (upper_bound_ == y.lower_bound_ &&
            upper_bound_type_ != y.lower_bound_type_)
            return true;
        if (lower_bound_ == y.upper_bound_ &&
            lower_bound_type_ != y.upper_bound_type_)
            return true;

        return false;
    }

    /**
     * Join the interval, the result is the hull of two intervals.
     */
    void Join(const Interval& y) {
        if (empty()) {
            *this = y;
            return;
        }
        if (y.empty()) return;

        if (lower_bound_ == y.lower_bound_) {
            if (y.lower_bound_type_ == CLOSED)
                lower_bound_type_ = CLOSED;
        } else if (lower_bound_ > y.lower_bound_) {
            lower_bound_ = y.lower_bound_;
            lower_bound_type_ = y.lower_bound_type_;
        }

        if (upper_bound_ == y.upper_bound_) {
            if (y.upper_bound_type_ == CLOSED)
                upper_bound_type_ = CLOSED;
        } else if (upper_bound_ < y.upper_bound_) {
            upper_bound_ = y.upper_bound_;
            upper_bound_type_ = y.upper_bound_type_;
        }
    }

    /**
     * Output the interval, for debug only.
     */
    friend std::ostream& operator <<(std::ostream& os,
                                     const Interval& interval) {
        if (interval.empty()) {
            os << "(empty interval)";
            return os;
        }

        Message msg;
        msg << ((interval.lower_bound_type_ == OPEN) ? "(" : "[");
        msg << interval.lower_bound_ << " " << interval.upper_bound_;
        msg << ((interval.upper_bound_type_ == OPEN) ? ")" : "]");
        os << msg;
        return os;
    }

protected:
    // The lower bound and upper bound of the interval.
    T lower_bound_, upper_bound_;
    BoundType lower_bound_type_, upper_bound_type_;
};

} // namespace cl

#endif // CODELIBRARY_UTIL_INTERVAL_INTERVAL_H_
