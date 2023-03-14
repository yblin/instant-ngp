//
// Copyright 2013-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_FRACTION_FRACTION_H_
#define CODELIBRARY_MATH_FRACTION_FRACTION_H_

#include "codelibrary/math/common_factor.h"

namespace cl {

/**
 * Fraction arithmetic.
 */ 
template <typename IntType>
class Fraction {
public:
    explicit Fraction(const IntType& numerator = 0,
                      const IntType& denominator = 1)
        : numerator_(numerator),
          denominator_(denominator) {
        assert(denominator_ != 0);

        if (denominator_ < 0) {
            numerator_ = -numerator_;
            denominator_ = -denominator_;
        }
    }

    bool operator ==(const Fraction& rhs) const { return Compare(rhs) == 0; }
    bool operator !=(const Fraction& rhs) const { return Compare(rhs) != 0; }
    bool operator < (const Fraction& rhs) const { return Compare(rhs) <  0; }
    bool operator <=(const Fraction& rhs) const { return Compare(rhs) <= 0; }
    bool operator > (const Fraction& rhs) const { return Compare(rhs) >  0; }
    bool operator >=(const Fraction& rhs) const { return Compare(rhs) >= 0; }
    bool operator ==(const IntType& rhs)  const { return Compare(rhs) == 0; }
    bool operator !=(const IntType& rhs)  const { return Compare(rhs) != 0; }
    bool operator < (const IntType& rhs)  const { return Compare(rhs) <  0; }
    bool operator <=(const IntType& rhs)  const { return Compare(rhs) <= 0; }
    bool operator > (const IntType& rhs)  const { return Compare(rhs) >  0; }
    bool operator >=(const IntType& rhs)  const { return Compare(rhs) >= 0; }

    Fraction& operator = (const IntType& number) {
        numerator_ = number;
        denominator_ = 1;
        return *this;
    }

    Fraction& operator += (const Fraction& rhs) {
        if (denominator_ == rhs.denominator_) {
            numerator_ += rhs.numerator_;
            return *this;
        }

        numerator_ = numerator_ * rhs.denominator_ +
                     denominator_ * rhs.numerator_;
        denominator_ *= rhs.denominator_;
        return *this;
    }

    Fraction& operator -= (const Fraction& rhs) {
        if (denominator_ == rhs.denominator_) {
            numerator_ -= rhs.numerator_;
            return *this;
        }

        numerator_ = numerator_ * rhs.denominator_ -
                     denominator_ * rhs.numerator_;
        denominator_ *= rhs.denominator_;
        return *this;
    }

    Fraction& operator *= (const Fraction& rhs) {
        numerator_   *= rhs.numerator_;
        denominator_ *= rhs.denominator_;
        return *this;
    }

    Fraction& operator /= (const Fraction& rhs) {
        assert(rhs.numerator_ != 0);

        numerator_   *= rhs.denominator_;
        denominator_ *= rhs.numerator_;
        return *this;
    }

    void set_numerator(const IntType& numerator) {
        numerator_ = numerator;
    }

    void set_denominator(const IntType& denominator) {
        assert(denominator != 0);
        denominator_ = denominator;
    }

    const IntType& numerator()   const { return numerator_;   }
    const IntType& denominator() const { return denominator_; }

    /**
     * Simplify the fraction by divide both numerator and denominator by
     * their greatest common divisor.
     */
    void Simplify() {
        if (numerator_ == 0) {
            denominator_ = 1;
            return;
        }

        IntType gcd = GCD(numerator_, denominator_);
        numerator_ /= gcd;
        denominator_ /= gcd;
    }

    friend Fraction operator -(const Fraction& rhs) {
        return Fraction(-rhs.numerator_, rhs.denominator_);
    }

    friend Fraction operator +(const Fraction& lhs, const Fraction& rhs) {
        if (lhs.denominator_ == rhs.denominator_) {
            return Fraction(lhs.numerator_ + rhs.numerator_, lhs.denominator_);
        }

        IntType numerator = lhs.numerator_ * rhs.denominator_ +
                            rhs.numerator_ * lhs.denominator_;
        IntType denominator = lhs.denominator_ * rhs.denominator_;
        return Fraction(numerator, denominator);
    }

    friend Fraction operator -(const Fraction& lhs, const Fraction& rhs) {
        if (lhs.denominator_ == rhs.denominator_) {
            return Fraction(lhs.numerator_ - rhs.numerator_, lhs.denominator_);
        }

        IntType numerator = lhs.numerator_ * rhs.denominator_ -
                            rhs.numerator_ * lhs.denominator_;
        IntType denominator = lhs.denominator_ * rhs.denominator_;
        return Fraction(numerator, denominator);
    }

    friend Fraction operator *(const Fraction& lhs, const Fraction& rhs) {
        IntType numerator = lhs.numerator_ * rhs.numerator_;
        IntType denominator = lhs.denominator_ * rhs.denominator_;
        return Fraction(numerator, denominator);
    }

    friend Fraction operator /(const Fraction& lhs, const Fraction& rhs) {
        assert(rhs.numerator_ != 0);

        IntType numerator = lhs.numerator_ * rhs.denominator_;
        IntType denominator = lhs.denominator_ * rhs.numerator_;
        return Fraction(numerator, denominator);
    }

private:
    /**
     * Compare this Fraction to Fraction rhs.
     * Return +1, if this > rhs;
     *        -1, if this < rhs;
     *         0, otherwise.
     */
    int Compare(const Fraction& rhs) const {
        IntType a = rhs.denominator_ * numerator_;
        IntType b = denominator_ * rhs.numerator_;
        if (a == b) return 0;

        if (denominator_ * rhs.denominator_ < 0) {
            return a > b ? -1 : 1;
        }

        return a < b ? -1 : 1;
    }

    /**
     * Compare this Fraction to rhs.
     * Return +1, if this > rhs;
     *        -1, if this < rhs;
     *         0, otherwise.
     */
    int Compare(const IntType& rhs) const {
        IntType b = denominator_ * rhs;
        if (numerator_ == b) return 0;

        if (rhs < 0) {
            return numerator_ > b ? -1 : 1;
        }

        return numerator_ < b ? -1 : 1;
    }

    IntType numerator_;   // The numerator of fraction.
    IntType denominator_; // The denominator of fraction.
};

} // namespace cl

#endif // CODELIBRARY_MATH_FRACTION_FRACTION_H_
