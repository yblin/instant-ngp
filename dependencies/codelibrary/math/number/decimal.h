//
// Copyright 2014 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_MATH_NUMBER_DECIMAL_H_
#define CODELIBRARY_MATH_NUMBER_DECIMAL_H_

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <istream>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

namespace cl {

/**
 * Decimal provides support for decimal floating point arithmetic.
 *
 * A Decimal characterized by three components: a sign, a signed exponent, and a
 * significand. Decimals also include special values such as +Infinity,
 * -Infinity and NaN (Not a number).
 *
 * The rounding strategy of Decimal is truncate rounding, i.e., 4.5->4.
 *
 * The precision of Decimal is given as the template parameter 'PRECISION'.
 */
template <int PRECISION>
class Decimal {
    // Flag of a decimal.
    enum Flag {
        POSITIVE_INFINITY,
        NEGATIVE_INFINITY,
        NOT_A_NUMBER,
        NORMAL_NUMBER
    };

    // The radix of Decimal.
    static const int RADIX = 10000;

    // The digits of radix.
    static const int RADIX_DIGITS = 4;

    // The maximum exponenet of Decimal.
    static const int MAX_EXPONENT = 999999999;

    // The minimum exponenet of Decimal.
    static const int MIN_EXPONENT = -999999999;

    // Threshold for Karatsuba multiplication.
    static const int KARATSUBA_MULTIPLY_THRESHOLD = 128;

public:
    // Constructors.
    Decimal()                       { Assign(0);   }
    Decimal(int32_t n)              { Assign(n);   }
    Decimal(uint32_t n)             { Assign(n);   }
    Decimal(int64_t n)              { Assign(n);   }
    Decimal(uint64_t n)             { Assign(n);   }
    Decimal(const char* str)        { Assign(str); }
    Decimal(const std::string& str) { Assign(str); }

    /**
     * Convert the Decimal to string.
     */
    std::string ToString() const {
        switch (flag_) {
        case POSITIVE_INFINITY: return "inf";
        case NEGATIVE_INFINITY: return "-inf";
        case NOT_A_NUMBER:      return "nan";
        default: break;
        }

        if (sign_ == 0) return "0";

        std::string str;
        for (int i = size_ - 1; i >= 0; --i) {
            std::string tmp(RADIX_DIGITS, '0');
            int index = RADIX_DIGITS;
            int number = data_[i];
            while (number != 0) {
                tmp[--index] = number % 10 + '0';
                number /= 10;
            }

            str += tmp;
        }
        size_t index = 0;
        while (index < str.size() - 1 && str[index] == '0') {
            ++index;
        }
        str.erase(0, index);

        assert(str.size() <= INT_MAX);
        int len = static_cast<int>(str.size());
        int exponent = exponent_;
        if (len > PRECISION) {
            str = str.substr(0, PRECISION);
            exponent += len - PRECISION;
            len = PRECISION;
        }

        if (exponent > 0) {
            if (len + exponent > PRECISION) {
                str = ToScientificNotion(str, exponent);
            } else {
                for (int i = 0; i < exponent; ++i) {
                    str += '0';
                }
            }
        } else if (exponent < 0) {
            if (std::abs(exponent) > PRECISION) {
                str = ToScientificNotion(str, exponent);
            } else if (std::abs(exponent) >= len) {
                int offset = std::abs(exponent) - len;
                std::string leading_zero = "0.";
                for (int i = 0; i < offset; ++i) {
                    leading_zero += "0";
                }
                str = leading_zero + str;
                DeleteTailingZero(&str);
            } else {
                str.insert(len - std::abs(exponent), ".");
                DeleteTailingZero(&str);
            }
        }

        return sign_ < 0 ? "-" + str : str;
    }

    // Compare operators.
    bool operator ==(const Decimal& rhs) const {
        return !IsUnordered(*this, rhs) && Compare(rhs) == 0;
    }

    bool operator !=(const Decimal& rhs) const {
        return !(*this == rhs);
    }

    bool operator < (const Decimal& rhs) const {
        return !IsUnordered(*this, rhs) && Compare(rhs) < 0;
    }

    bool operator > (const Decimal& rhs) const {
        return !IsUnordered(*this, rhs) && Compare(rhs) > 0;
    }

    bool operator <=(const Decimal& rhs) const {
        return !IsUnordered(*this, rhs) && Compare(rhs) <= 0;
    }

    bool operator >=(const Decimal& rhs) const {
        return !IsUnordered(*this, rhs) && Compare(rhs) >= 0;
    }

    // Assign operators.
    Decimal& operator = (int32_t rhs)            { return Assign(rhs); }
    Decimal& operator = (uint32_t rhs)           { return Assign(rhs); }
    Decimal& operator = (int64_t rhs)            { return Assign(rhs); }
    Decimal& operator = (uint64_t rhs)           { return Assign(rhs); }
    Decimal& operator = (const std::string& rhs) { return Assign(rhs); }

    // Arithmetic operators.
    Decimal& operator +=(const Decimal& rhs) { return Add(*this, rhs);      }
    Decimal& operator -=(const Decimal& rhs) { return Add(*this, -rhs);     }
    Decimal& operator *=(const Decimal& rhs) { return Multiply(*this, rhs); }
    Decimal& operator /=(const Decimal& rhs) { return Divide(*this, rhs);   }

    const Flag& flag()             const { return flag_;     }
    int sign()                     const { return sign_;     }
    int exponent()                 const { return exponent_; }
    int size()                     const { return size_;     }
    const std::vector<int>& data() const { return data_;     }

    friend Decimal operator -(const Decimal& rhs) {
        Decimal c(rhs);
        if (c.flag_ == POSITIVE_INFINITY) c.flag_ = NEGATIVE_INFINITY;
        if (c.flag_ == NEGATIVE_INFINITY) c.flag_ = POSITIVE_INFINITY;

        c.sign_ = -c.sign_;
        return c;
    }

    friend Decimal operator +(const Decimal& lhs, const Decimal& rhs) {
        Decimal t;
        t.Add(lhs, rhs);
        return t;
    }

    friend Decimal operator -(const Decimal& lhs, const Decimal& rhs) {
        Decimal t;
        t.Add(lhs, -rhs);
        return t;
    }

    friend Decimal operator *(const Decimal& lhs, const Decimal& rhs) {
        Decimal t;
        t.Multiply(lhs, rhs);
        return t;
    }

    friend Decimal operator /(const Decimal& lhs, const Decimal& rhs) {
        Decimal t;
        t.Divide(lhs, rhs);
        return t;
    }

    /**
     * Overloaded input stream, the radix is 10.
     */
    friend std::istream& operator >>(std::istream& is, Decimal& rhs) {
        std::string str;
        if (!(is >> str)) return is;
        rhs.Assign(str);
        return is;
    }

    /**
     * Overloaded output stream, the default radix is 10.
     */
    friend std::ostream& operator <<(std::ostream& os, const Decimal& rhs) {
        os << rhs.ToString();
        return os;
    }

private:
    /**
     * Check if the two Decimals is unordered.
     */
    bool IsUnordered(const Decimal& a, const Decimal& b) const {
        return a.flag_ == NOT_A_NUMBER || b.flag_ == NOT_A_NUMBER;
    }

    /**
     * Count the digits of Decimal.
     */
    int Digits() const {
        assert(flag_ == NORMAL_NUMBER);

        if (size_ == 0) return 0;
        return (size_ - 1) * RADIX_DIGITS + Log10(data_[size_ - 1]);
    }

    /**
     * This = 0.
     */
    Decimal& SetZero() {
        size_ = 0;
        sign_ = 0;
        exponent_ = 0;
        flag_ = NORMAL_NUMBER;
        return *this;
    }

    /**
     * This = a + b.
     */
    Decimal& Add(const Decimal& a, const Decimal& b,
                 int precision = PRECISION) {
        if (a.flag_ == NOT_A_NUMBER || b.flag_ == NOT_A_NUMBER) {
            flag_ = NOT_A_NUMBER;
            return *this;
        }
        if ((a.flag_ == POSITIVE_INFINITY && b.flag_ == NEGATIVE_INFINITY) ||
            (a.flag_ == NEGATIVE_INFINITY && b.flag_ == POSITIVE_INFINITY)) {
            flag_ = NOT_A_NUMBER;
            return *this;
        }
        if (a.flag_ == POSITIVE_INFINITY && b.flag_ == POSITIVE_INFINITY) {
            flag_ = POSITIVE_INFINITY;
            return *this;
        }
        if (a.flag_ == NEGATIVE_INFINITY && b.flag_ == NEGATIVE_INFINITY) {
            flag_ = NEGATIVE_INFINITY;
            return *this;
        }

        if (a.sign_ == 0) return *this = b;
        if (b.sign_ == 0) return *this = a;

        if (std::abs(a.exponent_ - b.exponent_) > precision) {
            return *this = (a.exponent_ > b.exponent_) ? a : b;
        }

        if (a.sign_ == b.sign_) {
            sign_ = a.sign_;
            if (a.exponent_ == b.exponent_) {
                exponent_ = a.exponent_;
                AddData(a, b);
            } else {
                if (a.exponent_ < b.exponent_) {
                    Decimal c;
                    c.ShiftLeftData(b, b.exponent_ - a.exponent_);
                    exponent_ = a.exponent_;
                    AddData(a, c);
                } else {
                    Decimal c;
                    c.ShiftLeftData(a, a.exponent_ - b.exponent_);
                    exponent_ = b.exponent_;
                    AddData(c, b);
                }
            }
        } else {
            if (a.exponent_ == b.exponent_) {
                exponent_ = a.exponent_;
                int res = a.CompareData(b);
                if (res == 0) return SetZero();
                if (res < 0) {
                    SubtractData(b, a);
                    sign_ = -a.sign_;
                } else {
                    SubtractData(a, b);
                    sign_ = a.sign_;
                }
            } else if (a.exponent_ < b.exponent_) {
                Decimal c;
                c.ShiftLeftData(b, b.exponent_ - a.exponent_);

                int res = a.CompareData(c);
                if (res == 0) return SetZero();
                if (res < 0) {
                    SubtractData(c, a);
                    sign_ = -a.sign_;
                } else {
                    SubtractData(a, c);
                    sign_ = a.sign_;
                }
                exponent_ = a.exponent_;
            } else {
                Decimal c;
                c.ShiftLeftData(a, a.exponent_ - b.exponent_);

                int res = c.CompareData(b);
                if (res == 0) return SetZero();
                if (res < 0) {
                    SubtractData(b, c);
                    sign_ = -a.sign_;
                } else {
                    SubtractData(c, b);
                    sign_ = a.sign_;
                }
                exponent_ = b.exponent_;
            }
        }

        return Truncate(precision);
    }

    /**
     * This = a + b, ignore the sign and exponent.
     */
    Decimal& AddData(const Decimal& a, const Decimal& b) {
        // a1 points to the longer input, b1 points to the shorter.
        const std::vector<int>*a1 = &a.data_, *b1 = &b.data_;
        int a_size = a.size_;
        int b_size = b.size_;
        if (a_size < b_size) {
            std::swap(a1, b1);
            std::swap(a_size, b_size);
        }

        data_.resize(a_size + 1);

        bool carry = false;
        int i;

        for (i = 0; i < b_size; ++i) {
            data_[i] = (*a1)[i] + (*b1)[i] + static_cast<int>(carry);
            carry = (data_[i] >= RADIX);
            if (carry) data_[i] -= RADIX;
        }

        for (i = b_size; i < a_size; ++i) {
            data_[i] = (*a1)[i] + static_cast<int>(carry);
            carry = (data_[i] >= RADIX);
            if (carry) data_[i] -= RADIX;
        }

        if (carry) {
            data_[i] = 1;
            size_ = i + 1;
        } else {
            size_ = i;
        }

        return *this;
    }

    /**
     * This = a - b, ignore the sign and exponent.
     * Note that, a must not less than b.
     */
    Decimal& SubtractData(const Decimal& a, const Decimal& b) {
        int a_size = a.size_;
        int b_size = b.size_;
        data_.resize(a_size);

        bool carry = false;
        int i;

        for (i = 0; i < b_size; ++i) {
            data_[i] = a.data_[i] - b.data_[i] - static_cast<int>(carry);
            carry = (data_[i] < 0);
            if (carry) data_[i] += RADIX;
        }

        for (i = b_size; i < a_size; ++i) {
            data_[i] = a.data_[i] - carry;
            carry = (data_[i] < 0);
            if (carry) data_[i] += RADIX;
        }

        size_ = a_size;
        return DeleteLeadingZero();
    }

    /**
     * This = a * b.
     *
     * Auto choose multiplication algorithm by two Decimal's sizes.
     */
    Decimal& Multiply(const Decimal& a, const Decimal& b,
                      int precision = PRECISION) {
        if (a.flag_ == NOT_A_NUMBER || b.flag_ == NOT_A_NUMBER) {
            flag_ = NOT_A_NUMBER;
            return *this;
        }
        if ((a.flag_ == POSITIVE_INFINITY && b.flag_ == NEGATIVE_INFINITY) ||
            (a.flag_ == NEGATIVE_INFINITY && b.flag_ == POSITIVE_INFINITY)) {
            flag_ = NEGATIVE_INFINITY;
            return *this;
        }
        if (a.flag_ == POSITIVE_INFINITY && b.flag_ == POSITIVE_INFINITY) {
            flag_ = POSITIVE_INFINITY;
            return *this;
        }
        if (a.flag_ == NEGATIVE_INFINITY && b.flag_ == NEGATIVE_INFINITY) {
            flag_ = POSITIVE_INFINITY;
            return *this;
        }

        if (a.sign_ == 0) return this->SetZero();
        if (b.sign_ == 0) return this->SetZero();

        MultiplyData(a, b);
        sign_ = a.sign_ * b.sign_;
        exponent_ = a.exponent_ + b.exponent_;

        return Truncate(precision);
    }

    /**
     * This = a * b. Ignore the sign and exponent.
     */
    Decimal& MultiplyData(const Decimal& a, const Decimal& b) {
        if (a.size_ > KARATSUBA_MULTIPLY_THRESHOLD &&
            b.size_ > KARATSUBA_MULTIPLY_THRESHOLD) {
            return MultiplyKaratsuba(a, b);
        }
        return MultiplySimple(a, b);
    }

    /**
     * This = a * b, ignore the flag, sign and exponent.
     *
     * Using baseline/comba algorithm.
     */
    Decimal& MultiplySimple(const Decimal& a, const Decimal& b) {
        int a_size = a.size_;
        int b_size = b.size_;
        int size = a_size + b_size;
        std::vector<int> data(size, 0);

        for (int i = 0; i < a_size; ++i) {
            int carry = 0;
            if (a.data_[i] == 0) continue;

            for (int j = 0; j < b_size; ++j) {
                data[i + j] += a.data_[i] * b.data_[j] + carry;
                carry = data[i + j] / RADIX;
                if (carry > 0) data[i + j] %= RADIX;
            }
            data[i + b_size] = carry;
        }

        data_ = data;
        size_ = size;
        DeleteLeadingZero();

        return *this;
    }

    /**
     * This = a * b.
     *
     * Using Karatsuba algorithm.
     *
     * This is known as divide-and-conquer and leads to the famous O(N ^ log(3))
     * or O(N ^ 1.584) work which is asymptotically lower than the standard
     * O(N ^ 2) that the baseline/comba methods use.
     */
    Decimal& MultiplyKaratsuba(const Decimal& a, const Decimal& b) {
        int a_size = a.size_;
        int b_size = b.size_;
        int size = std::min(a_size, b_size);

        if (size < KARATSUBA_MULTIPLY_THRESHOLD) {
            return this->MultiplySimple(a, b);
        }

        size >>= 1;

        if (a_size * 2 < b_size || b_size * 2 < a_size) {
            const Decimal *ta = &a, *tb = &b;
            if (a_size > b_size) std::swap(ta, tb);

            Decimal t, sum = 0;
            t.data_.resize(ta->size_);
            for (int i = 0; i < tb->size_; i += ta->size_) {
                t.size_ = std::min(ta->size_, tb->size_ - i);
                for (int j = 0; j < t.size_; ++j) {
                    t.data_[j] = tb->data_[i + j];
                }
                t.DeleteLeadingZero();
                t.MultiplyData(t, *ta);
                if (i > 0) t.ShiftLeftData(t, i * RADIX_DIGITS);
                sum.AddData(sum, t);
            }
            return *this = sum;
        }

        Decimal x0, x1, y0, y1;
        x0.data_.resize(size), x1.data_.resize(a_size - size);
        y0.data_.resize(size), y1.data_.resize(b_size - size);
        x0.size_ = y0.size_ = size;
        x1.size_ = a_size - size;
        y1.size_ = b_size - size;

        std::copy(a.data_.begin(), a.data_.begin() + size, x0.data_.begin());
        std::copy(b.data_.begin(), b.data_.begin() + size, y0.data_.begin());
        std::copy(a.data_.begin() + size, a.data_.begin() + a_size,
                  x1.data_.begin());
        std::copy(b.data_.begin() + size, b.data_.begin() + b_size,
                  y1.data_.begin());

        x0.DeleteLeadingZero();
        y0.DeleteLeadingZero();

        Decimal x0y0, x1y1, t1;
        x0y0.MultiplyKaratsuba(x0, y0);  // x0y0 = x0 * y0
        x1y1.MultiplyKaratsuba(x1, y1);  // x1y1 = x1 * y1
        t1.AddData(x1, x0);              // t1 = x1 + x0
        x0.AddData(y1, y0);              // x0 = y1 + y0
        t1.MultiplyKaratsuba(t1, x0);    // t1 = (x1 + x0) * (y1 + y0)
        x0.AddData(x0y0, x1y1);          // x0 = x0y0 + x1y1
        t1.SubtractData(t1, x0);         // t1 = (x1 + x0) * (y1 + y0) -
                                         //      (x1y1 + x0y0)

        t1.ShiftLeftData(t1, size * RADIX_DIGITS);
        x1y1.ShiftLeftData(x1y1, (size + size) * RADIX_DIGITS);

        t1.AddData(x0y0, t1);               // t1 = x0y0 + t1
        return this->AddData(x1y1, t1);     // *this = x1y1 + t1
    }

    /**
     * This = a / b.
     * Newton–Raphson division.
     */
    Decimal& Divide(const Decimal& a, const Decimal& b) {
        if ((a.flag_ != NORMAL_NUMBER && b.flag_ != NORMAL_NUMBER) ||
            (b.sign_ == 0)) {
            flag_ = NOT_A_NUMBER;
            return *this;
        }

        if (a.sign_ == 0) return this->SetZero();

        // x = 1 / b.
        Decimal x = RADIX / b.data_[b.size_ - 1];
        x.exponent_ -= b.exponent_ + b.size_ * RADIX_DIGITS;
        x.sign_ = b.sign_;

        int n_iterations = static_cast<int>(std::log2(PRECISION + 1));
        Decimal new_x, t;
        for (int i = 0; i < n_iterations; ++i) {
            new_x.Add(x, x, PRECISION + 2);
            t.Multiply(x, x, PRECISION + 2);
            t.Multiply(t, b, PRECISION + 2);
            new_x.Add(new_x, -t, PRECISION + 2);
            if (x == new_x) break;
            x = new_x;
        }

        this->MultiplyData(a, x); // a / b = a * (1 / b) = a * x.
        sign_ = a.sign_ * b.sign_;
        exponent_ = a.exponent_ + x.exponent_;

        return Truncate();
    }

    /**
     * Assign Decimal from an integer.
     */
    template <typename IntType>
    Decimal& Assign(IntType number) {
        static_assert(std::is_integral<IntType>::value,
                      "template argument is not a integral type");
        static_assert(PRECISION > 0, "Precision should be positive.");

        size_ = 0;
        flag_ = NORMAL_NUMBER;
        exponent_ = 0;
        data_.resize(std::numeric_limits<IntType>::digits10 / RADIX_DIGITS + 1);
        if (number == 0) {
            sign_ = 0;
            return *this;
        }

        sign_ = number > 0 ? 1 : -1;
        if (number < 0) number = -number;

        while (number) {
            data_[size_++] = static_cast<int>(number % RADIX);
            number /= RADIX;
        }
        return Truncate();
    }

    /**
     * Assign Decimal from string.
     *
     * The valid string must be one of the following entities:
     *   1) ^[-+]?([0-9]+(.[0-9]*)?|.[0-9]+|[0-9]+.)([eE][-+]?[0-9]+)?$;
     *   2) "+Inf", "-Inf" or "NaN".
     */
    Decimal& Assign(const std::string& str) {
        static_assert(PRECISION > 0, "Precision should be positive.");
        assert(!str.empty());

        size_ = 0;
        exponent_ = 0;
        if (str == "inf") {
            flag_ = POSITIVE_INFINITY;
            return *this;
        }
        if (str == "-inf") {
            flag_ = NEGATIVE_INFINITY;
            return *this;
        }
        if (str == "nan") {
            flag_ = NOT_A_NUMBER;
            return *this;
        }

        flag_ = NORMAL_NUMBER;
        sign_ = (str[0] == '-') ? -1 : 1;

        // Split the string into two parts by 'e' or 'E'.
        std::string str_part1, str_part2;
        int flag = 0;
        for (char i : str) {
            if (i == 'e' || i == 'E') {
                ++flag;
                // Only one 'e/E' allowed.
                assert(flag == 1);
            } else {
                if (flag != 0)
                    str_part2 += i;
                else
                    str_part1 += i;
            }
        }

        if (str_part1[0] == '+' || str_part1[0] == '-')
            str_part1 = str_part1.substr(1);

        assert(!str_part1.empty()); // Float number can not be empty.
        if (flag != 0) {
            assert(!str_part2.empty()); // Exponent has no digits.

            char* end = nullptr;
            errno = 0;
            exponent_ = strtol(str_part2.c_str(), &end, 10);
            assert(strlen(end) == 0); // Exponent has non digit number.
        }

        // Find the index of '.' and remove it.
        size_t index = str_part1.find('.');
        if (index == std::string::npos) {
            index = str_part1.size();
        } else {
            str_part1.erase(index, 1);
            int diff = static_cast<int>(str_part1.size() - index);
            assert(diff >= 0);
            exponent_ -= diff;
        }
        assert(!str_part1.empty());

        int n = static_cast<int>(str_part1.size() / RADIX_DIGITS + 1);
        assert(n > 0);

        data_.resize(n);
        for (size_t i = str_part1.size(); i > 0; i -= RADIX_DIGITS) {
            size_t begin = i < RADIX_DIGITS ? 0 : i - RADIX_DIGITS;
            int tmp = 0;
            for (size_t j = begin; j < i; ++j) {
                // Float point has non digit number.
                assert(str_part1[j] >= '0' && str_part1[j] <= '9');

                tmp = 10 * tmp + str_part1[j] - '0';
            }
            data_[size_++] = tmp;

            if (begin == 0) break;
        }

        DeleteLeadingZero();
        return (size_ == 0) ? SetZero() : Truncate();
    }

    /**
     * See Assign(const std::string&) for more details.
     */
    Decimal& Assign(const char* str) {
        return Assign(std::string(str));
    }

    /**
     * Compare this Decimal's data to that of other Decimal.
     *
     * Return +1, if this > a;
     *        -1, if this < a;
     *         0, otherwise.
     */
    int CompareData(const Decimal& a) const {
        if (size_ != a.size_) {
            return size_ < a.size_ ? -1 : 1;
        }
        for (int i = size_ - 1; i >= 0; --i) {
            if (data_[i] == a.data_[i]) continue;
            return (data_[i] < a.data_[i]) ? -1 : 1;
        }
        return 0;
    }

    /**
     * Compare this Decimal to Decimal a.
     *
     * Return +1, if this > a;
     *        -1, if this < a;
     *         0, otherwise.
     */
    int Compare(const Decimal& a) const {
        if (flag_ == POSITIVE_INFINITY) {
            return (a.flag_ == POSITIVE_INFINITY) ? 0 : 1;
        }
        if (flag_ == NEGATIVE_INFINITY) {
            return (a.flag_ == NEGATIVE_INFINITY) ? 0 : -1;
        }

        if (sign_ == a.sign_) {
            if (exponent_ == a.exponent_) {
                return this->CompareData(a) * sign_;
            }

            if (exponent_ > a.exponent_) {
                Decimal c;
                c.ShiftLeftData(*this, exponent_ - a.exponent_);
                return c.CompareData(a) * sign_;
            }
            Decimal c;
            c.ShiftLeftData(a, a.exponent_ - exponent_);
            return this->CompareData(c) * sign_;
        }
        return sign_ == 0 ? a.sign_ * -1 : sign_;
    }

    /**
     * Decreases size to eliminate any leading zero blocks.
     */
    Decimal& DeleteLeadingZero() {
        while (size_ > 0 && data_[size_ - 1] == 0) {
            --size_;
        }
        return *this;
    }

    /**
     * Truncate the Decimal according to the PRECISION.
     */
    Decimal& Truncate(int precision = PRECISION) {
        int n_digits = Digits();
        if (n_digits > precision) {
            ShiftRightData(*this, n_digits - precision);
            exponent_ += n_digits - precision;
        }

        if (exponent_ + n_digits > MAX_EXPONENT) {
            if (sign_ > 0) flag_ = POSITIVE_INFINITY;
            if (sign_ < 0) flag_ = NEGATIVE_INFINITY;
            return *this;
        }

        if (exponent_ + n_digits < MIN_EXPONENT) {
            return this->SetZero();
        }

        return *this;
    }

    /**
     * Convert the (unsigned) string into scientific notion.
     */
    static const std::string ToScientificNotion(const std::string& significand,
                                                int exponent) {
        int len = static_cast<int>(significand.size());
        assert(len >= 0);

        std::string str = significand;
        str.insert(1, ".");
        DeleteTailingZero(&str);

        std::string str_exponenet = std::to_string(exponent + len - 1);
        if (exponent + len - 1 > 0) {
            return str + "e+" + str_exponenet;
        }

        return str + "e" + str_exponenet;
    }

    /**
     * Compute Pow(10, i).
     */
    static int Pow10(int i) {
        switch (i) {
        case 0: return 1;
        case 1: return 10;
        case 2: return 100;
        case 3: return 1000;
        default: assert(false && "Unreachable code.");
        }
        return 0;
    }

    /**
     * Compute Log10(i).
     */
    static int Log10(int i) {
        if (i >= 1000) return 4;
        if (i >= 100) return 3;
        if (i >= 10) return 2;
        if (i >= 0) return 1;

        assert(false && "Unreachable code.");
        return 0;
    }

    /**
     * Shift left the data.
     */
    Decimal& ShiftLeftData(const Decimal& a, int number) {
        assert(number > 0);

        int t1 = number / RADIX_DIGITS;
        int t2 = number % RADIX_DIGITS;

        data_.resize(a.size_ + t1 + 1);

        std::copy_backward(a.data_.begin(), a.data_.begin() + a.size_,
                           data_.begin() + a.size_ + t1);
        std::fill_n(data_.begin(), t1, 0);

        size_ = a.size_ + t1 + 1;
        data_[size_ - 1] = 0;
        if (t2 != 0) {
            for (int i = size_ - 1; i > 0; --i) {
                data_[i] = (data_[i] * Pow10(t2) +
                            data_[i - 1] / Pow10(RADIX_DIGITS - t2)) % RADIX;
            }
            data_[0] *= Pow10(t2);
            data_[0] %= RADIX;
        }
        return DeleteLeadingZero();
    }

    /**
     * Shift right the data.
     */
    Decimal& ShiftRightData(const Decimal& a, int number) {
        assert(number > 0);

        int t1 = number / RADIX_DIGITS;
        int t2 = number % RADIX_DIGITS;
        assert(t1 < a.size_);

        if (this != &a) data_.resize(a.size_);
        int size = a.size_ - t1;

        std::copy(a.data_.begin() + t1, a.data_.begin() + t1 + size,
                  data_.begin());

        if (t2 != 0) {
            for (int i = 0; i < size - 1; ++i) {
                data_[i] = (data_[i] / Pow10(t2) +
                            data_[i + 1] * Pow10(RADIX_DIGITS - t2)) % RADIX;
            }
            data_[size - 1] /= Pow10(t2);
            size_ = size;
            return DeleteLeadingZero();
        }

        size_ = size;
        return *this;
    }

    /**
     * Delete the tailing zero.
     */
    static void DeleteTailingZero(std::string* str) {
        size_t i = 0;
        for (i = str->size() - 1; i > 0; --i) {
            if ((*str)[i] != '0') break;
        }
        if ((*str)[i] == '.') --i;
        *str = str->substr(0, i + 1);
    }

    // The flag of Decimal.
    Flag flag_ = NORMAL_NUMBER;

    // Sign of Decimal (0, 1 or -1).
    int sign_ = 0;

    // Exponent of Decimal.
    int exponent_ = 0;

    // Current used size of data.
    int size_ = 0;

    // Significant data.
    std::vector<int> data_;
};

} // namespace cl

#endif // CODELIBRARY_MATH_NUMBER_DECIMAL_H_
