//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_SET_DYNAMIC_BITSET_H_
#define CODELIBRARY_UTIL_SET_DYNAMIC_BITSET_H_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/base/bits.h"

namespace cl {

/**
 * Dynamic bitset.
 *
 * The DynamicBitset class is nearly identical to the std::bitset class.
 * The difference is that the size of the DynamicBitset (the number of bits) is
 * specified at run-time during the construction, whereas the size of a
 * std::bitset is specified at compile-time through an integer template
 * parameter.
 *
 * The main problem that dynamic_bitset is designed to solve is that of
 * representing a subset of a finite set. Each bit represents whether an element
 * of the finite set is in the subset or not. As such the bitwise operations of
 * DynamicBitset, such as operator& and operator|, correspond to set operations,
 * such as intersection and union.
 */
class DynamicBitset {
    using Block = uint32_t;

    // The number of bits in one block.
    static const int BITS_PER_BLOCK = 32;

    // The mask used to set the bits (to one).
    static const Block MASK = 0xffffffffU;

    /**
     * Reference-like type.
     *
     * This embedded class is the type returned by operator[] when applied to
     * non const-qualified bitset objects. It accesses individual bits with an
     * interface that emulates a reference to a bool.
     */
    class Reference {
        friend class DynamicBitset;

        /**
         * The one and only non-copy constructor.
         */
        Reference(Block* b, int pos)
            : block_(b), mask_(Block(1) << pos) {}

    public:
        Reference(const Reference&) = delete;

                   operator bool()     const { return (*block_ & mask_) != 0; }
        bool       operator ~()        const { return (*block_ & mask_) == 0; }
        Reference& operator |=(bool x)       { return (x)  ? Set()   : *this; }
        Reference& operator &=(bool x)       { return (!x) ? Reset() : *this; }
        Reference& operator ^=(bool x)       { return (x)  ? Flip()  : *this; }

        /**
         * For b[i] = x.
         */
        Reference& operator=(bool x) { return Assign(x); }

        Reference& operator=(int x) {
            CHECK(x == 0 || x == 1);

            return Assign(static_cast<bool>(x));
        }

        /**
         * For b[i] = b[j].
         */
        Reference& operator=(const Reference& rhs) { return Assign(rhs); }

        /**
         * Flip the bit.
         */
        Reference& Flip() {
            *block_ ^= mask_;
            return *this;
        }

        /**
         * Set bit (set bit to one).
         */
        Reference& Set() {
            *block_ |= mask_;
            return *this;
        }

        /**
         * Reset bit (set bit to zero).
         */
        Reference& Reset() {
            *block_ &= ~mask_;
            return *this;
        }

     private:
        /**
         * Assignment.
         */
        Reference& Assign(bool x) {
            x ? *block_ |= mask_ : *block_ &= mask_;
            return *this;
        }

        Block* block_;
        Block mask_;
    };

public:
    /**
     * Construct DynamicBitset by specifing the size of bits.
     *
     * Initialize the object with 'val'.
     */
    explicit DynamicBitset(int size = 0, Block value = 0) {
        Resize(size, value);
    }

    /**
     * Construct DynamicBitset by given binary string.
     */
    explicit DynamicBitset(const std::string& string) {
        CHECK(string.size() <= INT_MAX)
                << "The size of the string exceeds INT_MAX.";

        int size = static_cast<int>(string.size());
        Resize(size);

        int n = 0;
        for (; ; size -= BITS_PER_BLOCK) {
            if (size < BITS_PER_BLOCK) break;

            bits_[n++] = StringToUInt(string.substr(size - BITS_PER_BLOCK,
                                                    BITS_PER_BLOCK));
        }
        if (size != 0) {
            bits_[n++] = StringToUInt(string.substr(0, size));
        }
    }

    /**
     * Construct DynamicBitset by given binary string.
     */
    explicit DynamicBitset(const char* str)
        : DynamicBitset(std::string(str)) {}

    /**
     * Return whether the bit at position pos is set (i.e., whether it is one).
     *
     * Unlike the access operator (operator[]), this function performs a range
     * check on 'pos' before retrieveing the bit value.
     */
    bool Test(int pos) const {
        CHECK(pos >= 0);
        CHECK(pos < n_bits_);

        return UncheckedTest(pos) == 1;
    }

    /**
     * Reset (to zero) all bits in the bitset.
     */
    DynamicBitset& Reset() {
        std::fill(bits_.begin(), bits_.end(), 0);

        return *this;
    }

    /**
     * Reset (to zero) the bit at position pos.
     */
    DynamicBitset& Reset(int pos) {
        CHECK(pos >= 0);
        CHECK(pos < n_bits_);

        bits_[BlockIndex(pos)] &= ~BitMask(pos);

        return *this;
    }

    /**
     * Set (to one) all bits in the bitset.
     */
    DynamicBitset& Set() {
        std::fill(bits_.begin(), bits_.end(), MASK);

        int t = n_bits_ % BITS_PER_BLOCK;
        if (t != 0) {
            bits_[n_blocks_ - 1] >>= BITS_PER_BLOCK - t;
        }

        return *this;
    }

    /**
     * Set (to one) the bit at position pos.
     */
    DynamicBitset& Set(int pos, bool value = true) {
        CHECK(pos >= 0);
        CHECK(pos < n_bits_);

        if (value)
            bits_[BlockIndex(pos)] |= BitMask(pos);
        else
            bits_[BlockIndex(pos)] &= ~BitMask(pos);

        return *this;
    }

    /**
     * Flip all bits in the bitset.
     */
    DynamicBitset& Flip() {
        for (int i = 0; i < n_blocks_; ++i)
            bits_[i] = ~bits_[i];

        return *this;
    }

    /**
     * Flip the bit at position pos.
     */
    DynamicBitset& Flip(int pos) {
        CHECK(pos >= 0);
        CHECK(pos < n_bits_);

        bits_[BlockIndex(pos)] ^= BitMask(pos);

        return *this;
    }

    /**
     * Return whether any of the bits is set (i.e., whether at least one bit in
     * the bitset is set to one).
     */
    bool Any() const {
        for (int i = 0; i < n_blocks_; ++i)
            if (bits_[i] != 0)
                return true;
        return false;
    }

    /**
     * Return whether none of the bits is set (i.e., whether all bits in the
     * bitset have a value of zero).
     */
    bool None() const {
        return !Any();
    }

    /**
     * Return whether or not all bits in the bitset are set (to one).
     */
    bool All() const {
        for (int i = 0; i < n_blocks_; ++i) {
            if ((bits_[i] & MASK) != MASK) return false;
        }
        return true;
    }

    /**
     * Return the number of bits in the bitset that are set (i.e., that have a
     * value of one).
     */
    int Count() const {
        int n = 0;

        for (int i = 0; i < n_blocks_; ++i) {
            n += CountOne(bits_[i]);
        }

        return n;
    }

    /**
     * Return number of bits.
     */
    int size() const {
        return n_bits_;
    }

    /**
     * Return true if n_bits == 0.
     */
    bool empty() const {
        return n_bits_ == 0;
    }

    /**
     * Return the number of blocks.
     */
    int n_blocks() const {
        return n_blocks_;
    }

    /**
     * Clear the dynamic bitset.
     */
    void clear() {
        n_bits_ = 0;
        bits_.clear();
    }

    /**
     * Resize the bits.
     */
    void Resize(int size, Block value = 0) {
        CHECK(size >= 0);

        n_bits_ = size;
        n_blocks_ = n_bits_ / BITS_PER_BLOCK +
                    static_cast<int>(n_bits_ % BITS_PER_BLOCK != 0);
        bits_.resize(n_blocks_, 0);

        // Zero out all bits at pos >= num_bits, if any.
        // Note that: num_bits == 0 implies value == 0.
        if (n_bits_ < BITS_PER_BLOCK) {
            Block mask = (Block(1) << n_bits_) - 1;
            value &= mask;
        }

        if (n_bits_ > 0) bits_[0] = value;
    }

    /**
     * Convert the bitset to binary string.
     */
    std::string ToString() const {
        std::string str(n_bits_, '0');

        for (int i = 0; i < n_bits_; ++i) {
            if (UncheckedTest(i) == 1)
                str[n_bits_ - 1 - i] = '1';
        }

        return str;
    }

    /**
     * This &= rhs.
     */
    DynamicBitset& operator &=(const DynamicBitset& rhs) {
        CHECK(n_bits_ == rhs.n_bits_);

        for (int i = 0; i < n_blocks_; ++i) {
            bits_[i] &= rhs.bits_[i];
        }

        return *this;
    }

    /**
     * This |= rhs.
     */
    DynamicBitset& operator |=(const DynamicBitset& rhs) {
        CHECK(n_bits_ == rhs.n_bits_);

        for (int i = 0; i < n_blocks_; ++i) {
            bits_[i] |= rhs.bits_[i];
        }

        return *this;
    }

    /**
     * This ^= rhs.
     */
    DynamicBitset& operator ^=(const DynamicBitset& rhs) {
        CHECK(n_bits_ == rhs.n_bits_);

        for (int i = 0; i < n_blocks_; ++i) {
            bits_[i] ^= rhs.bits_[i];
        }

        return *this;
    }

    /**
     * This <<= n.
     */
    DynamicBitset& operator <<=(int n) {
        CHECK(n >= 0);

        if (n >= n_bits_) return Reset();
        if (n == 0) return *this;

        int t1 = n / BITS_PER_BLOCK;
        int t2 = n % BITS_PER_BLOCK;

        std::copy_backward(bits_.begin(), bits_.begin() + n_blocks_ - t1,
                           bits_.end());
        std::fill_n(bits_.begin(), t1, 0);

        if (t2 != 0) {
            for (int i = n_blocks_ - 1; i > 0; --i) {
                bits_[i] = (bits_[i] << t2) |
                           (bits_[i - 1] >> (BITS_PER_BLOCK - t2));
            }
            bits_[0] <<= t2;
        }

        return *this;
    }

    /**
     * This = a >> n.
     */
    DynamicBitset& operator >>=(int n) {
        CHECK(n >= 0);

        if (n >= n_bits_) return Reset();
        if (n == 0) return *this;

        int t1 = n / BITS_PER_BLOCK;
        int t2 = n % BITS_PER_BLOCK;

        std::copy(bits_.begin() + t1, bits_.end(), bits_.begin());
        int size = n_blocks_ - t1;
        std::fill(bits_.begin() + size, bits_.end(), 0);

        if (t2 != 0) {
            for (int i = 0; i < size - 1; ++i) {
                bits_[i] = (bits_[i] >> t2) |
                           (bits_[i + 1] << (BITS_PER_BLOCK - t2));
            }
            bits_[size - 1] >>= t2;
        }

        return *this;
    }

    bool operator ==(const DynamicBitset& rhs) const {
        if (rhs.n_bits_ != n_bits_) return false;

        for (int i = 0; i < n_bits_; ++i) {
            if (bits_[i] != rhs.bits_[i]) return false;
        }

        return true;
    }

    bool operator !=(const DynamicBitset& rhs) const {
        return !(*this == rhs);
    }

    Reference operator[] (int pos) {
        return {&bits_[BlockIndex(pos)], pos % BITS_PER_BLOCK};
    }

    int operator[] (int pos) const {
        return UncheckedTest(pos);
    }

    friend DynamicBitset operator ~(const DynamicBitset& rhs) {
        DynamicBitset res(rhs);
        return res.Flip();
    }

    friend DynamicBitset operator &(const DynamicBitset& lhs,
                                    const DynamicBitset& rhs) {
        DynamicBitset res(lhs);
        return res &= rhs;
    }

    friend DynamicBitset operator |(const DynamicBitset& lhs,
                                    const DynamicBitset& rhs) {
        DynamicBitset res(lhs);
        return res |= rhs;
    }

    friend DynamicBitset operator ^(const DynamicBitset& lhs,
                                    const DynamicBitset& rhs) {
        DynamicBitset res(lhs);
        return res ^= rhs;
    }

    friend DynamicBitset operator <<(const  DynamicBitset& lhs, int n) {
        DynamicBitset res(lhs);
        return res <<= n;
    }

    friend DynamicBitset operator >>(const DynamicBitset& lhs, int n) {
        DynamicBitset res(lhs);
        return res >>= n;
    }

private:
    /**
     * Return the index of block.
     */
    static int BlockIndex(int pos) {
        return pos / BITS_PER_BLOCK;
    }

    /**
     * Return the bit mask.
     */
    static Block BitMask(int pos) {
        return Block(1) << (pos % BITS_PER_BLOCK);
    }

    /**
     * Convert a binary string to a block.
     */
    static Block StringToUInt(const std::string& str) {
        Block b = 0;

        for (char i : str) {
            CHECK(i == '0' || i == '1');

            b = (b << 1) + static_cast<Block>(i - '0');
        }

        return b;
    }

    /**
     * Return the bit at position pos.
     */
    int UncheckedTest(int pos) const {
        return static_cast<int>(bits_[BlockIndex(pos)] & BitMask(pos));
    }

    /**
     * Count the number of one in the unsigned 32-bit integer.
     */
    static int CountOne(uint32_t n) {
        n -= (n >> 1) & 0x55555555;
        n  = (n & 0x33333333) + ((n >> 2) & 0x33333333);
        n  = (n + (n >> 4)) & 0x0f0f0f0f;
        return (n * 0x01010101) >> 24;
    }

    int n_bits_ = 0;    // Number of bits.
    int n_blocks_ = 0;  // Number of blocks.
    Array<Block> bits_; // Bits stored as block.
};

} // namespace cl

#endif // CODELIBRARY_UTIL_SET_DYNAMIC_BITSET_H_
