//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_CLAMP_H_
#define CODELIBRARY_BASE_CLAMP_H_

namespace cl {

/**
 * Identical to c++11's std::clamp().
 */
template <typename T>
const T& Clamp(const T& v, const T& lo, const T& hi) {
    return v < lo ? lo : hi < v ? hi : v;
}
template <class T, class Compare>
const T& Clamp( const T& v, const T& lo, const T& hi, Compare comp) {
    return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

} // namespace cl

#endif // CODELIBRARY_BASE_CLAMP_H_
