//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_STRING_JOIN_H_
#define CODELIBRARY_STRING_STRING_JOIN_H_

#include "codelibrary/base/array.h"

namespace cl {

/**
 * StringJoin() is a counterpart to StringSplit(). It joins strings from a
 * 'list' by adding user defined separator.
 */
template <typename String>
void StringJoin(const Array<String>& list, const typename String::value_type& c,
                String* result) {
    CHECK(result);

    result->clear();
    for (int i = 0; i + 1 < list.size(); ++i) {
        *result += list[i] + c;
    }
    if (!list.empty()) *result += list.back();
}

} // namespace cl

#endif // CODELIBRARY_STRING_STRING_JOIN_H_
