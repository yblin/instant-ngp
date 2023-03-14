//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_STRING_TRIM_H_
#define CODELIBRARY_STRING_STRING_TRIM_H_

#include <cstring>
#include <string>

#include "codelibrary/string/string_constants.h"

namespace cl {

/**
 * Remove characters in 'trim_chars' from the beginning and end of 'str', and
 * return the trimed string.
 */
template <typename String>
String StringTrim(const String& str, const String& trim_chars) {
    const size_t first_good_char = str.find_first_not_of(trim_chars);
    const size_t last_good_char = str.find_last_not_of(trim_chars);

    if (str.empty() || first_good_char == String::npos ||
        last_good_char == String::npos) {
        return String();
    } else {
        return str.substr(first_good_char,
                          last_good_char - first_good_char + 1);
    }
}

/**
 * Trim 'str' with 'trim_chars'. Special treat for wchar_t*.
 */
inline std::wstring StringTrim(const wchar_t* str, const wchar_t* trim_chars) {
    return StringTrim(std::wstring(str, str + wcslen(str)),
                      std::wstring(trim_chars,
                                   trim_chars + wcslen(trim_chars)));
}

/**
 * Trim 'str' with whitespace.
 */
inline std::wstring StringTrim(const wchar_t* str) {
    int size = sizeof(WHITESPACE_UTF16) / sizeof(wchar_t);
    std::wstring trimchars(WHITESPACE_UTF16 + 0, WHITESPACE_UTF16 + size);
    return StringTrim(std::wstring(str, str + wcslen(str)), trimchars);
}

/**
 * Trim 'str' with 'trim_chars'. Special treat for char*.
 */
inline std::string StringTrim(const char* str, const char* trim_chars) {
    return StringTrim(std::string(str, str + strlen(str)),
                      std::string(trim_chars, trim_chars + strlen(trim_chars)));
}

/**
 * Trim 'str' with whitespace.
 */
inline std::string StringTrim(const char* str) {
    int size = sizeof(WHITESPACE_ASCII) / sizeof(char);
    std::string trim_chars(WHITESPACE_ASCII + 0, WHITESPACE_ASCII + size);
    return StringTrim(std::string(str, str + strlen(str)), trim_chars);
}
inline std::string StringTrim(const std::string& str) {
    return StringTrim(str.c_str());
}

} // namespace cl

#endif // CODELIBRARY_STRING_STRING_TRIM_H_
