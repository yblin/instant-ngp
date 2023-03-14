//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_FILE_UTIL_H_
#define CODELIBRARY_UTIL_IO_FILE_UTIL_H_

#include <fstream>
#include <string>

#include "codelibrary/base/array.h"

namespace cl {
namespace file_util {

/**
 * Get file suffix and return it in lowercase.
 */
inline std::string GetSuffix(const std::string& filename) {
    std::size_t index = filename.find_last_of('.');
    if (index == std::string::npos) return "";
    std::string suffix = filename.substr(index + 1);
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);
    return suffix;
}

/**
 * Get file basename.
 */
inline std::string GetBasename(const std::string& filename) {
    return filename.substr(filename.find_last_of("/\\") + 1);
}

/**
 * Get the bytes of the file.
 */
inline size_t GetFileBytes(const std::string& filename) {
    FILE* file = std::fopen(filename.c_str(), "rb");
    std::fseek(file, 0, SEEK_END);
    size_t file_bytes = std::ftell(file);
    std::fclose(file);
    return file_bytes;
}

/**
 * Return true if the file is exist.
 */
inline bool IsFileExist(const std::string& filename) {
    std::ifstream f(filename.c_str());
    return f.good();
}

} // namespace file_util
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_FILE_UTIL_H_
