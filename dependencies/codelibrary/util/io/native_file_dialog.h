//
// Copyright 2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_
#define CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_

#include <string>

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "third_party/native_file_dialog/nfd.h"

#include "codelibrary/base/log.h"

namespace cl {
namespace io {

/**
 * Get single open file by native open dialog.
 */
inline std::string OpenFileDialog(const std::string& default_path = "",
                                  const std::string& filters = "") {
    nfdchar_t* out_path = nullptr;
    nfdresult_t result = NFD_OpenDialog(filters.c_str(), default_path.c_str(),
                                        &out_path);
    if (result == NFD_OKAY) {
        return std::string(out_path);
    }

    if (result == NFD_CANCEL) {
        return "";
    }

    LOG(WRONG) << NFD_GetError();
    return "";
}

/**
 * Save file by native open dialog.
 */
inline std::string SaveFileDialog(const std::string& default_path = "",
                                  const std::string& filters = "") {
    nfdchar_t* out_path = nullptr;
    nfdresult_t result = NFD_SaveDialog(filters.c_str(), default_path.c_str(),
                                        &out_path);
    if (result == NFD_OKAY) {
        return std::string(out_path);
    }

    if (result == NFD_CANCEL) {
        return "";
    }

    LOG(WRONG) << NFD_GetError();
    return "";
}

} // namespace io
} // namespace cl

#endif // CODELIBRARY_UTIL_IO_NATIVE_FILE_DIALOG_H_
