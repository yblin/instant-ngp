//
// Copyright 2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_ANSI_H_
#define CODELIBRARY_BASE_ANSI_H_

#include <string>

#ifdef _WIN32
#   ifndef NOMINMAX
#   define NOMINMAX
#   endif
#   ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#   define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#   endif
#   include <Windows.h>
#else
#   include <sys/ioctl.h>
#   include <unistd.h>
#endif

namespace cl {
namespace ansi {

/**
 * Support ANSI escape code?
 * ANSI escape sequences are a standard for in-band signaling to control cursor
 * location, color, font styling, and other options on terminals.
 */
inline bool SupportAnsiEscapeCode() {
    char* no_color_env = getenv("NO_COLOR");
    if (no_color_env && no_color_env[0] != '\0') {
        return false;
    }

#ifdef _WIN32
    // Set output mode to handle virtual terminal sequences
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) {
        return false;
    }
    DWORD dw_mode = 0;
    if (!GetConsoleMode(hOut, &dw_mode)) {
        return false;
    }
    dw_mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    if (!SetConsoleMode(hOut, dw_mode)) {
        return false;
    }
#endif
    return true;
}

} // namespace ansi
} // namespace cl

#endif // CODELIBRARY_BASE_ANSI_H_
