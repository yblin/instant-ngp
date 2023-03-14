//
// Copyright 2015-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_STRING_STRING_CONSTANTS_H_
#define CODELIBRARY_STRING_STRING_CONSTANTS_H_

namespace cl {

/**
 * White space encoded in UTF16.
 */
static const wchar_t WHITESPACE_UTF16[] = {
    0x0009, // CHARACTER TABULATION
    0x000A, // LINE FEED (LF)
    0x000B, // LINE TABULATION
    0x000C, // FORM FEED (FF)
    0x000D, // CARRIAGE RETURN (CR)
    0x0020, // SPACE
    0x0085, // NEXT LINE (NEL)
    0x00A0, // NO-BREAK SPACE
    0x1680, // OGHAM SPACE MARK
    0x2000, // EN QUAD
    0x2001, // EM QUAD
    0x2002, // EN SPACE
    0x2003, // EM SPACE
    0x2004, // THREE-PER-EM SPACE
    0x2005, // FOUR-PER-EM SPACE
    0x2006, // SIX-PER-EM SPACE
    0x2007, // FIGURE SPACE
    0x2008, // PUNCTUATION SPACE
    0x2009, // THIN SPACE
    0x200A, // HAIR SPACE
    0x2028, // LINE SEPARATOR
    0x2029, // PARAGRAPH SEPARATOR
    0x202F, // NARROW NO-BREAK SPACE
    0x205F, // MEDIUM MATHEMATICAL SPACE
    0x3000, // IDEOGRAPHIC SPACE
    0
};

/**
 * White space encoded in ASCII.
 */
static const char WHITESPACE_ASCII[] = {
    0x09,    // CHARACTER TABULATION
    0x0A,    // LINE FEED (LF)
    0x0B,    // LINE TABULATION
    0x0C,    // FORM FEED (FF)
    0x0D,    // CARRIAGE RETURN (CR)
    0x20,    // SPACE
    0
};

} // namespace cl

#endif // CODELIBRARY_STRING_STRING_CONSTANTS_H_
