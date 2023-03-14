//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_BASE_MESSAGE_H_
#define CODELIBRARY_BASE_MESSAGE_H_

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <numeric>
#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace cl {

namespace internal {

/**
 * This function is used for 'Message' to print the objects without << operator.
 */
template <typename Char, typename CharTraits, typename T>
std::basic_ostream<Char, CharTraits>& operator<<(
    std::basic_ostream<Char, CharTraits>& os, const T& object) {
    auto bytes = reinterpret_cast<const unsigned char*>(&object);

    int count = sizeof(object);

    // Tell the user how big the object is.
    os << count << "-byte object <";

    int i = 0;
    for (; i < std::min(128, count); ++i) {
        if (i != 0) {
            // Organizes the bytes into groups of 2 for easy parsing by human.
            if ((i % 2) == 0)
                os << ' ';
            else
                os << '-';
        }
        int t = static_cast<int>(bytes[i]);
        if (t < 16) {
            os << std::hex << 0 << t;
        } else {
            os << std::hex << t;
        }
    }
    if (i < count) os << " ... ";
    os << ">";

    return os;
}

} // namespace internal

/**
 * Message class stream information for various objects.
 * 
 * Why need Message?
 *  1. STL formating library do not support pointers.
 *  2. It is easier to use.
 *  3. It supports aligned formating.
 *
 * Sample usage;
 *
 *   bool flag = true;
 *   Message foo;
 *   foo << "flag" << " = " << flag;
 *   std::cout << foo;
 *
 * The above code will print "flag = true".
 */
class Message {
    struct MaxElementsFlag {
        MaxElementsFlag(int n)
            : max_elements(n) {}
        int max_elements;
    };

public:
    /**
     * Constructs an empty Message.
     */
    Message() {
        stream_ << std::setprecision(precision_);
    }

    /**
     * Copy constructor.
     */
    Message(const Message& message) {
        stream_ << message.ToString();
        precision_ = message.precision_;
        stream_ << std::setprecision(message.precision_);
        max_elements_ = message.max_elements_;
        screen_width_ = message.screen_width_;
    }

    /**
     * Construct a message from sequence [first, last) in aligned manner.
     *
     * This function is especially useful for printing matices and vectors.
     * It can greatly improve human readability.
     *
     * For example,
     *   [ 1,  2,  3,
     *     4,  5,  6,
     *     7,  8,  9,
     *    10, 11, 12]
     *
     * is much better than
     *   [1, 2, 3,
     *   4, 5, 6,
     *   7, 8, 9,
     *   10, 11, 12]
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message(Iterator first, Iterator last) {
        stream_ << std::setprecision(precision_);
        Append(first, last);
    }

    /**
     * Construct a message from N-D array in aligned manner.
     */
    template <typename Iterator,
          typename = typename std::enable_if<std::is_convertible<
                     typename std::iterator_traits<Iterator>::iterator_category,
                              std::input_iterator_tag>::value>::type>
    Message(Iterator first, Iterator last, const std::vector<int>& shape) {
        stream_ << std::setprecision(precision_);
        Append(first, last, shape);
    }

    /**
     * Construct a message of an object.
     */
    template <typename T>
    Message(const T& object) {
        stream_ << std::setprecision(precision_);
        Append(object);
    }

    /**
     * Construct a message of a list.
     */
    template <typename T, typename... Args>
    Message(const T& t1, const Args&... t2) {
        stream_ << std::setprecision(precision_);
        Append(t1, t2...);
    }

    /**
     * Copy constructor.
     */
    Message& operator =(const Message& message) {
        stream_.clear();
        stream_ << message.ToString();
        precision_ = message.precision_;
        stream_ << std::setprecision(message.precision_);
        max_elements_ = message.max_elements_;
        screen_width_ = message.screen_width_;
        return *this;
    }

    /**
     * Check if the message is multi-line or single line.
     */
    bool IsMultiLine() const {
        return ToString().find('\n') != std::string::npos;
    }

    /**
     * Append STL pair to the message.
     */
    template <class T1, class T2>
    Message& Append(const std::pair<T1, T2>& p) {
        Append(p.first, p.second);
        return *this;
    }

    /**
     * Append variable object to the message.
     */
    template <typename T>
    Message& Append(const T& object) {
        using internal::operator <<;
        stream_ << object;
        return *this;
    }

    /**
     * Instead of 1/0, we want to see true/false for bool values.
     */
    Message& Append(bool b) {
        stream_ << (b ? "true" : "false");
        return *this;
    }

    /**
     * uint8_t needs special treatment.
     */
    Message& Append(uint8_t v) {
        stream_ << static_cast<int>(v);
        return *this;
    }

    /**
     * Return the richer information for pointer.
     */
    template <typename T>
    Message& Append(T* pointer) {
        if (pointer == nullptr) {
            stream_ <<  "(nullptr)";
        } else {
            stream_ << std::hex << reinterpret_cast<size_t>(pointer);
        }
        return *this;
    }

    /**
     * Return the richer information for constant pointer.
     */
    template <typename T>
    Message& Append(const T* pointer) {
        if (pointer == nullptr) {
            stream_ <<  "(nullptr)";
        } else {
            stream_ << std::hex << reinterpret_cast<size_t>(pointer);
        }
        return *this;
    }

    /**
     * When the input value is a std::string or std::wstring object, each NULL
     * character in it is replaced with "\\0".
     */
    Message& Append(const std::string& data) {
        std::string str;
        for (size_t i = 0; i < data.size(); ++i) {
            if (data[i] == '\0') {
                str += "\\0";
            } else {
                str += data[i];
            }
        }
        stream_ << str;
        return *this;
    }
    Message& Append(char* data) {
        return Append(std::string(data));
    }
    Message& Append(const char* data) {
        return Append(std::string(data));
    }

    /**
     * Convert a wide string to a narrow string in UTF-8 encoding.
     * The wide string is assumed to have the following encoding:
     *   UTF-16 if sizeof(wchar_t) == 2 (on Windows, Cygwin, Symbian OS)
     *   UTF-32 if sizeof(wchar_t) == 4 (on Linux)
     *
     * Note If the string contains code points that are not valid Unicode code
     * points (i.e. outside of Unicode range U+0 to U+10FFFF) they will be
     * output as '(Invalid Unicode 0xXXXXXXXX)'.
     * If the string is in UTF16 encoding and contains invalid UTF-16 surrogate
     * pairs, values in those pairs will be encoded as individual Unicode
     * characters from Basic Normal Plane.
     */
    Message& Append(const std::wstring& str) {
        size_t len = str.size();

        std::string result;
        for (size_t i = 0; i < len; ++i) {
            uint32_t code_point;

            if (str[i] == L'\0') break;

            if (i + 1 < len && sizeof(wchar_t) == 2 &&
                (str[i] & 0xFC00) == 0xD800 &&
                (str[i + 1] & 0xFC00) == 0xDC00) {
                const uint32_t mask = (1 << 10) - 1;
                code_point = (((str[i] & mask) << 10) |
                              (str[i + 1] & mask)) + 0x10000;
                ++i;
            } else {
                code_point = static_cast<uint32_t>(str[i]);
            }

            result += UnicodeCodePointToUtf8(code_point);
        }

        stream_ << result;
        return *this;
    }
    Message& Append(wchar_t* data) {
        return Append(std::wstring(data));
    }
    Message& Append(const wchar_t* data) {
        return Append(std::wstring(data));
    }
    Message& Append(const Message& msg) {
        return Append(msg.ToString());
    }

    /**
     * Append a sequence [first, last) to the message in aligned manner.
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message& Append(Iterator first, Iterator last) {
        std::vector<std::string> terms;
        size_t width = GetTerms(first, last, &terms);
        if (terms.empty()) {
            stream_ << "[]";
            return *this;
        }

        // Check if the sequence can be shown in one line.
        size_t screen_w = screen_width_;
        if (screen_w == 0) screen_w = std::numeric_limits<size_t>::max();
        size_t length = 0;
        for (const std::string& term : terms)
            length += term.size() + 2;
        if (length <= screen_w) {
            stream_ << "[";
            this->Join(terms, ", ");
            stream_ << "]";
            return *this;
        }

        // Otherwise, we split terms into multi lines.
        std::vector<std::string> lines;
        Split(terms, width, &lines);
        Join(lines, "\n");

        return *this;
    }

    /**
     * Append an N-dimensional array to the message in aligned manner.
     */
    template <typename Iterator,
        typename = typename std::enable_if<std::is_convertible<
                   typename std::iterator_traits<Iterator>::iterator_category,
                            std::input_iterator_tag>::value>::type>
    Message& Append(Iterator first, Iterator last,
                    const std::vector<int>& shape) {
        if (shape.empty()) return *this;

        auto n = std::distance(first, last);
        for (int s : shape) {
            assert(s != 0);
            assert(n % s == 0);
            n /= s;
        }
        assert(n == 1 && "The shape is not matched to [first, last).");

        std::vector<int> strides(shape.size());
        strides.back() = 1;
        for (size_t i = shape.size() - 1; i > 0; --i) {
            strides[i - 1] = strides[i] * shape[i];
        }

        size_t width = ComputeWidth(first, shape, strides, 0);
        return AppendND(first, shape, strides, 0, width);
    }

    /**
     * Append a tuple to the message.
     */
    template <typename T, typename... Args>
    Message& Append(const T& t1, const Args&... t2) {
        Append("(");
        AppendUnpack(t1, t2...);
        Append(")");
        return *this;
    }

    /**
     * Allow that Message() << "fdsa", 12.
     */
    template <typename T>
    Message& operator,(const T& t1) {
        Append(" ");
        Append(t1);
        return *this;
    }

    /**
     * Using opeartor << Message::SetMaxElements(n) to set the max elements for
     * printing.
     */
    static MaxElementsFlag MaxElements(int n) {
        assert(n >= 0);

        return MaxElementsFlag(n);
    }

    /**
     * Clear the message.
     */
    void clear() {
        stream_.str("");
    }

    bool empty() const {
        return stream_.rdbuf()->in_avail() == 0;
    }

    /**
     * Return the internal stream data.
     */
    std::string ToString() const {
        return stream_.str();
    }

    /**
     * Return the maximum elements in the sequence will be recorded.
     */
    int max_elements() const {
        return max_elements_;
    }

    /**
     * Return the current screen width setting.
     */
    int screen_width() const {
        return screen_width_;
    }

    /**
     * Return the current precision setting.
     */
    int precision() const {
        return precision_;
    }

    /**
     * Set the maximum number of elements in sequence that need to be recorded.
     *
     * max_elements = 0 means that all elements will be recorded. The default
     * value is 10.
     */
    void set_max_elements(int max_elements) {
        assert(max_elements >= 0);

        max_elements_ = max_elements;
    }

    /**
     * Set the width of the screen (the maximum characters in one line).
     * screen_width = 0 means that the screen width is infinity. The default
     * value is 120.
     */
    void set_screen_width(int screen_width) {
        assert(screen_width > 1);

        screen_width_ = screen_width;
    }

    /**
     * Set the precision for double and float.
     * By default, we want there to be enough precision when printing a double
     * to Message.
     */
    void set_precision(int precision) {
        assert(precision > 0);

        stream_ << std::setprecision(precision);
        precision_ = precision;
    }

    template <typename T>
    Message& operator <<(const T& v)                 { return Append(v); }
    Message& operator <<(bool v)                     { return Append(v); }
    template <typename T>
    Message& operator <<(T* v)                       { return Append(v); }
    template <typename T>
    Message& operator <<(const T* v)                 { return Append(v); }
    Message& operator <<(const std::string& v)       { return Append(v); }
    Message& operator <<(char* v)                    { return Append(v); }
    Message& operator <<(const char* v)              { return Append(v); }
    Message& operator <<(const std::wstring& v)      { return Append(v); }
    Message& operator <<(wchar_t* v)                 { return Append(v); }
    Message& operator <<(const wchar_t* v)           { return Append(v); }
    template <typename T1, typename T2>
    Message& operator <<(const std::pair<T1, T2>& v) { return Append(v); }
    Message& operator <<(const Message& v)           { return Append(v); }

    Message& operator <<(const MaxElementsFlag& m) {
        max_elements_ = m.max_elements;
        return *this;
    }

    friend std::ostream& operator <<(std::ostream& os, const Message& msg) {
        os << msg.ToString();
        return os;
    }

private:
    /**
     * Initialize the message of a ND array recursively.
     */
    template <typename Iterator>
    Message& AppendND(Iterator first,
                      const std::vector<int>& shape,
                      const std::vector<int>& strides,
                      size_t depth,
                      size_t width) {
        if (depth == shape.size()) return *this;

        int n = shape[depth];
        if (depth + 1 == shape.size()) {
            // Info of 1D sequence.
            screen_width_ -= static_cast<int>(depth + depth);
            std::vector<std::string> terms;
            Iterator p = first;
            std::advance(p, n);
            GetTerms(first, p, &terms);

            std::string space(depth, ' ');
            std::vector<std::string> lines;
            Split(terms, width, &lines);
            for (size_t i = 1; i < lines.size(); ++i) {
                lines[i] = space + lines[i];
            }
            screen_width_ += static_cast<int>(depth + depth);
            Join(lines, "\n");
            return *this;
        }

        std::string space(depth + 1, ' ');
        stream_ << "[";

        if (n <= max_elements_ || max_elements_ == 0) {
            Iterator p = first;
            for (int i = 0; i < n; ++i) {
                if (i != 0) stream_ << space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                if (i + 1 < n) {
                    stream_ << ",\n";
                    if (depth + 2 < shape.size()) stream_ << "\n";
                }
            }
        } else {
            Iterator p = first;
            for (int i = 0; i < (max_elements_ + 1) / 2; ++i) {
                if (i != 0) stream_ << space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                stream_ << ",\n";
                if (depth + 2 < shape.size()) stream_ << '\n';
            }

            stream_ << space;
            stream_ << "...,\n";
            if (depth + 2 < shape.size()) stream_ << "\n";

            p = first;
            std::advance(p, strides[depth] * (n - max_elements_ / 2));
            for (int i = n - max_elements_ / 2; i < n; ++i) {
                stream_ << space;
                AppendND(p, shape, strides, depth + 1, width);
                std::advance(p, strides[depth]);
                if (i + 1 < n) {
                    stream_ << ",\n";
                    if (depth + 2 < shape.size()) stream_ << '\n';
                }
            }
        }

        stream_ << "]";
        return *this;
    }

    /**
     * Append a variable number of objects to the message.
     */
    template <typename T, typename... Args>
    Message& AppendUnpack(const T& first_arg, const Args&... args) {
        this->Append(first_arg);
        stream_ << ", ";
        return AppendUnpack(args...);
    }
    template <typename T>
    Message& AppendUnpack(const T& last_arg) {
        this->Append(last_arg);
        return *this;
    }

    /**
     * Get the string message of various object.
     */
    template <typename T>
    std::string Info(const T& t) const {
        Message msg;
        msg.set_precision(precision_);
        msg.set_max_elements(max_elements_);
        msg.set_screen_width(screen_width_);
        msg.Append(t);
        return msg.ToString();
    }

    /**
     * Convert a sequence [first, last) to string terms.
     *
     * Return the max width in terms.
     */
    template <typename Iterator>
    size_t GetTerms(Iterator first, Iterator last,
                    std::vector<std::string>* terms) const {
        terms->clear();
        if (first == last) return 0;

        auto n = std::distance(first, last);

        size_t width = 0;
        if (max_elements_ >= n || max_elements_ == 0) {
            for (Iterator p = first; p != last; ++p) {
                terms->push_back(Info(*p));
                width = std::max(width, terms->back().size());
            }
        } else {
            Iterator p = first;
            for (int k = 0; k < (max_elements_ + 1) / 2; ++k) {
                terms->push_back(Info(*p++));
                width = std::max(width, terms->back().size());
            }
            terms->push_back("...");
            std::advance(p, n - max_elements_);
            for (; p != last; ++p) {
                terms->push_back(Info(*p));
                width = std::max(width, terms->back().size());
            }
        }
        return width;
    }

    /**
     * Recursively find a suitable width to stream the N-D array .
     */
    template <typename Iterator>
    size_t ComputeWidth(Iterator first,
                        const std::vector<int>& shape,
                        const std::vector<int>& strides,
                        size_t depth) const {
        if (depth == shape.size()) return 0;

        int n = shape[depth];
        if (depth + 1 == shape.size()) {
            std::vector<std::string> terms;
            Iterator last = first;
            std::advance(last, n);
            return GetTerms(first, last, &terms);
        }

        size_t width = 0;
        if (n <= max_elements_ || max_elements_ == 0) {
            Iterator p = first;
            for (int i = 0; i < n; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
        } else {
            Iterator p = first;
            for (int i = 0; i < (max_elements_ + 1) / 2; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
            p = first;
            std::advance(p, strides[depth] * (n - max_elements_ / 2));
            for (int i = n - max_elements_ / 2; i < n; ++i) {
                size_t w = ComputeWidth(p, shape, strides, depth + 1);
                std::advance(p, strides[depth]);
                width = std::max(width, w);
            }
        }

        return width;
    }

    /**
     * Concatenating a set of strings with a delimiter and append it to the
     * message.
     */
    void Join(const std::vector<std::string>& strs, const char* delimiter) {
        if (strs.empty()) return;

        stream_ << strs[0];
        for (size_t i = 1; i < strs.size(); ++i) {
            stream_ << delimiter + strs[i];
        }
    }

    /**
     * Split terms into multi-lines with the specific width.
     */
    void Split(const std::vector<std::string>& terms,
               size_t width,
               std::vector<std::string>* lines) const {
        lines->clear();
        if (terms.empty()) {
            lines->push_back("[]");
            return;
        }

        size_t screen_w = screen_width_ - 1;
        if (screen_w == 0) screen_w = std::numeric_limits<size_t>::max();

        std::string str = "[";
        size_t leading = 1;

        for (size_t i = 0; i < terms.size(); ++i) {
            std::string term = terms[i];
            if (term.size() < width) {
                std::string space(width - term.size(), ' ');
                term = space + term;
            } else if (term.size() > width) {
                size_t n = width;
                while (n < term.size()) n += 2 + width;
                n -= term.size();
                std::string left_space(n / 2, ' ');
                std::string right_space(n - n / 2, ' ');
                term = left_space + term + right_space;
            }

            if (leading + term.size() + 2 <= screen_w) {
                str += term;
                leading += term.size();
            } else {
                if (!str.empty()) lines->push_back(str);
                str = " " + term;
                leading = term.size() + 1;
            }

            if (i + 1 < terms.size()) {
                str += ", ";
                leading += 2;
            } else {
                str += "]";
                lines->push_back(str);
            }
        }
    }

    /**
     * Convert a Unicode code point to a narrow string in UTF-8 encoding.
     *
     * code_point parameter is of unsigned 32-bit integer type because wchar_t
     * may not be wide enough to contain a code point.
     *
     * A Unicode code-point can have up to 21 bits and is encoded in UTF-8 like
     * this:
     *
     * Code-point length   Encoding
     *   0 -  7 bits       0xxxxxxx
     *   8 - 11 bits       110xxxxx 10xxxxxx
     *  12 - 16 bits       1110xxxx 10xxxxxx 10xxxxxx
     *  17 - 21 bits       11110xxx 10xxxxxx 10xxxxxx 10xxxxxx
     *
     * If the code_point is not a valid Unicode code point
     * (i.e. outside of Unicode range U+0 to U+10FFFF) it will be converted to
     * "(Invalid Unicode 0xXXXXXXXX)".
     */
    static std::string UnicodeCodePointToUtf8(uint32_t c) {
        // The maximum code-point a one-byte UTF-8 sequence can represent.
        const uint32_t max_code_point1 = (static_cast<uint32_t>(1) << 7) - 1;

        // The maximum code-point a two-byte UTF-8 sequence can represent.
        const uint32_t max_code_point2 = (static_cast<uint32_t>(1) << 11) - 1;

        // The maximum code-point a three-byte UTF-8 sequence can represent.
        const uint32_t max_code_point3 = (static_cast<uint32_t>(1) << 16) - 1;

        // The maximum code-point a four-byte UTF-8 sequence can represent.
        const uint32_t max_code_point4 = (static_cast<uint32_t>(1) << 21) - 1;

        if (c > max_code_point4) {
            std::stringstream ss;
            ss << std::hex << std::uppercase << c;
            return "(Invalid Unicode 0x" + ss.str() + ")";
        }

        char str[5]; // Big enough for the largest valid code point.
        if (c <= max_code_point1) {
            str[1] = '\0';
            str[0] = static_cast<char>(c);                         // 0xxxxxxx
        } else if (c <= max_code_point2) {
            str[2] = '\0';
            str[1] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[0] = static_cast<char>(0xC0 | (c >> 6));           // 110xxxxx
        } else if (c <= max_code_point3) {
            str[3] = '\0';
            str[2] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[1] = static_cast<char>(0x80 | ((c >> 6) & 0x3F));  // 10xxxxxx
            str[0] = static_cast<char>(0xE0 | (c >> 12));          // 1110xxxx
        } else {
            str[4] = '\0';
            str[3] = static_cast<char>(0x80 | (c & 0x3F));         // 10xxxxxx
            str[2] = static_cast<char>(0x80 | ((c >> 6) & 0x3F));  // 10xxxxxx
            str[1] = static_cast<char>(0x80 | ((c >> 12) & 0x3F)); // 10xxxxxx
            str[0] = static_cast<char>(0xF0 | (c >> 18));          // 11110xxx
        }

        return str;
    }

    // The maximum number of elements in a sequence that need to be recorded in
    // the message.
    // max_elements = 0 means that all elements will be recorded.
    int max_elements_ = 10;

    // The width of the screen (the maximum characters in one line).
    // screen_width = 0 means that the screen width is infinity.
    int screen_width_ = 120;

    // Precision for double and float.
    // By default, we want there to be enough precision when printing a double
    // to Message.
    int precision_ = std::numeric_limits<double>::digits10 + 2;

    // Stream to hold the text data.
    std::stringstream stream_;
};

} // namespace cl

#endif // CODELIBRARY_BASE_MESSAGE_H_
