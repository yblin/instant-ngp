//
// Copyright 2015 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_STRING_STRING_SPLIT_TEST_H_
#define CODELIBRARY_TEST_STRING_STRING_SPLIT_TEST_H_

#include <string>

#include "codelibrary/base/testing.h"
#include "codelibrary/string/string_split.h"

namespace cl {
namespace test {

TEST(StringSplitTest, SplitWithOneCharacter) {
    Array<std::wstring> r;

    StringSplit(std::wstring(), L',', &r);
    ASSERT_EQ(r.size(), 0);

    StringSplit(L"a,b,c", L',', &r);
    ASSERT_EQ(r.size(), 3);
    ASSERT_EQ(r[0], L"a");
    ASSERT_EQ(r[1], L"b");
    ASSERT_EQ(r[2], L"c");

    StringSplit(L"a,,c", L',', &r);
    ASSERT_EQ(r.size(), 3);
    ASSERT_EQ(r[0], L"a");
    ASSERT_EQ(r[1], L"");
    ASSERT_EQ(r[2], L"c");

    StringSplit(L"foo", L'*', &r);
    ASSERT_EQ(r.size(), 1);
    ASSERT_EQ(r[0], L"foo");

    StringSplit(L"foo,", L',', &r);
    ASSERT_EQ(r.size(), 2);
    ASSERT_EQ(r[0], L"foo");
    ASSERT_EQ(r[1], L"");

    StringSplit(L",", L',', &r);
    ASSERT_EQ(r.size(), 2);
    ASSERT_EQ(r[0], L"");
    ASSERT_EQ(r[1], L"");

    StringSplit(L"\t\ta\t", L'\t', &r);
    ASSERT_EQ(r.size(), 4);
    ASSERT_EQ(r[0], L"");
    ASSERT_EQ(r[1], L"");
    ASSERT_EQ(r[2], L"a");
    ASSERT_EQ(r[3], L"");

    StringSplit(L"a\nb\tcc", L'\n', &r);
    ASSERT_EQ(r.size(), 2);
    ASSERT_EQ(r[0], L"a");
    ASSERT_EQ(r[1], L"b\tcc");
}

TEST(StringSplitTest, SplitEmptyString) {
    Array<std::string> results;
    StringSplit("", "DELIMITER", &results);
    ASSERT_EQ(results.size(), 1);
    ASSERT_EQ(results[0], "");
}

TEST(StringSplitTest, StringWithNoDelimiter) {
    Array<std::string> results;
    StringSplit("alongwordwithnodelimiter", "DELIMITER", &results);
    ASSERT_EQ(results.size(), 1);
    ASSERT_EQ(results[0], "alongwordwithnodelimiter");
}

TEST(StringSplitTest, LeadingDelimitersSkipped) {
    Array<std::string> results;
    StringSplit("DELIMITERDELIMITERDELIMITERoneDELIMITERtwoDELIMITERthree",
                "DELIMITER",
                &results);

    Array<std::string> ans = { "", "", "", "one", "two", "three" };
    ASSERT_EQ_RANGE(results.begin(), results.end(), ans.begin(), ans.end());
}

TEST(StringSplitTest, ConsecutiveDelimitersSkipped) {
    Array<std::string> results;
    StringSplit(
        "unoDELIMITERDELIMITERDELIMITERdosDELIMITERtresDELIMITERDELIMITERcua",
        "DELIMITER",
        &results);

    Array<std::string> ans = { "uno", "", "", "dos", "tres", "", "cua" };
    ASSERT_EQ_RANGE(results.begin(), results.end(), ans.begin(), ans.end());
}

TEST(StringSplitTest, TrailingDelimitersSkipped) {
    Array<std::string> results;
    StringSplit(
        "unDELIMITERdeuxDELIMITERtroisDELIMITERquatreDELIMITERDELIMITER",
        "DELIMITER",
        &results);

    Array<std::string> ans = { "un", "deux", "trois", "quatre", "", "" };
    ASSERT_EQ_RANGE(results.begin(), results.end(), ans.begin(), ans.end());
}

TEST(StringSplitTest, SplitStringWithWhitespace) {
    struct TestData {
        std::string input;
        int expected_result_count;
        std::string output1;
        std::string output2;
    };

    Array<TestData> data = {
        { "a",       1, "a",  ""   },
        { " ",       0, "",   ""   },
        { " a",      1, "a",  ""   },
        { " ab ",    1, "ab", ""   },
        { " ab c",   2, "ab", "c"  },
        { " ab c ",  2, "ab", "c"  },
        { " ab cd",  2, "ab", "cd" },
        { " ab cd ", 2, "ab", "cd" },
        { " \ta\t",  1, "a",  ""   },
        { " b\ta\t", 2, "b",  "a"  },
        { " b\tat",  2, "b",  "at" },
        { "b\tat",   2, "b",  "at" },
        { "b\t at",  2, "b",  "at" }
    };

    for (auto & i : data) {
        Array<std::string> results;
        StringSplit(i.input, &results);
        ASSERT_EQ(i.expected_result_count, results.size());
        if (i.expected_result_count > 0)
            ASSERT_EQ(i.output1, results[0]);
        if (i.expected_result_count > 1)
            ASSERT_EQ(i.output2, results[1]);
    }
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_STRING_STRING_SPLIT_TEST_H_
