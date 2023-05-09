//
// Copyright 2018-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_BASE_ARRAY_TEST_H_
#define CODELIBRARY_TEST_BASE_ARRAY_TEST_H_

#include <vector>

#include "codelibrary/base/array.h"
#include "codelibrary/base/timer.h"
#include "codelibrary/base/testing.h"

namespace cl {
namespace test {

TEST(ArrayTest, PushBackPerformance) {
    const int n_test = 11;
    int n[]     = { 1, 2, 4, 8, 10, 100, 1000, 10000, 100000, 1000000,
                    10000000 };
    int tests[] = { 1000000, 1000000, 1000000, 1000000, 1000000, 100000, 10000,
                    1000, 1000, 50, 5 };

    printf("\n");
    printf("        n    STL vector     Array\n");

    for (int i = 0; i < n_test; ++i) {
        Timer timer1;
        timer1.Start();
        for (int j = 0; j < tests[i]; ++j) {
            std::vector<int> x;
            for (int k = 0; k < n[i]; ++k) {
                x.push_back(k);
            }
        }
        timer1.Stop();

        Timer timer2;
        timer2.Start();
        for (int j = 0; j < tests[i]; ++j) {
            Array<int> x;
            for (int k = 0; k < n[i]; ++k) {
                x.push_back(k);
            }
        }
        timer2.Stop();

        printf("%9d %11s %11s\n", n[i], timer1.average_time(tests[i]).c_str(),
               timer2.average_time(tests[i]).c_str());
    }
    printf("\n");
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_BASE_ARRAY_TEST_H_
