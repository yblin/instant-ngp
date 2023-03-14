//
// Copyright 2017-2023 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_TEST_GRPAH_FLOW_PERFORMANCE_TEST_H_
#define CODELIBRARY_TEST_GRPAH_FLOW_PERFORMANCE_TEST_H_

#include <random>

#include "codelibrary/base/testing.h"
#include "codelibrary/base/timer.h"
#include "codelibrary/graph/flow/boykov_kolmogorov_max_flow.h"
#include "codelibrary/graph/flow/improved_sap_max_flow.h"
#include "codelibrary/graph/flow/push_relable_max_flow.h"

namespace cl {
namespace test {

/**
 * Genereate the flow problems to test the performance of max flow algorithms.
 */
class MaxFlowPerformanceTest : public Test {
    using Edge = Graph::Edge;

protected:
    MaxFlowPerformanceTest() {
        capacity_ = graph_.AddEdgeProperty<int>("capacity");
        flow_ = graph_.AddEdgeProperty<int>("flow");
    }

    /**
     * Generate a random graph with n vertices and m edges.
     */
    void RandomGraph(int n, int m) {
        graph_.clear();
        graph_.Resize(n);

        std::mt19937 random;
        std::uniform_int_distribution<int> uniform_cap(1, 10);
        std::uniform_int_distribution<int> uniform_v(0, n - 1);

        for (int i = 0; i < m; ++i) {
            int a = uniform_v(random);
            int b = uniform_v(random);
            int c = uniform_cap(random);
            Edge* e = graph_.InsertTwoWayEdge(a, b);
            capacity_[e] = c;
            capacity_[e->twin()] = 0;
        }
    }

    /**
     * Generate a random dense graph with n vertices and n^2 edges.
     */
    void RandomGraph(int n) {
        graph_.clear();
        graph_.Resize(n);

        std::mt19937 random;
        std::uniform_int_distribution<int> uniform_cap(0, 10);

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                int c = uniform_cap(random);
                if (c != 0) {
                    Edge* e = graph_.InsertTwoWayEdge(i, j);
                    capacity_[e] = capacity_[e->twin()] = c;
                }
            }
        }
    }

    virtual void Finish() override {
        graph_.clear();
    }

    Graph graph_;
    Graph::EdgeProperty<int> capacity_;
    Graph::EdgeProperty<int> flow_;
};

TEST_F(MaxFlowPerformanceTest, DenseGraph) {
    int n_tests = 10;
    int v[]     = { 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
    int tests[] = { 100000, 10000, 10000, 1000, 1000, 1000, 100, 10, 5, 5 };

    graph::ImprovedSAPMaxFlow<int> isap;
    graph::PushRelableMaxFlow<int> push_relabel;
    graph::BoykovKolmogorovMaxFlow<int> bk;

    printf("\n");
    printf("    |V|     Imporeved SAP    Push Relable    Boykov Kolmogorov\n");
    printf("--------------------------------------------------------------\n");
    for (int i = 0; i < n_tests; ++i) {
        int n = v[i];
        RandomGraph(n);

        Timer timer1, timer2, timer3;
        timer1.Start();
        for (int j = 0; j < tests[i]; ++j) {
            isap(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer1.Stop();

        timer2.Start();
        for (int j = 0; j < tests[i]; ++j) {
            push_relabel(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer2.Stop();

        timer3.Start();
        for (int j = 0; j < tests[i]; ++j) {
            bk(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer3.Stop();

        printf("%6d %13s %16s %16s\n", n,
               timer1.average_time(tests[i]).c_str(),
               timer2.average_time(tests[i]).c_str(),
               timer3.average_time(tests[i]).c_str());
    }
    printf("--------------------------------------------------------------\n");
    printf("\n");
}

// Sparse graph.
TEST_F(MaxFlowPerformanceTest, SparseGraph) {
    int n_tests = 7;
    int v[]     = { 100,  1000,  1000,   10000,  10000,  100000, 100000  };
    int e[]     = { 1000, 10000, 100000, 100000, 500000, 500000, 1000000 };
    int tests[] = { 100,  10,    10,     1,      1,      1,      1       };

    graph::ImprovedSAPMaxFlow<int> isap;
    graph::PushRelableMaxFlow<int> push_relabel;
    graph::BoykovKolmogorovMaxFlow<int> bk;

    printf("\n");
    printf("    |V|        |E|      Imporeved SAP   Push Relable"
           "   Boykov Kolmogorov\n");
    printf("-------------------------------------------------------------------"
           "-----\n");
    for (int i = 0; i < n_tests; ++i) {
        int n = v[i];
        int m = e[i];
        RandomGraph(n, m);

        Timer timer1, timer2, timer3;
        timer1.Start();
        for (int j = 0; j < tests[i]; ++j) {
            isap(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer1.Stop();

        timer2.Start();
        for (int j = 0; j < tests[i]; ++j) {
            push_relabel(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer2.Stop();

        timer3.Start();
        for (int j = 0; j < tests[i]; ++j) {
            bk(graph_, capacity_, 0, n - 1, &flow_);
        }
        timer3.Stop();

        std::string t1 = Timer::ReadableTime(timer1.elapsed_seconds() /
                                                   tests[i]);
        std::string t2 = Timer::ReadableTime(timer2.elapsed_seconds() /
                                                   tests[i]);
        std::string t3 = Timer::ReadableTime(timer3.elapsed_seconds() /
                                                   tests[i]);
        printf("%7d %10d %13s %16s %16s\n",
               n, m, t1.c_str(), t2.c_str(), t3.c_str());
    }
    printf("-------------------------------------------------------------------"
           "-----\n");
    printf("\n");
    const int n = 1000, m = 10000;
    graph::ImprovedSAPMaxFlow<int> max_flow;
    RandomGraph(n, m);
    max_flow(graph_, capacity_, 0, n - 1, &flow_);
}

} // namespace test
} // namespace cl

#endif // CODELIBRARY_TEST_GRPAH_FLOW_PERFORMANCE_TEST_H_
