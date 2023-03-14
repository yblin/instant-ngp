//
// Copyright 2014-2022 Yangbin Lin. All Rights Reserved.
//
// Author: yblin@jmu.edu.cn (Yangbin Lin)
//
// This file is part of the Code Library.
//
// Simply include this file to enable the Code Library testing framework.
//

#ifndef CODELIBRARY_BASE_TESTING_H_
#define CODELIBRARY_BASE_TESTING_H_

#include <cinttypes>
#include <chrono>
#include <cstdarg>
#include <map>
#include <string>

#include "codelibrary/base/array.h"
#include "codelibrary/base/equal.h"
#include "codelibrary/base/message.h"
#include "codelibrary/base/string_printf.h"

namespace cl {

/**
 * The abstract test class that all tests should inherit from.
 *
 * We do not need to define a Test object explicitly. The TEST macro
 * automatically do this for you.
 *
 * The only time you derive from Test is when defining a test fixture to be used
 * a TEST_F.  For example:
 *
 *   class IsPrimeTest : public Test {
 *   protected:
 *       virtual void SetUp() { ... }
 *       virtual void Finish() { ... }
 *       ...
 *   };
 *
 *   TEST_F(IsPrimeTest, TestSmallPrimes) { ... }
 *   TEST_F(IsPrimeTest, TestLargePrimes) { ... }
 */
class Test {
public:
    Test() = default;

    Test(const Test&) = delete;

    virtual ~Test() = default;

    Test& operator=(const Test&) = delete;

    /**
     * Set up the test fixture.
     */
    virtual void SetUp() {}

    /**
     * Call when test finished.
     */
    virtual void Finish() {}

    /**
     * Set up, execute, and finish the test.
     */
    void Run() {
        SetUp();
        TestBody();
        Finish();
    }

private:
    /**
     * Run the test after the test fixture has been set up.
     *
     * A sub-class must implement this to define the test logic.
     *
     * DO NOT OVERRIDE THIS FUNCTION DIRECTLY IN A USER PROGRAM.
     * Instead, use the TEST or TEST_F macros.
     */
    virtual void TestBody() {}
};

/**
 * TestProgram is a singleton class, which is consist of a set of test cases.
 * It has only one instance, which is created by TestProgram::GetInstance().
 * This instance is never deleted.
 *
 * TestProgram is not copyable.
 */
class TestProgram {
public:
    /**
     * A TestInfo object stores the following information about a test:
     *   test case name;
     *   test name;
     *   a pointer to the test.
     *
     * The constructor of TestInfo registers itself with the UnitTest singleton
     * such that the RUN_ALL_TESTS() function knows which tests to run.
     */
    struct TestInfo {
        TestInfo(const char* test_case_name,
                 const char* test_name,
                 Test* test_ptr)
            : test_case_name(test_case_name),
              name(test_name),
              test(test_ptr) {}

        TestInfo() = delete;

        TestInfo(const TestInfo&) = delete;

        /**
         * Destructs a TestInfo object.
         * This function is not virtual, so don't inherit from TestInfo.
         */
        ~TestInfo() {
            delete test;
        }

        TestInfo& operator=(const TestInfo&) = delete;

        // Test case name.
        std::string test_case_name;

        // Test name.
        std::string name;

        // Pointer to the test.
        Test* test = nullptr;
    };

private:
    /**
     * Result for TestProgram.
     */
    struct Result {
        // The number of all test cases.
        int total_test_case_count = 0;

        // The number of successful tests.
        int successful_test_count = 0;

        // The number of failed tests.
        int failed_test_count = 0;

        // The number of all tests.
        int total_test_count = 0;

        // The elapsed time, in milliseconds.
        int64_t elapsed_time = 0;

        // Store failed tests.
        Array<TestInfo*> failed_tests;
    };

    /**
     * TestProgram is consist of a set of test cases.
     */
    struct TestCase {
        /**
         * Creates a TestCase with the given name.
         */
        explicit TestCase(const char* test_case_name)
            : name(test_case_name) {}

        ~TestCase() {
            for (TestInfo* test_info : test_info_list) {
                delete test_info;
            }
        }

        // Test case name.
        std::string name;

        // Successful test count. A test case has several TestBlock, this member
        // is used to count the number of successful blocks.
        int successful_test_count = 0;

        // Failed test count.
        int failed_test_count = 0;

        // Elapsed time (in milliseconds). It is the sum of the running time of
        // all TestBlocks of this test case.
        int64_t elapsed_time = 0;

        // Test info list.
        Array<TestInfo*> test_info_list;
    };

private:
    // Prevent users define their own TestProgram.
    TestProgram() = default;

public:
    /**
     * Each ASSERT macro, such as ASSERT(), ASSERT_EQ(), has a result.
     */
    struct AssertResult {
        AssertResult(const char* filename, int line, const char* msg,
                     bool success_or_fail)
            : file_name(filename),
              line_number(line),
              message(msg),
              success(success_or_fail) {}

        AssertResult& operator =(const Message& msg) {
            if (!msg.empty()) message += "\n" + msg.ToString();
            return *this;
        }

        // The source file where the test took place.
        std::string file_name;

        // The line number in the source file where the test took place.
        int line_number = -1;

        // The failure message.
        std::string message;

        // The ASSERT macro is success or not.
        bool success = false;
    };

    // We disallow the copy constructor.
    TestProgram(const TestProgram&) = delete;

    // We disallow the copy assignment.
    TestProgram& operator=(const TestProgram&) = delete;

    ~TestProgram() {
        for (auto test_case : test_case_list_) {
            delete test_case;
        }
    }

    /**
     * Get the singleton TestProgram object.
     *
     * A TestProgram object is constructed when this method is called at first
     * time. The consecutive calls will return the same object.
     */
    static TestProgram* GetInstance() {
        static TestProgram instance;
        return &instance;
    }

    /**
     * Register a Test.
     */
    TestInfo* RegisterTest(const char *test_case_name, const char *name,
                           Test* test) {
        ++result_.total_test_count;

        TestInfo* test_info = new TestInfo(test_case_name, name, test);

        if (test_case_map_.find(test_case_name) == test_case_map_.end()) {
            TestCase* test_case = new TestCase(test_case_name);
            test_case->test_info_list.emplace_back(test_info);
            test_case_list_.push_back(test_case);
            ++n_test_cases_;
            test_case_map_[test_case_name] = test_case;
        } else {
            test_case_map_[test_case_name]->test_info_list.
                    emplace_back(test_info);
        }

        return test_info;
    }

    /**
     * Add a AssertResult.
     *
     * All ASSERT macros (e.g. ASSERT, ASSERT_EQ, etc) implicitly call this to
     * report their results.
     * Users should use the ASSERT macros instead of calling this directly.
     */
    void operator += (const AssertResult& result) {
        assert_results_.push_back(result);
        PrintOnAssertResult(result);
    }

    /**
     * Run all tests in this TestProgram object and print the result.
     *
     * This method can only be called from the main thread.
     *
     * Return 0 if successful, or 1 otherwise.
     */
    int Run() {
        result_.total_test_case_count = n_test_cases_;

        PrintOnTestProgramStart();

        std::chrono::time_point<std::chrono::system_clock> start, end;
        for (TestCase* test_case : test_case_list_) {
            int64_t elapsed_time = 0;

            PrintOnTestCaseStart(test_case);
            for (TestInfo* test_info : test_case->test_info_list) {
                PrintOnTestStart(test_info);

                start = std::chrono::system_clock::now();
                test_info->test->Run();
                end = std::chrono::system_clock::now();

                int64_t time = std::chrono::duration_cast<
                        std::chrono::milliseconds>(end - start).count();

                elapsed_time += time;

                if (IsPassed()) {
                    ++result_.successful_test_count;
                    PrintOnTestSuccessed(test_info, time);
                } else {
                    result_.failed_tests.push_back(test_info);
                    ++result_.failed_test_count;
                    PrintOnTestFailed(test_info, time);
                }

                assert_results_.clear();
            }

            test_case->elapsed_time = elapsed_time;
            PrintOnTestCaseEnd(test_case);

            result_.elapsed_time += elapsed_time;
        }

        PrintOnTestProgramEnd();

        return (result_.failed_test_count > 0) ? 1 : 0;
    }

    /**
     * Return a failure message for ASSERT.
     */
    static std::string AssertFailureMessage(const char* expression_text) {
        return StringPrintf("ASSERT(%s) failed!\n", expression_text);
    }

    /**
     * Return a failure message for ASSERT_FALSE.
     */
    static std::string AssertFalseFailureMessage(const char* expression_text) {
        return StringPrintf("ASSERT_FALSE(%s) failed!\n", expression_text);
    }

    /**
     * Return a failure message for ASSERT_EQ.
     *
     * For example, for ASSERT_EQ(foo, bar) where foo is 5 and bar is 6,
     * we have:
     *
     *   expected_expression: "foo"
     *   actual_expression:   "bar"
     *   expected_value:      "5"
     *   actual_value:        "6"
     */
    template <typename T1, typename T2>
    static std::string AssertEQFailureMessage(const char* actual_expression,
                                              const char* expected_expression,
                                              const T1& actual_value,
                                              const T2& expected_value) {
        Message msg1(actual_value), msg2(expected_value);
        std::string actual_str = msg1.ToString();
        std::string expected_str = msg2.ToString();

        std::string message = StringPrintf("ASSERT_EQ(%s, %s) failed!\n",
                                           actual_expression,
                                           expected_expression);

        if (actual_str == actual_expression) {
            if (expected_str == expected_expression) {
                message += StringPrintf("%s != %s", actual_expression,
                                                    expected_expression);
            } else {
                message += StringPrintf("%s = %s, not equal to %s",
                                        expected_expression,
                                        expected_str.c_str(),
                                        actual_str.c_str());
            }
        } else if (expected_str == expected_expression) {
            message += StringPrintf("%s = %s, not equal to %s",
                                    actual_expression,
                                    actual_str.c_str(),
                                    expected_str.c_str());
        } else {
            message += StringPrintf("%s = %s, but %s = %s",
                                    actual_expression,
                                    actual_str.c_str(),
                                    expected_expression,
                                    expected_str.c_str());
        }

        return message;
    }

    /**
     * Return a failure message for ASSERT_EQ_NEAR.
     */
    template <typename T>
    static std::string
    AssertEQNearFailureMessage(const char* a_expression,
                               const char* b_expression,
                               const char* epsilon_expression,
                               const T& a,
                               const T& b,
                               const T& epsilon) {
        Message msg1(a), msg2(b), msg3(epsilon), msg4(std::abs(a - b));
        std::string str_a = msg1.ToString();
        std::string str_b = msg2.ToString();
        std::string str_e = msg3.ToString();
        std::string diff  = msg4.ToString();

        return StringPrintf("ASSERT_EQ_NEAR(%s, %s, %s) failed!\n"
                            "The difference between %s and %s is %s, "
                            "which exceeds\n"
                            "%s, where\n"
                            "%s evaluates to %s,\n"
                            "%s evaluates to %s, and\n"
                            "%s evaluates to %s.",
                            a_expression,
                            b_expression,
                            epsilon_expression,
                            a_expression,
                            b_expression,
                            diff.c_str(),
                            epsilon_expression,
                            a_expression,
                            str_a.c_str(),
                            b_expression,
                            str_b.c_str(),
                            epsilon_expression,
                            str_e.c_str());
    }

    /**
     * Return a failure message for equality assertions such as ASSERT_EQ_RANGE.
     */
    template <typename T1, typename T2>
    static std::string
    AssertEQRangeFailureMessage(const char* actual_expression_first,
                                const char* actual_expression_last,
                                const char* expected_expression_first,
                                const char* expected_expression_last,
                                T1 actual_first,
                                T1 actual_last,
                                T2 expected_first,
                                T2 expected_last) {
        using Type1 = typename std::iterator_traits<T1>::value_type;
        using Type2 = typename std::iterator_traits<T2>::value_type;
        Array<Type1> a(actual_first, actual_last);
        Array<Type2> b(expected_first, expected_last);
        Message msg1(a), msg2(b);

        return StringPrintf("ASSERT_EQ_RANGE(%s, %s, %s, %s) failed!\n"
                            "Value of: [%s, %s)\n"
                            "Actual  : \n"
                            "%s\n"
                            "Expected: [%s, %s)\n"
                            "Which is: \n"
                            "%s",
                            actual_expression_first,
                            actual_expression_last,
                            expected_expression_first,
                            expected_expression_last,
                            actual_expression_first,
                            actual_expression_last,
                            msg1.ToString().c_str(),
                            expected_expression_first,
                            expected_expression_last,
                            msg2.ToString().c_str());
    }

private:
    /**
     * Return true if the current test successes.
     */
    bool IsPassed() const {
        for (const AssertResult& result : assert_results_) {
            if (!result.success) {
                return false;
            }
        }
        return true;
    }

    /**
     * Print information before any test activity starts.
     */
    void PrintOnTestProgramStart() const {
        printf("[==========] ");
        printf("Running %s from %s.\n",
               PrintTestCount(result_.total_test_count).c_str(),
               PrintTestCaseCount(result_.total_test_case_count).c_str());
        fflush(stdout);
    }

    /**
     * Print information before the test case starts.
     */
    void PrintOnTestCaseStart(TestCase* test_case) const {
        printf("[----------] ");
        printf("%s from %s\n",
               PrintTestCount(test_case->test_info_list.size()).c_str(),
               test_case->name.c_str());
        fflush(stdout);
    }

    /**
     * Print information before the test starts.
     */
    void PrintOnTestStart(TestInfo* test_info) const {
        printf("[ RUN      ] ");
        PrintTestName(test_info->test_case_name, test_info->name);
        printf("\n");
        fflush(stdout);
    }

    /**
     * Print information after a failed assertion or a SUCCEED() invocation.
     */
    void PrintOnAssertResult(const AssertResult& result) const {
        // If the ASSERT succeeds, we don't need to do anything.
        if (result.success) return;

        printf("[          ]\n");
        printf("[ FAILURE  ] At ");
        printf("%s(%d)\n", result.file_name.c_str(), result.line_number);

        // Print failure message.
        printf("\n%s\n\n", result.message.c_str());

        fflush(stdout);
    }

    /**
     * Print information after the test successes.
     */
    void PrintOnTestSuccessed(TestInfo* test_info, int64_t test_time) const {
        printf("[       OK ] ");
        PrintTestName(test_info->test_case_name, test_info->name);
        printf(" (%" PRIu64 " ms)\n", test_time);

        fflush(stdout);
    }

    /**
     * Print information after the test failed.
     */
    void PrintOnTestFailed(TestInfo* test_info, int64_t test_time) const {
        printf("[  FAILED  ] ");
        PrintTestName(test_info->test_case_name, test_info->name);
        printf(" (%" PRIu64 " ms)\n", test_time);

        fflush(stdout);
    }

    /**
     * Print information after the test case ends.
     */
    void PrintOnTestCaseEnd(TestCase* test_case) const {
        printf("[----------] ");
        printf("%s from %s (%" PRIu64 " ms total)\n\n",
               PrintTestCount(test_case->test_info_list.size()).c_str(),
               test_case->name.c_str(),
               test_case->elapsed_time);
        fflush(stdout);
    }

    /**
     * Print information after all test activities have ended.
     */
    void PrintOnTestProgramEnd() const {
        printf("[==========] ");
        printf("%s from %s ran.",
               PrintTestCount(result_.total_test_count).c_str(),
               PrintTestCaseCount(result_.total_test_case_count).c_str());
        printf(" (%" PRIu64 " ms total)\n", result_.elapsed_time);
        printf("[  PASSED  ] ");
        printf("%s.\n", PrintTestCount(result_.successful_test_count).c_str());

        if (result_.failed_test_count > 0) {
            printf("[  FAILED  ] ");
            printf("%s, listed below:\n",
                   PrintTestCount(result_.failed_test_count).c_str());

            for (TestInfo* test_info : result_.failed_tests) {
                printf("[  FAILED  ] ");
                PrintTestName(test_info->test_case_name, test_info->name);
                printf("\n");
            }

            printf("\n%2d FAILED %s\n", result_.failed_test_count,
                   result_.failed_test_count == 1 ? "TEST" : "TESTS");
        }
        fflush(stdout);
    }

    /**
     * Print Test name.
     */
    static void PrintTestName(const std::string& test_case,
                              const std::string& test) {
        printf("%s.%s", test_case.c_str(), test.c_str());
    }

    /**
     * Print a countable noun.
     * Depending on its quantity, either the singular form or the plural form is
     * used. e.g.
     *
     * PrintCountableNoun(1, "formula", "formuli") returns "1 formula".
     * PrintCountableNoun(5, "book", "books") returns "5 books".
     */
    static std::string PrintCountableNoun(int count,
                                          const char* singular_form,
                                          const char* plural_form) {
        return std::to_string(count) + " " + ((count <= 1) ? singular_form :
                                                             plural_form);
    }

    /**
     * Print the count of tests.
     */
    static std::string PrintTestCount(int test_count) {
        return PrintCountableNoun(test_count, "test", "tests");
    }

    /**
     * Print the count of test cases.
     */
    static std::string PrintTestCaseCount(int test_case_count) {
        return PrintCountableNoun(test_case_count, "test case", "test cases");
    }

    // The number of test cases.
    int n_test_cases_ = 0;

    // Store the registered test cases.
    Array<TestCase*> test_case_list_;

    // Map the test by its test case.
    std::map<std::string, TestCase*> test_case_map_;

    // The array of AssertResult.
    Array<AssertResult> assert_results_;

    // The test program result.
    Result result_;
};

} // namespace cl

#define TESTING_MESSAGE_AT(file, line, message, success_or_fail) \
    (*cl::TestProgram::GetInstance()) += \
        cl::TestProgram::AssertResult(file, line, message, success_or_fail) = \
            cl::Message()

#define TESTING_MESSAGE(message, success_or_fail) \
    TESTING_MESSAGE_AT(__FILE__, __LINE__, message, success_or_fail)

#define TESTING_FAILURE(message) return TESTING_MESSAGE(message, false)

/**
 * Assert expression is true.
 */
#define ASSERT(expression) \
    if (!(expression)) \
        TESTING_FAILURE(cl::TestProgram:: \
                        AssertFailureMessage(#expression).c_str())

/**
 * Assert expression is false.
 */
#define ASSERT_FALSE(expression) \
    if (expression) \
        TESTING_FAILURE(cl::TestProgram:: \
                        AssertFalseFailureMessage(#expression).c_str())

/**
 * Assert the expected equal to actual.
 */
#define ASSERT_EQ(actual, expected) \
    if (!cl::Equal(actual, expected)) \
        TESTING_FAILURE(cl::TestProgram::AssertEQFailureMessage( \
                        #actual, #expected, actual, expected).c_str())

/**
 * Assert if two parameter is nearly the same.
 */
#define ASSERT_EQ_NEAR(a, b, absolute_range) \
    if (!cl::Equal(a, b, absolute_range))   \
        TESTING_FAILURE(cl::TestProgram::AssertEQNearFailureMessage( \
             #a, #b, #absolute_range, a, b, absolute_range).c_str())

/**
 * Range EQ assert.
 */
#define ASSERT_EQ_RANGE(actual_first, actual_last,      \
                        expected_first, expected_last)  \
    if (!(cl::Equal(actual_first, actual_last,          \
                    expected_first, expected_last)))    \
        TESTING_FAILURE(cl::TestProgram::AssertEQRangeFailureMessage( \
             #actual_first, #actual_last, #expected_first, #expected_last, \
             actual_first, actual_last, expected_first, expected_last).c_str())

/**
 * Expands to the name of the class that implements the given test.
 */
#define TEST_CLASS_NAME(test_case_name, test_name) \
    test_case_name##_##test_name##_Test

/**
 * Helper macro for defining tests.
 */
#define TESTING(test_case_name, test_name, parent_test_class) \
class TEST_CLASS_NAME(test_case_name, test_name) : public parent_test_class {\
public:\
    TEST_CLASS_NAME(test_case_name, test_name)() = default; \
    TEST_CLASS_NAME(test_case_name, test_name) \
                 (const TEST_CLASS_NAME(test_case_name, test_name)&) = delete; \
    TEST_CLASS_NAME(test_case_name, test_name)& \
    operator=(const TEST_CLASS_NAME(test_case_name, test_name)&) = delete; \
private:\
    void TestBody();\
    static cl::TestProgram::TestInfo* TEST_INFO;\
};\
\
cl::TestProgram::TestInfo* \
TEST_CLASS_NAME(test_case_name, test_name)::TEST_INFO = \
    cl::TestProgram::GetInstance()->RegisterTest(\
        #test_case_name, #test_name, \
        new TEST_CLASS_NAME(test_case_name, test_name));\
void TEST_CLASS_NAME(test_case_name, test_name)::TestBody()

/**
 * Define a test.
 *
 * Each test will be grouped into test cases according to its 'test_case_name'.
 * And each test has its own name, 'test_name', within the test case.
 *
 * Users should put the test code between braces after using this macro.
 *
 * For example:
 *
 * TEST(IsPrimeTest, IsPrime) {
 *     ASSERT(Isprime(23));
 * }
 */
#define TEST(test_case_name, test_name) \
    TESTING(test_case_name, test_name, cl::Test)

/**
 * Define a test that uses a test fixture.
 *
 * A test fixture class must be declared earlier.
 * Users should put the test code between braces after using this macro.
 *
 * For example:
 *
 * class IsPrimeTest : public cl::Test {
 * protected:
 *     virtual void SetUp() { random_.set_seed(3); }
 *
 *     Random random_;
 * };
 *
 * TEST_F(IsPrimeTest, TestRandomPriems) {
 *     ASSERT(IsPrime(random_());
 * }
 */
#define TEST_F(test_case_name, test_name) \
    TESTING(test_case_name, test_name, test_case_name)

/**
 * Use this macro in main() to run all tests.
 * It returns 0 if all tests are successful, or 1 otherwise.
 */
#define RUN_ALL_TESTS() \
    cl::TestProgram::GetInstance()->Run()

#endif // CODELIBRARY_BASE_TESTING_H_
