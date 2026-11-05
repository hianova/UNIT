#ifndef UNIT_TEST_KIT_H
#define UNIT_TEST_KIT_H

#include "unit_flow_exec.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    const char *suite_name;
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t current_stage;
} UnitTestSuiteState;

#define UNIT_TEST_SUITE_BEGIN(name_str)                                                                \
    static UnitTestSuiteState _unit_test_state = {.suite_name = (name_str),                            \
                                                  .total_tests = 0,                                    \
                                                  .passed_tests = 0,                                   \
                                                  .failed_tests = 0,                                   \
                                                  .current_stage = 0};                                 \
    printf(                                                                                            \
        "========================================================================================\n"); \
    printf("  🧪 Running UNIT Test Suite: %s\n", (name_str));                                          \
    printf(                                                                                            \
        "========================================================================================\n\n");

#define UNIT_STAGE_BEGIN(stage_num, stage_name_str)                                  \
    do {                                                                             \
        _unit_test_state.current_stage = (stage_num);                                \
        printf("--- [Stage %u: %s] ---\n", (unsigned)(stage_num), (stage_name_str)); \
    } while (0)

#define UNIT_ASSERT_TRUE(cond)                                                                 \
    do {                                                                                       \
        _unit_test_state.total_tests++;                                                        \
        if (!(cond)) {                                                                         \
            _unit_test_state.failed_tests++;                                                   \
            fprintf(stderr, "❌ ASSERT_TRUE FAILED [%s:%d]: %s\n", __FILE__, __LINE__, #cond); \
        } else {                                                                               \
            _unit_test_state.passed_tests++;                                                   \
        }                                                                                      \
    } while (0)

#define UNIT_ASSERT_FALSE(cond)                                                                 \
    do {                                                                                        \
        _unit_test_state.total_tests++;                                                         \
        if (cond) {                                                                             \
            _unit_test_state.failed_tests++;                                                    \
            fprintf(stderr, "❌ ASSERT_FALSE FAILED [%s:%d]: %s\n", __FILE__, __LINE__, #cond); \
        } else {                                                                                \
            _unit_test_state.passed_tests++;                                                    \
        }                                                                                       \
    } while (0)

#define UNIT_ASSERT_EQ(a, b)                                                               \
    do {                                                                                   \
        long long _unit_a = (long long)(a);                                                \
        long long _unit_b = (long long)(b);                                                \
        _unit_test_state.total_tests++;                                                    \
        if (_unit_a != _unit_b) {                                                          \
            _unit_test_state.failed_tests++;                                               \
            fprintf(stderr, "❌ ASSERT_EQ FAILED [%s:%d]: (%s == %lld) != (%s == %lld)\n", \
                    __FILE__, __LINE__, #a, _unit_a, #b, _unit_b);                         \
        } else {                                                                           \
            _unit_test_state.passed_tests++;                                               \
        }                                                                                  \
    } while (0)

#define UNIT_ASSERT_NE(a, b)                                                                      \
    do {                                                                                          \
        long long _unit_a = (long long)(a);                                                       \
        long long _unit_b = (long long)(b);                                                       \
        _unit_test_state.total_tests++;                                                           \
        if (_unit_a == _unit_b) {                                                                 \
            _unit_test_state.failed_tests++;                                                      \
            fprintf(stderr, "❌ ASSERT_NE FAILED [%s:%d]: %s == %s (%lld)\n", __FILE__, __LINE__, \
                    #a, #b, _unit_a);                                                             \
        } else {                                                                                  \
            _unit_test_state.passed_tests++;                                                      \
        }                                                                                         \
    } while (0)

#define UNIT_TEST_SUITE_END()                                                                                \
    do {                                                                                                     \
        printf(                                                                                              \
            "\n========================================================================================\n"); \
        printf("  🏁 SUITE SUMMARY [%s]: %u PASSED, %u FAILED (Total %u Assertions)\n",                      \
               _unit_test_state.suite_name, _unit_test_state.passed_tests,                                   \
               _unit_test_state.failed_tests, _unit_test_state.total_tests);                                 \
        printf(                                                                                              \
            "========================================================================================\n\n"); \
        if (_unit_test_state.failed_tests > 0)                                                               \
            return 1;                                                                                        \
        return 0;                                                                                            \
    } while (0)

#endif /* UNIT_TEST_KIT_H */
