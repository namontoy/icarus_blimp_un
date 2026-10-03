/* test_util.h - minimal assertion helpers shared by every core unit test.
 *
 * No framework, no dependencies beyond libc. Each test binary is standalone:
 * it prints one line per check group and returns non-zero on any failure, so
 * `python tools/build.py all` can just look at exit codes.
 */
#ifndef BLIMP_TEST_UTIL_H
#define BLIMP_TEST_UTIL_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int blimp_test_failures = 0;
static int blimp_test_checks = 0;

#define CHECK(cond)                                                          \
    do {                                                                     \
        blimp_test_checks++;                                                 \
        if (!(cond)) {                                                       \
            blimp_test_failures++;                                           \
            printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                    \
    } while (0)

#define CHECK_MSG(cond, ...)                                                 \
    do {                                                                     \
        blimp_test_checks++;                                                 \
        if (!(cond)) {                                                       \
            blimp_test_failures++;                                           \
            printf("  FAIL %s:%d  ", __FILE__, __LINE__);                    \
            printf(__VA_ARGS__);                                             \
            printf("\n");                                                    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                \
    do {                                                                     \
        blimp_test_checks++;                                                 \
        const double _a = (double)(a), _b = (double)(b);                     \
        if (!(fabs(_a - _b) <= (double)(tol))) {                             \
            blimp_test_failures++;                                           \
            printf("  FAIL %s:%d  %s ~= %s  (%.9g vs %.9g, tol %g)\n",       \
                   __FILE__, __LINE__, #a, #b, _a, _b, (double)(tol));       \
        }                                                                    \
    } while (0)

#define CHECK_FINITE(x)                                                      \
    do {                                                                     \
        blimp_test_checks++;                                                 \
        if (!isfinite((double)(x))) {                                        \
            blimp_test_failures++;                                           \
            printf("  FAIL %s:%d  %s not finite (%.9g)\n",                   \
                   __FILE__, __LINE__, #x, (double)(x));                     \
        }                                                                    \
    } while (0)

#define TEST_GROUP(name) printf("  [%s]\n", (name))

#define TEST_REPORT()                                                        \
    do {                                                                     \
        printf("  %d checks, %d failures\n",                                 \
               blimp_test_checks, blimp_test_failures);                      \
        return blimp_test_failures == 0 ? 0 : 1;                             \
    } while (0)

#endif /* BLIMP_TEST_UTIL_H */
