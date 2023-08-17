/**********************************************************************************************************************/
/* Description        : Global junkyard.                                                                              */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Important: Here we ONLY insert includes that are relevant for
   the operation of the rest of THIS file. */
/* TODO: Include these files directly in all relevant files */
#include <cmsis_compiler.h>
#include <cmsis_os2.h>
#include <macros.h>
#include <os.h>
#include <proj_assert.h>

/* Library headers */
#include <stdint.h>

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* WD reset. */
#define RESET() while (1)

/* Timeout no wait value */
#define NO_WAIT (0)

/* --------------------------------------------- Generally useful macros -------------------------------------------- */

#define FUNCTION_NOT_IMPLEMENTED (NULL)
#define ARGUMENTS_UNUSED(...)    PP_OVERLOAD(PP_UNUSED, __VA_ARGS__)(__VA_ARGS__)
#define ARGUMENT_UNUSED(x)       ARGUMENTS_UNUSED(x)
#define RET_VAL_UNUSED(x)        ((void)x)

#define MSEC_TO_TICKS(time_msec) (((float)(time_msec) / 1000) * osKernelGetTickFreq())

#define BITS_IN_BYTE         (8)
#define BYTES_TO_BITS(bytes) (BITS_IN_BYTE * (bytes))

#define HZ_IN_MHZ 1000000ULL
/* Warning - not double - will round down! */
#define HZ_TO_MHZ(freq) ((freq) / HZ_IN_MHZ)

#define IS_MD5_EQUAL(p_md5_1, p_md5_2) ((memcmp((p_md5_1), (p_md5_2), 16)) == 0)

/* Evaluates to number of elements in an array; compile error if not an array (e.g. pointer) */
#define ARRAY_LEN(array) ((size_t)(IS_ARRAY(array) + (sizeof(array) / sizeof((array)[0]))))

/* Enumerate each item in an array. Note that 'array' must be defined as an array, and can't be just a pointer. */
#define array_for_each(item, array)                                                                                    \
    for (__typeof__((array)[0])*(item) = (array); (item) < (array) + ARRAY_LEN(array); (item)++)

/* Enumerate each item in an array. Supports pointers and not just arrays */
#define array_for_each_len(item, array, len)                                                                           \
    for (__typeof__((array)[0])*(item) = (array); (item) < (array) + (len); (item)++)

/* --------------------------------------------- Arithmetic Operations ---------------------------------------------- */

/* MIN macro for calculating the min of values, should be used for const evaluation */
#define MIN(...)                                       PP_OVERLOAD_ALT(_MIN, __VA_ARGS__)(__VA_ARGS__)
#define _MIN_1(n_)                                     (n_)
#define _MIN_2(n1_, n2_)                               ((n1_) < (n2_) ? (n1_) : (n2_))
#define _MIN_3(n1_, n2_, n3_)                          (_MIN_2(n1_, _MIN_2(n2_, n3_)))
#define _MIN_4(n1_, n2_, n3_, n4_)                     (_MIN_2(n1_, _MIN_3(n2_, n3_, n4_)))
#define _MIN_5(n1_, n2_, n3_, n4_, n5_)                (_MIN_2(n1_, _MIN_4(n2_, n3_, n4_, n5_)))
#define _MIN_6(n1_, n2_, n3_, n4_, n5_, n6_)           (_MIN_2(n1_, _MIN_5(n2_, n3_, n4_, n5_, n6_)))
#define _MIN_7(n1_, n2_, n3_, n4_, n5_, n6_, n7_)      (_MIN_2(n1_, _MIN_6(n2_, n3_, n4_, n5_, n6_, n7_)))
#define _MIN_8(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_) (_MIN_2(n1_, _MIN_7(n2_, n3_, n4_, n5_, n6_, n7_, n8_)))
#define _MIN_9(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_)                                                            \
    (_MIN_2(n1_, _MIN_8(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_)))
#define _MIN_10(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_)                                                     \
    (_MIN_2(n1_, _MIN_9(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_)))
#define _MIN_11(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_)                                               \
    (_MIN_2(n1_, _MIN_10(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_)))
#define _MIN_12(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_)                                         \
    (_MIN_2(n1_, _MIN_11(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_)))
#define _MIN_13(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_)                                   \
    (_MIN_2(n1_, _MIN_12(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_)))
#define _MIN_14(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_)                             \
    (_MIN_2(n1_, _MIN_13(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_)))
#define _MIN_15(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_)                       \
    (_MIN_2(n1_, _MIN_14(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_)))
#define _MIN_16(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_)                 \
    (_MIN_2(n1_, _MIN_15(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_)))
#define _MIN_17(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_)           \
    (_MIN_2(n1_, _MIN_16(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_)))
#define _MIN_18(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_)     \
    (_MIN_2(n1_, _MIN_17(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_)))
#define _MIN_19(                                                                                                       \
    n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_)           \
    (_MIN_2(                                                                                                           \
        n1_,                                                                                                           \
        _MIN_18(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_)))
#define _MIN_20(                                                                                                       \
    n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_, n20_)     \
    (_MIN_2(_MIN_10(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_),                                                \
            _MIN_10(n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_, n20_)))

/* MAX macro for calculating the max of values, should be used for const evaluation */
#define MAX(...)                                       PP_OVERLOAD_ALT(_MAX, __VA_ARGS__)(__VA_ARGS__)
#define _MAX_1(n_)                                     (n_)
#define _MAX_2(n1_, n2_)                               ((n1_) > (n2_) ? (n1_) : (n2_))
#define _MAX_3(n1_, n2_, n3_)                          (_MAX_2(n1_, _MAX_2(n2_, n3_)))
#define _MAX_4(n1_, n2_, n3_, n4_)                     (_MAX_2(n1_, _MAX_3(n2_, n3_, n4_)))
#define _MAX_5(n1_, n2_, n3_, n4_, n5_)                (_MAX_2(n1_, _MAX_4(n2_, n3_, n4_, n5_)))
#define _MAX_6(n1_, n2_, n3_, n4_, n5_, n6_)           (_MAX_2(n1_, _MAX_5(n2_, n3_, n4_, n5_, n6_)))
#define _MAX_7(n1_, n2_, n3_, n4_, n5_, n6_, n7_)      (_MAX_2(n1_, _MAX_6(n2_, n3_, n4_, n5_, n6_, n7_)))
#define _MAX_8(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_) (_MAX_2(n1_, _MAX_7(n2_, n3_, n4_, n5_, n6_, n7_, n8_)))
#define _MAX_9(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_)                                                            \
    (_MAX_2(n1_, _MAX_8(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_)))
#define _MAX_10(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_)                                                     \
    (_MAX_2(n1_, _MAX_9(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_)))
#define _MAX_11(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_)                                               \
    (_MAX_2(n1_, _MAX_10(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_)))
#define _MAX_12(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_)                                         \
    (_MAX_2(n1_, _MAX_11(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_)))
#define _MAX_13(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_)                                   \
    (_MAX_2(n1_, _MAX_12(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_)))
#define _MAX_14(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_)                             \
    (_MAX_2(n1_, _MAX_13(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_)))
#define _MAX_15(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_)                       \
    (_MAX_2(n1_, _MAX_14(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_)))
#define _MAX_16(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_)                 \
    (_MAX_2(n1_, _MAX_15(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_)))
#define _MAX_17(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_)           \
    (_MAX_2(n1_, _MAX_16(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_)))
#define _MAX_18(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_)     \
    (_MAX_2(n1_, _MAX_17(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_)))
#define _MAX_19(                                                                                                       \
    n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_)           \
    (_MAX_2(                                                                                                           \
        n1_,                                                                                                           \
        _MAX_18(n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_)))
#define _MAX_20(                                                                                                       \
    n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_, n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_, n20_)     \
    (_MAX_2(_MAX_10(n1_, n2_, n3_, n4_, n5_, n6_, n7_, n8_, n9_, n10_),                                                \
            _MAX_10(n11_, n12_, n13_, n14_, n15_, n16_, n17_, n18_, n19_, n20_)))

/* Safe versions of MIN/MAX */
#define MIN_S(a, b)                                                                                                    \
    ({                                                                                                                 \
        __typeof__(a) _a = (a);                                                                                        \
        __typeof__(b) _b = (b);                                                                                        \
        _a < b ? _a : _b;                                                                                              \
    })

#define MAX_S(a, b)                                                                                                    \
    ({                                                                                                                 \
        __typeof__(a) _a = (a);                                                                                        \
        __typeof__(b) _b = (b);                                                                                        \
        _a > b ? _a : _b;                                                                                              \
    })

#define ABS_S(x_)                                                                                                      \
    ({                                                                                                                 \
        __typeof__(x_) x = (x_);                                                                                       \
        x > 0 ? x : -x;                                                                                                \
    })

/* Note - DIVIDE_AND_ROUND_UP - Do not use with negative values! */
#define DIVIDE_AND_ROUND_UP(numerator, denominator) (((numerator) + (denominator)-1) / (denominator))

/* Divide positive or negative dividend by positive divisor and round to closesr integer.
   Result is undefined for negative divisors and for negative dividends if the divisor variable is unsigned. */
#define DIVIDE_AND_ROUND(x, divisor)                                                                                   \
    ({                                                                                                                 \
        __typeof__(x)       __x = x;                                                                                   \
        __typeof__(divisor) __d = divisor;                                                                             \
        (((__typeof__(x))-1) > 0 || (__x) > 0) ? (((x) + ((__d) / 2)) / (__d)) : (((x) - ((__d) / 2)) / (__d));        \
    })

/* Checks whether numerator is divisible by denominator */
#define IS_DIVISIBLE(numerator_, denominator_)                                                                         \
    (((numerator_) - (((uint64_t)((double)(numerator_) / (double)(denominator_))) * (double)(denominator_))) == 0)

/* Pads the given size to the next multiple of multiplier (doesn't pad if it is a multiple already) */
#define PAD(size, multiplier) ((size) % (multiplier) == 0 ? (size) : (size) + ((multiplier) - ((size) % (multiplier))))

/* Declare variables and low power manager function used
 * in this section without entering include loops errors.
 * This should be handled when we split this file to several files
 * in framework and product. It can be under os_wrapper for example.
 */
extern void low_power_manager_os_delay(uint32_t time_msec);

#if defined(CONFIG_LOW_POWER_MANAGER)
#define os_delay_msec(delay_time_msec) low_power_manager_os_delay(delay_time_msec)
#else /* CONFIG_LOW_POWER_MANAGER */
#define os_delay_msec(delay_time_msec) proj_assert(osDelay(MSEC_TO_TICKS(delay_time_msec)) == osOK)
#endif /* CONFIG_LOW_POWER_MANAGER */
