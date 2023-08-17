/**********************************************************************************************************************/
/* Description: Header containing a bunch of magical macros                                                           */
/**********************************************************************************************************************/
#pragma once

/**********************************************************************************************************************/
/* Includes                                                                                                           */
/**********************************************************************************************************************/

/* Library headers */
#include <stdbool.h>

#include "zephyr/util_macro.h"

/**********************************************************************************************************************/
/* Macros                                                                                                             */
/**********************************************************************************************************************/

/* Most macros are taken from github.com/pfultz2/Cloak/wiki */

/* These allow you to print a value of a macro at compile time, for debugging purposes.
 * For example -
 * #pragma message(VAR_NAME_VALUE(MACRO))
 * */
#define PP_STRINGIFY_IMPL(x) #x
#define PP_STRINGIFY(x)      PP_STRINGIFY_IMPL(x)

#define VAR_NAME_VALUE(var) #var "=" PP_STRINGIFY(var)

#define PP_CONCAT_IMPL(x, y) x##y
#define PP_CONCAT(x, y)      PP_CONCAT_IMPL(x, y)

#define PP_FIRST_ARG(...)          _PP_FIRST_ARG(__VA_ARGS__, throwaway)
#define _PP_FIRST_ARG(first_, ...) first_

#define PP_NARG(...)  PP_NARG_(__VA_ARGS__, PP_RSEQ_N())
#define PP_NARG_(...) PP_ARG_N(__VA_ARGS__)

#define PP_RSEQ_N()                                                                                                    \
    99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72,    \
        71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45,    \
        44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18,    \
        17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
#define PP_ARG_N(_1,                                                                                                   \
                 _2,                                                                                                   \
                 _3,                                                                                                   \
                 _4,                                                                                                   \
                 _5,                                                                                                   \
                 _6,                                                                                                   \
                 _7,                                                                                                   \
                 _8,                                                                                                   \
                 _9,                                                                                                   \
                 _10,                                                                                                  \
                 _11,                                                                                                  \
                 _12,                                                                                                  \
                 _13,                                                                                                  \
                 _14,                                                                                                  \
                 _15,                                                                                                  \
                 _16,                                                                                                  \
                 _17,                                                                                                  \
                 _18,                                                                                                  \
                 _19,                                                                                                  \
                 _20,                                                                                                  \
                 _21,                                                                                                  \
                 _22,                                                                                                  \
                 _23,                                                                                                  \
                 _24,                                                                                                  \
                 _25,                                                                                                  \
                 _26,                                                                                                  \
                 _27,                                                                                                  \
                 _28,                                                                                                  \
                 _29,                                                                                                  \
                 _30,                                                                                                  \
                 _31,                                                                                                  \
                 _32,                                                                                                  \
                 _33,                                                                                                  \
                 _34,                                                                                                  \
                 _35,                                                                                                  \
                 _36,                                                                                                  \
                 _37,                                                                                                  \
                 _38,                                                                                                  \
                 _39,                                                                                                  \
                 _40,                                                                                                  \
                 _41,                                                                                                  \
                 _42,                                                                                                  \
                 _43,                                                                                                  \
                 _44,                                                                                                  \
                 _45,                                                                                                  \
                 _46,                                                                                                  \
                 _47,                                                                                                  \
                 _48,                                                                                                  \
                 _49,                                                                                                  \
                 _50,                                                                                                  \
                 _51,                                                                                                  \
                 _52,                                                                                                  \
                 _53,                                                                                                  \
                 _54,                                                                                                  \
                 _55,                                                                                                  \
                 _56,                                                                                                  \
                 _57,                                                                                                  \
                 _58,                                                                                                  \
                 _59,                                                                                                  \
                 _60,                                                                                                  \
                 _61,                                                                                                  \
                 _62,                                                                                                  \
                 _63,                                                                                                  \
                 _64,                                                                                                  \
                 _65,                                                                                                  \
                 _66,                                                                                                  \
                 _67,                                                                                                  \
                 _68,                                                                                                  \
                 _69,                                                                                                  \
                 _70,                                                                                                  \
                 _71,                                                                                                  \
                 _72,                                                                                                  \
                 _73,                                                                                                  \
                 _74,                                                                                                  \
                 _75,                                                                                                  \
                 _76,                                                                                                  \
                 _77,                                                                                                  \
                 _78,                                                                                                  \
                 _79,                                                                                                  \
                 _80,                                                                                                  \
                 _81,                                                                                                  \
                 _82,                                                                                                  \
                 _83,                                                                                                  \
                 _84,                                                                                                  \
                 _85,                                                                                                  \
                 _86,                                                                                                  \
                 _87,                                                                                                  \
                 _88,                                                                                                  \
                 _89,                                                                                                  \
                 _90,                                                                                                  \
                 _91,                                                                                                  \
                 _92,                                                                                                  \
                 _93,                                                                                                  \
                 _94,                                                                                                  \
                 _95,                                                                                                  \
                 _96,                                                                                                  \
                 _97,                                                                                                  \
                 _98,                                                                                                  \
                 _99,                                                                                                  \
                 N,                                                                                                    \
                 ...)                                                                                                  \
    N

/* Allows overloading functions with multiple arguments
 * Limitations:
 *  - Up to 8 "extra arguments", should be way more than enough.
 *  - Cannot have parentheses: `()` in the arguments.
 *
 * Create some functions:
 *
 * void func_0(uint32_t arg1, uint32_t arg2);                                            // 0 extra args
 * void func_1(uint32_t arg1, uint32_t arg2, uint32_t extra_arg1);                       // 1 extra arg
 * void func_2(uint32_t arg1, uint32_t arg2, uint32_t extra_arg1, uint32_t extra_arg2);  // 2 extra args
 * etc...
 *
 * overload the functions:
 * #define func(arg1_, arg2_, ...) PP_OVERLOAD(func, __VA_ARGS__)(arg1_, arg2_, __VA_ARGS__)
 *                                                 ^ Notice the _n is implicit.
 *
 * call the functions using the macro:
 *
 * func(arg1, arg2);             -> func_0(arg1, arg2);             // 0 extra args
 * func(arg1, arg2, arg3);       -> func_1(arg1, arg2, arg3);       // 1 extra arg
 * func(arg1, arg2, arg3, arg4); -> func_2(arg1, arg2, arg3, arg4); // 2 extra args
 *  */
#define __OVERLOAD_CHOOSER(                                                                                            \
    f0_, f1_, f2_, f3_, f4_, f5_, f6_, f7_, f8_, f9_, f10_, f11_, f12_, f13_, f14_, f15_, f16_, ...)                   \
    f16_
#define __OVERLOAD_RECOMPOSER(args_with_paren_) __OVERLOAD_CHOOSER args_with_paren_
#define __OVERLOAD_CHOOSE_FROM_ARG_COUNT(f_, ...)                                                                      \
    __OVERLOAD_RECOMPOSER((__VA_ARGS__,                                                                                \
                           f_##_16,                                                                                    \
                           f_##_15,                                                                                    \
                           f_##_14,                                                                                    \
                           f_##_13,                                                                                    \
                           f_##_12,                                                                                    \
                           f_##_11,                                                                                    \
                           f_##_10,                                                                                    \
                           f_##_9,                                                                                     \
                           f_##_8,                                                                                     \
                           f_##_7,                                                                                     \
                           f_##_6,                                                                                     \
                           f_##_5,                                                                                     \
                           f_##_4,                                                                                     \
                           f_##_3,                                                                                     \
                           f_##_2,                                                                                     \
                           f_##_1, ))
#define __OVERLOAD_NO_ARGS_EXPAND(f_) , , , , , , , , , , , , , , , , f_##_0
#define PP_OVERLOAD(func_, ...)       __OVERLOAD_CHOOSE_FROM_ARG_COUNT(func_, __OVERLOAD_NO_ARGS_EXPAND __VA_ARGS__(func_))

/* Alternative overload macro, has different limitations.
 * Allows overloading functions with multiple arguments
 * Limitations:
 *  - Up to 99 "extra arguments", should be way more than enough.
 *  - Doesn't work with 0 extra arguments, can be overcome by turning a mandatory argument into an "extra argument".
 *    (`__VA_ARGS__` must contain at least one argument)
 *
 * Create some functions:
 *
 * void func_1(uint32_t arg1, uint32_t arg2);
 * void func_2(uint32_t arg1, uint32_t arg2, uint32_t extra_arg1);
 * void func_3(uint32_t arg1, uint32_t arg2, uint32_t extra_arg1, uint32_t extra_arg2);
 * etc...
 *
 * overload the functions:
 * #define func(arg1_, ...) PP_OVERLOAD_ALT(func, __VA_ARGS__)(arg1_, __VA_ARGS__)
 *                                              ^ Notice the _n is implicit.
 *
 * call the functions using the macro:
 *
 * func(arg1, arg2);             -> func_1(arg1, arg2);
 * func(arg1, arg2, arg3);       -> func_2(arg1, arg2, arg3);
 * func(arg1, arg2, arg3, arg4); -> func_3(arg1, arg2, arg3, arg4);
 *  */
#define PP_OVERLOAD_ALT(macro_, ...) PP_CONCAT(macro_##_, PP_NARG(__VA_ARGS__))

/* Same as above but starts from func_0 instead of func_1 - !!! still need at least one extra arg !!! */
#define PP_OVERLOAD_ALT_0(macro_, ...) PP_CONCAT(macro_##_, DEC(PP_NARG(__VA_ARGS__)))

#define PP_UNUSED_0()
#define PP_UNUSED_1(x1_)                                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
    } while (false);
#define PP_UNUSED_2(x1_, x2_)                                                                                          \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
    } while (false);
#define PP_UNUSED_3(x1_, x2_, x3_)                                                                                     \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
    } while (false);
#define PP_UNUSED_4(x1_, x2_, x3_, x4_)                                                                                \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
        (void)x4_;                                                                                                     \
    } while (false);
#define PP_UNUSED_5(x1_, x2_, x3_, x4_, x5_)                                                                           \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
        (void)x4_;                                                                                                     \
        (void)x5_;                                                                                                     \
    } while (false);
#define PP_UNUSED_6(x1_, x2_, x3_, x4_, x5_, x6_)                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
        (void)x4_;                                                                                                     \
        (void)x5_;                                                                                                     \
        (void)x6_;                                                                                                     \
    } while (false);
#define PP_UNUSED_7(x1_, x2_, x3_, x4_, x5_, x6_, x7_)                                                                 \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
        (void)x4_;                                                                                                     \
        (void)x5_;                                                                                                     \
        (void)x6_;                                                                                                     \
        (void)x7_;                                                                                                     \
    } while (false);
#define PP_UNUSED_8(x1_, x2_, x3_, x4_, x5_, x6_, x7_, x8_)                                                            \
    do                                                                                                                 \
    {                                                                                                                  \
        (void)x1_;                                                                                                     \
        (void)x2_;                                                                                                     \
        (void)x3_;                                                                                                     \
        (void)x4_;                                                                                                     \
        (void)x5_;                                                                                                     \
        (void)x6_;                                                                                                     \
        (void)x7_;                                                                                                     \
        (void)x8_;                                                                                                     \
    } while (false);

#define PRIMITIVE_CAT(a, ...) a##__VA_ARGS__

#define DEC(x) PRIMITIVE_CAT(DEC_, x)

#define IIF(cond)     PRIMITIVE_CAT(IIF_, cond)
#define IIF_0(t, ...) __VA_ARGS__
#define IIF_1(t, ...) t

#define COMPL(b) PRIMITIVE_CAT(COMPL_, b)
#define COMPL_0  1
#define COMPL_1  0

#define BITAND(x)   PRIMITIVE_CAT(BITAND_, x)
#define BITAND_0(y) 0
#define BITAND_1(y) y


#define CHECK_N(x, n, ...) n
#define CHECK(...)         CHECK_N(__VA_ARGS__, 0, )
#define PROBE(x)           x, 1

#define IS_PAREN(x)         CHECK(IS_PAREN_PROBE x)
#define IS_PAREN_PROBE(...) PROBE(~)

#define NOT(x) CHECK(PRIMITIVE_CAT(NOT_, x))
#define NOT_0  PROBE(~)

#define BOOL(x) COMPL(NOT(x))
#define IF(c)   IIF(BOOL(c))

#define EAT(...)
#define EXPAND(...) __VA_ARGS__
#define WHEN(c)     IF(c)(EXPAND, EAT)

#define EMPTY_FUNC()
#define DEFER(id)     id EMPTY_FUNC()
#define OBSTRUCT(...) __VA_ARGS__ DEFER(EMPTY_FUNC)()

#define EVAL(...)  EVAL1(EVAL1(EVAL1(__VA_ARGS__)))
#define EVAL1(...) EVAL2(EVAL2(EVAL2(__VA_ARGS__)))
#define EVAL2(...) EVAL3(EVAL3(EVAL3(__VA_ARGS__)))
#define EVAL3(...) EVAL4(EVAL4(EVAL4(__VA_ARGS__)))
#define EVAL4(...) EVAL5(EVAL5(EVAL5(__VA_ARGS__)))
#define EVAL5(...) __VA_ARGS__

/* A complex macro for repeating any macro different arguments multiple times, repeats `count_`.
   Each repetition gets n (0..count_) as its first param, followed by __VA_ARGS__ */
#define REPEAT_MACRO(count_, macro_, ...)                                                                              \
    WHEN(count_)                                                                                                       \
    (OBSTRUCT(REPEAT_MACRO_INDIRECT)()(DEC(count_), macro_, __VA_ARGS__)OBSTRUCT(macro_)(DEC(count_), __VA_ARGS__))

#define REPEAT_MACRO_INDIRECT() REPEAT_MACRO

#define WHILE(pred_, op_, ...)                                                                                         \
    IF(pred_(__VA_ARGS__))                                                                                             \
    (OBSTRUCT(WHILE_INDIRECT)()(pred_, op_, op_(__VA_ARGS__)), __VA_ARGS__)

#define WHILE_INDIRECT() WHILE

#define PRIMITIVE_COMPARE(x, y) IS_PAREN(COMPARE_##x(COMPARE_##y)(()))
#define IS_COMPARABLE(x)        IS_PAREN(PRIMITIVE_CAT(COMPARE_, x)(()))

#define NOT_EQUAL(x, y)                                                                                                \
    IIF(BITAND(IS_COMPARABLE(x))(IS_COMPARABLE(y)))                                                                    \
    (PRIMITIVE_COMPARE, 1 EAT)(x, y)

#define EQUAL(x, y) COMPL(NOT_EQUAL(x, y))

#define WHEN_ENABLED(config_macro) WHEN(IS_ENABLED(config_macro))

/* Based on zephyr's trick for the IS_ENABLED macro, refer to util_internal.h:22 to see their explanation */
#define AND(x, y) _AND1(_ZZZ##x##y)

#define _ZZZ11 _YYYY,

#define _AND1(one_or_two_args)       _AND2(one_or_two_args 1, 0)
#define _AND2(ignore_this, val, ...) val

#define WHEN_ENABLED_BOTH(config_macro1, config_macro2) WHEN(IS_ENABLED(AND(config_macro1, config_macro2)))

#define IS_TYPE(type_, obj_) _Generic((obj_), type_ : 1, default : 0)

#define sizeof_field(_type, _field) sizeof(((_type*)NULL)->_field)

/* ------------------------------------------------ Expansion Filler ------------------------------------------------ */

#define DEC_0   0
#define DEC_1   0
#define DEC_2   1
#define DEC_3   2
#define DEC_4   3
#define DEC_5   4
#define DEC_6   5
#define DEC_7   6
#define DEC_8   7
#define DEC_9   8
#define DEC_10  9
#define DEC_11  10
#define DEC_12  11
#define DEC_13  12
#define DEC_14  13
#define DEC_15  14
#define DEC_16  15
#define DEC_17  16
#define DEC_18  17
#define DEC_19  18
#define DEC_20  19
#define DEC_21  20
#define DEC_22  21
#define DEC_23  22
#define DEC_24  23
#define DEC_25  24
#define DEC_26  25
#define DEC_27  26
#define DEC_28  27
#define DEC_29  28
#define DEC_30  29
#define DEC_31  30
#define DEC_32  31
#define DEC_33  32
#define DEC_34  33
#define DEC_35  34
#define DEC_36  35
#define DEC_37  36
#define DEC_38  37
#define DEC_39  38
#define DEC_40  39
#define DEC_41  40
#define DEC_42  41
#define DEC_43  42
#define DEC_44  43
#define DEC_45  44
#define DEC_46  45
#define DEC_47  46
#define DEC_48  47
#define DEC_49  48
#define DEC_50  49
#define DEC_51  50
#define DEC_52  51
#define DEC_53  52
#define DEC_54  53
#define DEC_55  54
#define DEC_56  55
#define DEC_57  56
#define DEC_58  57
#define DEC_59  58
#define DEC_60  59
#define DEC_61  60
#define DEC_62  61
#define DEC_63  62
#define DEC_64  63
#define DEC_65  64
#define DEC_66  65
#define DEC_67  66
#define DEC_68  67
#define DEC_69  68
#define DEC_70  69
#define DEC_71  70
#define DEC_72  71
#define DEC_73  72
#define DEC_74  73
#define DEC_75  74
#define DEC_76  75
#define DEC_77  76
#define DEC_78  77
#define DEC_79  78
#define DEC_80  79
#define DEC_81  80
#define DEC_82  81
#define DEC_83  82
#define DEC_84  83
#define DEC_85  84
#define DEC_86  85
#define DEC_87  86
#define DEC_88  87
#define DEC_89  88
#define DEC_90  89
#define DEC_91  90
#define DEC_92  91
#define DEC_93  92
#define DEC_94  93
#define DEC_95  94
#define DEC_96  95
#define DEC_97  96
#define DEC_98  97
#define DEC_99  98
#define DEC_100 99

#define COMPARE_0(x)   x
#define COMPARE_1(x)   x
#define COMPARE_2(x)   x
#define COMPARE_3(x)   x
#define COMPARE_4(x)   x
#define COMPARE_5(x)   x
#define COMPARE_6(x)   x
#define COMPARE_7(x)   x
#define COMPARE_8(x)   x
#define COMPARE_9(x)   x
#define COMPARE_10(x)  x
#define COMPARE_11(x)  x
#define COMPARE_12(x)  x
#define COMPARE_13(x)  x
#define COMPARE_14(x)  x
#define COMPARE_15(x)  x
#define COMPARE_16(x)  x
#define COMPARE_17(x)  x
#define COMPARE_18(x)  x
#define COMPARE_19(x)  x
#define COMPARE_20(x)  x
#define COMPARE_21(x)  x
#define COMPARE_22(x)  x
#define COMPARE_23(x)  x
#define COMPARE_24(x)  x
#define COMPARE_25(x)  x
#define COMPARE_26(x)  x
#define COMPARE_27(x)  x
#define COMPARE_28(x)  x
#define COMPARE_29(x)  x
#define COMPARE_30(x)  x
#define COMPARE_31(x)  x
#define COMPARE_32(x)  x
#define COMPARE_33(x)  x
#define COMPARE_34(x)  x
#define COMPARE_35(x)  x
#define COMPARE_36(x)  x
#define COMPARE_37(x)  x
#define COMPARE_38(x)  x
#define COMPARE_39(x)  x
#define COMPARE_40(x)  x
#define COMPARE_41(x)  x
#define COMPARE_42(x)  x
#define COMPARE_43(x)  x
#define COMPARE_44(x)  x
#define COMPARE_45(x)  x
#define COMPARE_46(x)  x
#define COMPARE_47(x)  x
#define COMPARE_48(x)  x
#define COMPARE_49(x)  x
#define COMPARE_50(x)  x
#define COMPARE_51(x)  x
#define COMPARE_52(x)  x
#define COMPARE_53(x)  x
#define COMPARE_54(x)  x
#define COMPARE_55(x)  x
#define COMPARE_56(x)  x
#define COMPARE_57(x)  x
#define COMPARE_58(x)  x
#define COMPARE_59(x)  x
#define COMPARE_60(x)  x
#define COMPARE_61(x)  x
#define COMPARE_62(x)  x
#define COMPARE_63(x)  x
#define COMPARE_64(x)  x
#define COMPARE_65(x)  x
#define COMPARE_66(x)  x
#define COMPARE_67(x)  x
#define COMPARE_68(x)  x
#define COMPARE_69(x)  x
#define COMPARE_70(x)  x
#define COMPARE_71(x)  x
#define COMPARE_72(x)  x
#define COMPARE_73(x)  x
#define COMPARE_74(x)  x
#define COMPARE_75(x)  x
#define COMPARE_76(x)  x
#define COMPARE_77(x)  x
#define COMPARE_78(x)  x
#define COMPARE_79(x)  x
#define COMPARE_80(x)  x
#define COMPARE_81(x)  x
#define COMPARE_82(x)  x
#define COMPARE_83(x)  x
#define COMPARE_84(x)  x
#define COMPARE_85(x)  x
#define COMPARE_86(x)  x
#define COMPARE_87(x)  x
#define COMPARE_88(x)  x
#define COMPARE_89(x)  x
#define COMPARE_90(x)  x
#define COMPARE_91(x)  x
#define COMPARE_92(x)  x
#define COMPARE_93(x)  x
#define COMPARE_94(x)  x
#define COMPARE_95(x)  x
#define COMPARE_96(x)  x
#define COMPARE_97(x)  x
#define COMPARE_98(x)  x
#define COMPARE_99(x)  x
#define COMPARE_100(x) x

/* ------------------------------------------------ Taken from Zephyr ----------------------------------------------- */

/** @brief 0 if @p cond is true-ish; causes a compile error otherwise. */
#define ZERO_OR_COMPILE_ERROR(cond) ((int)sizeof(char[1 - 2 * !(cond)]) - 1)

/**
 * @brief Zero if @p array has an array type, a compile error otherwise
 *
 * This macro is available only from C, not C++.
 */
#define IS_ARRAY(array) ZERO_OR_COMPILE_ERROR(!__builtin_types_compatible_p(__typeof__(array), __typeof__(&(array)[0])))

/**
 * @brief Number of elements in the given @p array
 *
 * In C++, due to language limitations, this will accept as @p array
 * any type that implements <tt>operator[]</tt>. The results may not be
 * particulary meaningful in this case.
 *
 * In C, passing a pointer as @p array causes a compile error.
 */
#define ARRAY_SIZE(array) ((long)(IS_ARRAY(array) + (sizeof(array) / sizeof((array)[0]))))

/**
 * @brief Check if a pointer @p ptr lies within @p array.
 *
 * In C but not C++, this causes a compile error if @p array is not an array
 * (e.g. if @p ptr and @p array are mixed up).
 *
 * @param ptr a pointer
 * @param array an array
 * @return 1 if @p ptr is part of @p array, 0 otherwise
 */
#define PART_OF_ARRAY(array, ptr) ((ptr) && ((ptr) >= &array[0] && (ptr) < &array[ARRAY_SIZE(array)]))

/**
 * @brief Get a pointer to a container structure from an element
 *
 * Example:
 *
 *	struct foo {
 *		int bar;
 *	};
 *
 *	struct foo my_foo;
 *	int *ptr = &my_foo.bar;
 *
 *	struct foo *container = CONTAINER_OF(ptr, struct foo, bar);
 *
 * Above, @p container points at @p my_foo.
 *
 * @param ptr pointer to a structure element
 * @param type name of the type that @p ptr is an element of
 * @param field the name of the field within the struct @p ptr points to
 * @return a pointer to the structure that contains @p ptr
 */
#define CONTAINER_OF(ptr, type, field) ((type*)(((char*)(ptr)) - offsetof(type, field)))

/**
 * @brief Value of @p x rounded up to the next multiple of @p align,
 *        which must be a power of 2.
 */
#define ROUND_UP(x, align) (((unsigned long)(x) + ((unsigned long)(align)-1)) & ~((unsigned long)(align)-1))

/**
 * @brief Value of @p x rounded down to the previous multiple of @p
 *        align, which must be a power of 2.
 */
#define ROUND_DOWN(x, align) ((unsigned long)(x) & ~((unsigned long)(align)-1))

/** @brief Value of @p x rounded up to the next word boundary. */
#define WB_UP(x) ROUND_UP(x, sizeof(void*))

/** @brief Value of @p x rounded down to the previous word boundary. */
#define WB_DN(x) ROUND_DOWN(x, sizeof(void*))

/**
 * @def CLAMP
 * @brief Clamp a value to a given range.
 * @note Arguments are evaluated multiple times.
 */
#ifndef CLAMP
/* Use Z_CLAMP for a GCC-only, single evaluation version */
#define CLAMP(val, low, high) (((val) <= (low)) ? (low) : MIN(val, high))
#endif

/** @brief returns true if a value is in a given range, else returns false */
#define IN_RANGE(val, low, high) ((((val) >= (low)) && ((val) < (high))) ? true : false)

#define _CHECK_UNIQUE_1(a) 1

#define _CHECK_UNIQUE_2(a, b) ((a) != (b))

#define _CHECK_UNIQUE_3(a, b, c) (_CHECK_UNIQUE_2(a, b) && _CHECK_UNIQUE_2(c, b) && _CHECK_UNIQUE_2(a, c))

#define _CHECK_UNIQUE_4(a, b, c, d)                                                                                    \
    (_CHECK_UNIQUE_3(a, b, c) && _CHECK_UNIQUE_3(a, b, d) && _CHECK_UNIQUE_3(a, d, c) && _CHECK_UNIQUE_3(d, b, c))

#define _CHECK_UNIQUE_5(a, b, c, d, e)                                                                                 \
    (_CHECK_UNIQUE_4(a, b, c, d) && _CHECK_UNIQUE_4(a, b, c, e) && _CHECK_UNIQUE_4(a, b, e, d)                         \
     && _CHECK_UNIQUE_4(a, e, c, d) && _CHECK_UNIQUE_4(e, b, c, d))

#define _CHECK_UNIQUE_6(a, b, c, d, e, f)                                                                              \
    (_CHECK_UNIQUE_5(a, b, c, d, f) && _CHECK_UNIQUE_5(a, b, c, e, f) && _CHECK_UNIQUE_5(a, b, e, d, f)                \
     && _CHECK_UNIQUE_5(a, e, c, d, f) && _CHECK_UNIQUE_5(e, b, c, d, f) && _CHECK_UNIQUE_5(e, b, c, d, a))


#define CHECK_UNIQUE(...) PP_OVERLOAD(_CHECK_UNIQUE, __VA_ARGS__)(__VA_ARGS__)
