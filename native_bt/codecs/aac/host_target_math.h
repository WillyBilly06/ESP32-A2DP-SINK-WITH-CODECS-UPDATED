#ifndef DBT_AAC_HOST_TARGET_MATH_H
#define DBT_AAC_HOST_TARGET_MATH_H
/* Use the SDK's portable ESP32 arithmetic when testing on Windows. Its
 * _WIN32 branch contains legacy x86 inline assembly without a C return and
 * is not a reliable x64 reference. Keep platform macros unchanged afterward. */
#if defined(_WIN32) && !defined(__XTENSA__)
#pragma push_macro("_WIN32")
#pragma push_macro("ESP32")
#undef _WIN32
#undef ESP32
#define ESP32 1
#include "assembly.h"
#pragma pop_macro("ESP32")
#pragma pop_macro("_WIN32")
#endif
#endif
