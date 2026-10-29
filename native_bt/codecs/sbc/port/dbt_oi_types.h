#ifndef DBT_OI_TYPES_H
#define DBT_OI_TYPES_H
#include <stdint.h>
/* Standalone codec port: the SDK's legacy x86 typedef uses long, which is 64
 * bits on Linux x64. Fixed widths keep decoder arithmetic identical to ESP32.
 * No Bluetooth protocol-stack configuration headers are included. */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "The standalone SBC port currently supports little-endian targets"
#endif
#define _OI_CPU_DEP_H
#define OI_BIG_ENDIAN_BYTE_ORDER 0
#define OI_LITTLE_ENDIAN_BYTE_ORDER 1
#define OI_CPU_BYTE_ORDER OI_LITTLE_ENDIAN_BYTE_ORDER
typedef int OI_BOOL;
typedef int OI_INT;
typedef unsigned OI_UINT;
typedef uint8_t OI_BYTE;
typedef int8_t OI_INT8;
typedef int16_t OI_INT16;
typedef int32_t OI_INT32;
typedef uint8_t OI_UINT8;
typedef uint16_t OI_UINT16;
typedef uint32_t OI_UINT32;
typedef uintptr_t OI_ELEMENT_UNION;
#endif
