#ifndef _STRUCT_H
#define _STRUCT_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/***************************************************************************************************
    Macro Definition
***************************************************************************************************/

#define DECLFUNC static
#define UNUSED_ATTR __attribute__((unused))

/* Optimization hints for ESP32 */
#ifndef LDAC_LIKELY
#define LDAC_LIKELY(x)   __builtin_expect(!!(x), 1)
#endif
#ifndef LDAC_UNLIKELY
#define LDAC_UNLIKELY(x) __builtin_expect(!!(x), 0)
#endif

/* Force inline for critical functions */
#ifndef LDAC_ALWAYS_INLINE
#define LDAC_ALWAYS_INLINE __attribute__((always_inline)) inline
#endif

/* Cache line alignment for ESP32 (32 bytes) */
#ifndef LDAC_CACHE_ALIGNED
#define LDAC_CACHE_ALIGNED __attribute__((aligned(32)))
#endif

/* Hot function attribute - place in IRAM if possible */
#ifndef LDAC_HOT
#define LDAC_HOT __attribute__((hot))
#endif

#ifndef PI
#ifdef M_PI
#define PI M_PI
#else /* M_PI */
#define PI (double)(3.14159265358979323846)
#endif /* M_PI */
#endif /* PI */

/***************************************************************************************************
    Type Definition
***************************************************************************************************/
typedef unsigned char STREAM;

typedef short          INT16;
typedef int            INT32;
typedef unsigned int  UINT32;
typedef long long      INT64;

/* ESP32 has single-precision FPU only - always use float */
#if CONFIG_SCALAR_F64
typedef double        SCALAR;
#define _scalar(x)         x
#else
typedef float         SCALAR;
#define _scalar(x)       x##f
#endif



typedef union {
    float f;
    int i;
} IEEE754_FI;

/***************************************************************************************************
    Macro Functions
***************************************************************************************************/
/* Buffer Operations */
#define clear_data_ldac(p, n)      memset((p), 0, (n))
#define clear_seq_s_ldac(p, n)     memset((char *)(p), 0, (n)*sizeof(short))
#define clear_seq_l_ldac(p, n)     memset((char *)(p), 0, (n)*sizeof(int))
#define clear_seq_f_ldac(p, n)     memset((char *)(p), 0, (n)*sizeof(SCALAR))

#if _MSC_VER >=1400
/* Secured CRT Functions */
#define copy_data_ldac(p1, p2, n)  memcpy_s((p2), (n), (p1), (n))
#define copy_seq_s_ldac(p1, p2, n) memcpy_s((p2), (n)*sizeof(short), (p1), (n)*sizeof(short))
#define copy_seq_l_ldac(p1, p2, n) memcpy_s((p2), (n)*sizeof(int), (p1), (n)*sizeof(int))
#define copy_seq_f_ldac(p1, p2, n) memcpy_s((p2), (n)*sizeof(SCALAR), (p1), (n)*sizeof(SCALAR))
#define move_seq_f_ldac(p1, p2, n) memmove_s((p2), (n)*sizeof(SCALAR), (p1), (n)*sizeof(SCALAR))
#else
#define copy_data_ldac(p1, p2, n)  memcpy((p2), (p1), (n))
#define copy_seq_s_ldac(p1, p2, n) memcpy((p2), (p1), (n)*sizeof(short))
#define copy_seq_l_ldac(p1, p2, n) memcpy((p2), (p1), (n)*sizeof(int))
#define copy_seq_f_ldac(p1, p2, n) memcpy((p2), (p1), (n)*sizeof(SCALAR))
#define move_seq_f_ldac(p1, p2, n) memmove((p2), (p1), (n)*sizeof(SCALAR))
#endif

#endif /* _STRUCT_H */

