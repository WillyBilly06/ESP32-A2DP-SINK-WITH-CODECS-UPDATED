#pragma once

#include "libFDK/FDK_archdef.h"
#include "libSYS/machine_type.h"

#define FUNCTION_fixmuldiv2_DD
#define FUNCTION_fixmul_DD
#define FUNCTION_fixmuldiv2BitExact_DD
#define FUNCTION_fixmulBitExact_DD
#define FUNCTION_fixpow2div2_D
#define FUNCTION_fixpow2_D
#define fixmuldiv2BitExact_DD(a, b) fixmuldiv2_DD(a, b)
#define fixmulBitExact_DD(a, b) fixmul_DD(a, b)
#define fixpow2div2_D(a) fixmuldiv2_DD(a, a)
#define fixpow2_D(a) fixmul_DD(a, a)

inline LONG fixmuldiv2_DD(const LONG a, const LONG b) {
  LONG result1 = 0;
  asm("MULSH %0, %1, %2;" : "=a"(result1) : "a"(a), "a"(b));
  return result1;
}

inline LONG fixmul_DD(const LONG a, const LONG b) {
  LONG result1 = 0;
  asm("MULSH %[res], %[val1], %[val2]; \
       SLLI %[res], %[res], 1;"
        : [res] "=a"(result1) 
        : [val1] "a" (a), [val2] "a" (b));
  return result1;
}
