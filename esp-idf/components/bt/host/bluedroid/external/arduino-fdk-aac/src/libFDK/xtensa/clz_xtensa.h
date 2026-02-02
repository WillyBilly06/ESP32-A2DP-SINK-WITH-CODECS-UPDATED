#pragma once

#include "libFDK/FDK_archdef.h"
#include "libSYS/machine_type.h"

#define FUNCTION_fixnormz_D
#define FUNCTION_fixnorm_D

inline INT fixnormz_D(LONG value) {
  INT result1 = 0;
  asm("NSAU %0, %1;" : "=a"(result1) : "a"(value));
  return result1;
}

inline INT fixnorm_D(LONG value) {
  INT result1 = 0;
  asm("NSA %0, %1;" : "=a"(result1) : "a"(value));
  return result1;
}
