cmake_minimum_required(VERSION 3.15)

idf_component_register(SRCS src/ldacBT.c
                            src/ldaclib.c
                       INCLUDE_DIRS inc
                       PRIV_INCLUDE_DIRS inc src)

# LDAC Decoder Performance Optimizations for ESP32
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -Werror=implicit-function-declaration
    # Maximum optimization level
    -O3
    # Fast math - allows reordering of float operations for speed
    -ffast-math
    # Single precision only - ESP32 has no double FPU
    -fsingle-precision-constant
    # Aggressive loop optimizations
    -funroll-loops
    -fomit-frame-pointer
    # Better inlining for small functions
    -finline-functions
    -finline-limit=300
    # Link-time optimization hint
    -flto=auto
    # ESP32 specific - use hardware FPU
    -mfpu=auto
)

