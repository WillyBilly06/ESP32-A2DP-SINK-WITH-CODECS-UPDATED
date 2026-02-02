# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
    "main.cpp" 
    "ota/idf_update.cpp" 
    "led/led_strip_encoder.c"
)
set(COMPONENT_ADD_INCLUDEDIRS 
    "." 
    "audio" 
    "ble" 
    "config" 
    "core"
    "dsp" 
    "input" 
    "led" 
    "ota" 
    "storage"
)

register_component()

# Suppress warnings for compatibility with newer GCC
target_compile_options(${COMPONENT_LIB} PRIVATE 
    -DESP32_CMAKE=1 
    -Wno-error 
    -Wno-format 
    -fpermissive 
    -DA2DP_LEGACY_I2S_SUPPORT=0 
    -DA2DP_I2S_AUDIOTOOLS=1
    -Wno-overloaded-virtual
)