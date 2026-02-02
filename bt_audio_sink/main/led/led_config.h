#pragma once

// -----------------------------------------------------------
// LED Matrix Configuration
// 16x16 WS2812B matrix with audio-reactive effects
// Uses SPI DMA driver for reliable LED output
// -----------------------------------------------------------

#include "sdkconfig.h"

// Matrix dimensions from Kconfig (with defaults)
#ifdef CONFIG_LED_MATRIX_WIDTH
    #define LED_MATRIX_WIDTH    CONFIG_LED_MATRIX_WIDTH
#else
    #define LED_MATRIX_WIDTH    16
#endif

#ifdef CONFIG_LED_MATRIX_HEIGHT
    #define LED_MATRIX_HEIGHT   CONFIG_LED_MATRIX_HEIGHT
#else
    #define LED_MATRIX_HEIGHT   16
#endif

#define LED_MATRIX_COUNT    (LED_MATRIX_WIDTH * LED_MATRIX_HEIGHT)

// Hardware configuration from Kconfig (with defaults)
#ifdef CONFIG_LED_MATRIX_GPIO
    #define LED_DATA_PIN        CONFIG_LED_MATRIX_GPIO
    #define LED_GPIO_PIN        ((gpio_num_t)CONFIG_LED_MATRIX_GPIO)
#else
    #define LED_DATA_PIN        4
    #define LED_GPIO_PIN        GPIO_NUM_4
#endif

#ifdef CONFIG_LED_BRIGHTNESS
    #define LED_DEFAULT_BRIGHTNESS  CONFIG_LED_BRIGHTNESS
#else
    #define LED_DEFAULT_BRIGHTNESS  64
#endif

#ifdef CONFIG_LED_FPS
    #define LED_FPS             CONFIG_LED_FPS
#else
    #define LED_FPS             30
#endif

// Demo mode timeout (ms without audio before switching to demo)
#ifdef CONFIG_LED_DEMO_TIMEOUT
    #define LED_DEMO_TIMEOUT_MS     (CONFIG_LED_DEMO_TIMEOUT * 1000)
#else
    #define LED_DEMO_TIMEOUT_MS     5000
#endif

// RMT configuration
#define LED_RMT_CHANNEL     0
#define LED_RMT_MEM_BLOCKS  4  // 4 blocks should be enough with rmt_write_sample translator

// Color order (most WS2812B are GRB)
#define LED_COLOR_ORDER_GRB 1

// Effect IDs
enum LedEffectId {
    LED_EFFECT_SPECTRUM_BARS = 0,    // Classic spectrum analyzer bars
    LED_EFFECT_BEAT_PULSE,           // Full matrix pulse on beat
    LED_EFFECT_RIPPLE,               // Ripples from center on beat
    LED_EFFECT_FIRE,                 // Fire effect modulated by bass
    LED_EFFECT_PLASMA,               // Plasma waves modulated by audio
    LED_EFFECT_RAIN,                 // Matrix-style rain, speed by audio
    LED_EFFECT_VU_METER,             // Dual VU meter bars
    LED_EFFECT_STARFIELD,            // Stars that flash on beat
    LED_EFFECT_WAVE,                 // Sine wave visualization
    LED_EFFECT_FIREWORKS,            // Fireworks on beat
    LED_EFFECT_RAINBOW_WAVE,         // Rainbow that pulses with bass
    LED_EFFECT_PARTICLE_BURST,       // Particles explode from center on beat
    LED_EFFECT_KALEIDOSCOPE,         // Kaleidoscope pattern reactive to audio
    LED_EFFECT_FREQUENCY_SPIRAL,     // Spiral colored by frequency
    LED_EFFECT_BASS_REACTOR,         // Concentric rings react to bass
    // New effects
    LED_EFFECT_METEOR_SHOWER,        // Meteors fall down, triggered by bass
    LED_EFFECT_BREATHING,            // Smooth breathing/pulsing with audio
    LED_EFFECT_DNA_HELIX,            // DNA double helix rotating with audio
    LED_EFFECT_AUDIO_SCOPE,          // Oscilloscope-style waveform
    LED_EFFECT_BOUNCING_BALLS,       // Balls bounce to the beat
    LED_EFFECT_LAVA_LAMP,            // Blob-like lava lamp effect
    LED_EFFECT_AMBIENT,              // Ambient mode with configurable colors, gradient, speed
    // === User-selectable effects end here ===
    LED_EFFECT_USER_COUNT,           // Number of user-selectable effects (for cycling)
    // === Internal/special effects below ===
    LED_EFFECT_VOLUME = LED_EFFECT_USER_COUNT,  // Volume level visualizer (internal only)
    LED_EFFECT_COUNT                 // Total number of effects
};

// Gradient types for Ambient effect
enum LedGradientType {
    GRADIENT_NONE = 0,       // Solid color (use color1)
    GRADIENT_LINEAR_H,       // Horizontal linear gradient
    GRADIENT_LINEAR_V,       // Vertical linear gradient
    GRADIENT_RADIAL,         // Radial gradient from center
    GRADIENT_DIAGONAL,       // Diagonal gradient
    GRADIENT_COUNT
};

// Demo mode sub-effects (cycle through when no audio)
enum LedDemoMode {
    LED_DEMO_RAINBOW_CYCLE = 0,
    LED_DEMO_COLOR_WIPE,
    LED_DEMO_THEATER_CHASE,
    LED_DEMO_GRADIENT_FLOW,
    LED_DEMO_SPARKLE,
    LED_DEMO_COUNT
};
