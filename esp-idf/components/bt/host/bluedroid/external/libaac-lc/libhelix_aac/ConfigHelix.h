/* ConfigHelix.h - ESP32 IDF Configuration for Helix AAC Decoder */

#ifndef CONFIG_HELIX_H
#define CONFIG_HELIX_H

/* Use standard library (required for ESP-IDF) */
#ifndef USE_DEFAULT_STDLIB
#define USE_DEFAULT_STDLIB
#endif

/* Disable SBR to save memory - we only need AAC-LC for A2DP */
/* #define HELIX_FEATURE_AUDIO_CODEC_AAC_SBR */

/* Buffer sizes */
#ifndef AAC_MAX_OUTPUT_SIZE
#define AAC_MAX_OUTPUT_SIZE (1024 * 8)
#endif

#ifndef AAC_MAX_FRAME_SIZE
#define AAC_MAX_FRAME_SIZE 2100
#endif

#ifndef AAC_MIN_FRAME_SIZE
#define AAC_MIN_FRAME_SIZE 1024
#endif

/* Logging - use ESP-IDF logger */
#define HELIX_LOGGING_ACTIVE 1
#define USE_IDF_LOGGER

#endif /* CONFIG_HELIX_H */
