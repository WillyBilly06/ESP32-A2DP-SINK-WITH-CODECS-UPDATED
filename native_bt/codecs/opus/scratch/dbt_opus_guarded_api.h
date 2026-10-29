#ifndef DBT_OPUS_GUARDED_API_H
#define DBT_OPUS_GUARDED_API_H
#include "opus.h"
/* This symbol is absent from an unguarded VLA decoder build. */
int dbt_opus_arena_decode(OpusDecoder *, const unsigned char *, opus_int32,
                          opus_int16 *, int, int);
#endif
