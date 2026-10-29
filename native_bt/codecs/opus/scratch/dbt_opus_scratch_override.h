#ifndef DBT_OPUS_SCRATCH_OVERRIDE_H
#define DBT_OPUS_SCRATCH_OVERRIDE_H
/* Force-include for every Opus translation unit in the selected build. */
#include "dbt_opus_scratch.h"
#ifdef DBT_OPUS_GUARDED_API
#define opus_decode dbt_opus_arena_decode
#endif
#define STACK_ALLOC_H
#define VARDECL(type, var) type *var
#define ALLOC(var, count, type) ((var) = (type *)dbt_opus_scratch_push((size_t)(count), sizeof(type), _Alignof(type)))
#define SAVE_STACK size_t _saved_stack = dbt_opus_scratch_mark()
#define ALLOC_STACK SAVE_STACK
#define RESTORE_STACK dbt_opus_scratch_restore(_saved_stack)
#define ALLOC_NONE 1
#define OPUS_CHECK_ARRAY(ptr, len) 0
#define OPUS_CHECK_VALUE(value) 0
#define OPUS_CHECK_ARRAY_COND(ptr, len) 0
#define OPUS_PRINT_INT(value) ((void)0)
#define OPUS_FPRINTF (void)
#endif
