#ifndef DBT_LHDC_ALLOC_H
#define DBT_LHDC_ALLOC_H
#include <stddef.h>
/* Implemented by the one-active-codec worker adapter. */
void *dbt_lhdc_mem_alloc(size_t size);
void dbt_lhdc_mem_free(void *ptr);
int dbt_lhdc_entropy_ready(void);
#ifdef DBT_LHDC_QUALITY
/* Single decoder-owner notifications; only capture metadata here. */
void dbt_lhdc_channel_failure(unsigned channel,int result);
void dbt_lhdc_frame_quality(float peak,unsigned clipped_samples,int concealed);
#endif
#endif
