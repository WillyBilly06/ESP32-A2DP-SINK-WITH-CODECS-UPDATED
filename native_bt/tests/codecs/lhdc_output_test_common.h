#ifndef LHDC_OUTPUT_TEST_COMMON_H
#define LHDC_OUTPUT_TEST_COMMON_H
#include <stdint.h>
typedef struct {uint32_t peak,clipped,concealed,nonfinite;} output_event_t;
int32_t reference_sample(uint32_t bits);
int32_t candidate_sample(uint32_t bits);
void reference_emit(const float *,const float *,int32_t *,int,float);
void candidate_emit(const float *,const float *,int32_t *,int,float);
void reference_frame(const float *,const float *,void *,int,int,int,float,output_event_t *,float *,float *);
void candidate_frame(const float *,const float *,void *,int,int,int,float,output_event_t *,float *,float *);
#endif
