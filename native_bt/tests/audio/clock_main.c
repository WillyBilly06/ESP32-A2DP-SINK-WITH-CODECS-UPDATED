#include <stdio.h>
int clock_model_rates(void),clock_fractional_waveform(void),clock_real_pipeline(void);
int clock_anti_windup(void),clock_backpressure_relief(void),clock_switch_reset(void),clock_interval_jitter(void),clock_disabled(void),clock_spectral_tones(void),clock_drift_tone(void),clock_resampler_bounds_and_wrap(void),clock_resampler_drain(void);
int main(void){
    int (*tests[])(void)={clock_model_rates,clock_fractional_waveform,clock_real_pipeline,clock_anti_windup,clock_backpressure_relief,clock_switch_reset,clock_interval_jitter,clock_disabled,clock_spectral_tones,clock_drift_tone,clock_resampler_bounds_and_wrap,clock_resampler_drain};
    const char *names[]={"model_rates","fractional_waveform","real_pipeline","anti_windup","backpressure_relief","switch_reset","interval_jitter","disabled","spectral_tones","drift_tone","resampler_bounds_and_wrap","resampler_drain"};
    for(unsigned i=0;i<sizeof(tests)/sizeof(tests[0]);i++){int line=tests[i]();if(line){printf("FAIL %s line %d\n",names[i],line);return 1;}printf("PASS %s\n",names[i]);}
    return 0;
}
