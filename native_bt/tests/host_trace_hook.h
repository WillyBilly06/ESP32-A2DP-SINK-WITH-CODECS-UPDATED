#ifndef PROBE_HOST_TRACE_TEST_HOOK_H
#define PROBE_HOST_TRACE_TEST_HOOK_H
void probe_audio_trace_test_part(unsigned part);
#define PROBE_AUDIO_TRACE_TEST_PART(part) probe_audio_trace_test_part(part)
#endif
