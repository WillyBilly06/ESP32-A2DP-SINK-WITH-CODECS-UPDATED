#define _POSIX_C_SOURCE 200112L
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
static size_t live_bytes, peak_bytes;
static int fail_next_allocation;
static void *workspace_pointer;
static size_t workspace_size;
void *heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps) {
    (void)caps;
    if (fail_next_allocation) { fail_next_allocation = 0; return NULL; }
    void *pointer = NULL;
    if (posix_memalign(&pointer, alignment, size)) return NULL;
    if (workspace_pointer) abort();
    workspace_pointer = pointer; workspace_size = size;
    live_bytes += size;
    if (live_bytes > peak_bytes) peak_bytes = live_bytes;
    return pointer;
}
void heap_caps_free(void *pointer) {
    if (!pointer) return;
    if (pointer != workspace_pointer) abort();
    live_bytes -= workspace_size;
    workspace_pointer = NULL; workspace_size = 0;
    free(pointer);
}
#include "aac_decoder_helix.c"
static aac_decoder_t decoder;
int probe_open(int rate, int channels) {
    int status = aac_decoder_init(&decoder);
    return status == AAC_OK ? aac_decoder_configure(&decoder, rate, channels) : status;
}
void probe_close(void) { aac_decoder_deinit(&decoder); }
int probe_decode(const uint8_t *data, int size, int16_t *pcm, int *samples) {
    return aac_decoder_decode(&decoder, data, size, pcm, samples);
}
size_t probe_live(void) { return live_bytes; }
size_t probe_peak(void) { return peak_bytes; }
size_t probe_resident_workspace(void) {
#if AAC_CANDIDATE
    return s_helix_buffer ? HELIX_BUFFER_SIZE : 0;
#else
    return HELIX_BUFFER_SIZE;
#endif
}
void probe_fail_next(void) { fail_next_allocation = 1; }

#ifdef AAC_SANITIZER_MAIN
int main(int argc, char **argv) {
    if (argc != 4 || probe_open(atoi(argv[2]), atoi(argv[3]))) return 1;
    FILE *file = fopen(argv[1], "rb");
    if (!file) return 2;
    static uint8_t frame[8192];
    static int16_t pcm[2048];
    unsigned decoded = 0;
    for (;;) {
        uint8_t header[7];
        if (fread(header, 1, 7, file) != 7) break;
        if (header[0] != 0xff || (header[1] & 0xf0) != 0xf0) return 3;
        unsigned length = ((header[3] & 3u) << 11) | ((unsigned)header[4] << 3) | (header[5] >> 5);
        if (length < 7 || length - 7 > sizeof(frame) || !(header[1] & 1)) return 4;
        if (fread(frame, 1, length - 7, file) != length - 7) return 5;
        int samples = 0;
        if (probe_decode(frame, (int)length - 7, pcm, &samples) != AAC_OK || samples != 1024) return 6;
        ++decoded;
    }
    fclose(file);
    probe_close(); probe_close();
    if (!decoded || probe_live()) return 7;
#if AAC_CANDIDATE
    if (probe_resident_workspace()) return 8;
    probe_fail_next();
    if (probe_open(48000, 2) != AAC_ERR_NOMEM || probe_live()) return 9;
    if (probe_open(48000, 2) || probe_open(44100, 1)) return 10;
    probe_close();
    if (probe_live()) return 11;
#endif
    return 0;
}
#endif
