#include "lhdc_bit_reader.h"
#include <string.h>

/* Bit reader called on essentially every coefficient/symbol read -> keep the tiny
 * hot fns in IRAM to avoid flash cache-miss stalls under BT IRQs. (HOST: no-op.) */
#if !defined(LHDC_HOST_BUILD)
  #include "esp_attr.h"
  #define LHDC_HOT IRAM_ATTR
#else
  #define LHDC_HOT
#endif

void lhdc_bit_reader_init(lhdc_dec_bit_reader_t *br, const uint8_t *data, size_t data_bytes)
{
    br->data = data;
    br->data_bytes = data_bytes;
    br->byte_pos = 0;
    br->cache = 0;
    br->cache_bits = 0;
}

static LHDC_HOT void lhdc_bit_reader_refill(lhdc_dec_bit_reader_t *br, int need_bits)
{
    while (br->cache_bits < need_bits && br->byte_pos < br->data_bytes) {
        br->cache = (br->cache << 8) | br->data[br->byte_pos++];
        br->cache_bits += 8;
    }
}

LHDC_HOT uint32_t lhdc_bit_reader_read(lhdc_dec_bit_reader_t *br, int num_bits)
{
    if (num_bits <= 0) return 0;
    if (num_bits > 32) num_bits = 32;

    lhdc_bit_reader_refill(br, num_bits);

    if (br->cache_bits < num_bits) {
        int shift = br->cache_bits;
        uint32_t val = (uint32_t)(br->cache & ((1ULL << shift) - 1));
        br->cache = 0;
        br->cache_bits = 0;
        return val;
    }

    int shift = br->cache_bits - num_bits;
    uint32_t val = (uint32_t)((br->cache >> shift) & ((1ULL << num_bits) - 1));
    br->cache_bits = shift;
    br->cache &= (1ULL << shift) - 1;
    return val;
}

LHDC_HOT uint32_t lhdc_bit_reader_peek(lhdc_dec_bit_reader_t *br, int num_bits)
{
    if (num_bits <= 0) return 0;
    if (num_bits > 32) num_bits = 32;

    lhdc_bit_reader_refill(br, num_bits);

    if (br->cache_bits < num_bits) {
        return (uint32_t)(br->cache & ((1ULL << br->cache_bits) - 1));
    }

    int shift = br->cache_bits - num_bits;
    return (uint32_t)((br->cache >> shift) & ((1ULL << num_bits) - 1));
}

LHDC_HOT void lhdc_bit_reader_skip(lhdc_dec_bit_reader_t *br, int num_bits)
{
    if (num_bits <= 0) return;

    lhdc_bit_reader_refill(br, num_bits);

    if (br->cache_bits >= num_bits) {
        int shift = br->cache_bits - num_bits;
        br->cache_bits = shift;
        br->cache &= (1ULL << shift) - 1;
    } else {
        br->cache = 0;
        br->cache_bits = 0;
    }
}

size_t lhdc_bit_reader_tell(const lhdc_dec_bit_reader_t *br)
{
    return br->byte_pos * 8 - br->cache_bits;
}

size_t lhdc_bit_reader_remaining(const lhdc_dec_bit_reader_t *br)
{
    return (br->data_bytes - br->byte_pos) * 8 + br->cache_bits;
}

void lhdc_bit_reader_align_byte(lhdc_dec_bit_reader_t *br)
{
    int rem = br->cache_bits % 8;
    if (rem) {
        lhdc_bit_reader_skip(br, rem);
    }
}

LHDC_HOT int lhdc_bit_reader_read_flag(lhdc_dec_bit_reader_t *br)
{
    return (int)lhdc_bit_reader_read(br, 1);
}
