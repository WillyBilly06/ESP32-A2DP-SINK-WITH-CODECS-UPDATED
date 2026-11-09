#ifndef LHDC_BIT_READER_H
#define LHDC_BIT_READER_H

#include "lhdc_dec_internal.h"

void lhdc_bit_reader_init(lhdc_dec_bit_reader_t *br, const uint8_t *data, size_t data_bytes);
uint32_t lhdc_bit_reader_read(lhdc_dec_bit_reader_t *br, int num_bits);
uint32_t lhdc_bit_reader_peek(lhdc_dec_bit_reader_t *br, int num_bits);
void lhdc_bit_reader_skip(lhdc_dec_bit_reader_t *br, int num_bits);
size_t lhdc_bit_reader_tell(const lhdc_dec_bit_reader_t *br);
size_t lhdc_bit_reader_remaining(const lhdc_dec_bit_reader_t *br);
void lhdc_bit_reader_align_byte(lhdc_dec_bit_reader_t *br);
int lhdc_bit_reader_read_flag(lhdc_dec_bit_reader_t *br);

#endif /* LHDC_BIT_READER_H */
