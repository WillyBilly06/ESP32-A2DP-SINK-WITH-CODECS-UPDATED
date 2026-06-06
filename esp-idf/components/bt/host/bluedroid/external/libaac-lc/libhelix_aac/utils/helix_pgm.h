/* helix_pgm.h - ESP32 IDF version (no PROGMEM needed) */

#ifndef HELIX_PGM_H
#define HELIX_PGM_H

/* ESP32 has no PROGMEM - all memory is directly accessible */
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))

#endif /* HELIX_PGM_H */
