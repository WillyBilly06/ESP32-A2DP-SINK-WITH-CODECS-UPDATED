#ifndef _LDAC_H
#define _LDAC_H

#include "ldaclib.h"
#include "struct_ldac.h"

/***************************************************************************************************
    Macro Definitions
***************************************************************************************************/
/* Configuration */
#define LDAC_SYNCWORDBITS    8
#define LDAC_SYNCWORD        0xAA
/** Sampling Rate **/
#define LDAC_SMPLRATEBITS    3
#define LDAC_NSMPLRATEID     6
#define LDAC_NSUPSMPLRATEID  4
#define LDAC_SMPLRATEID_0    0x0
#define LDAC_SMPLRATEID_1    0x1
#define LDAC_SMPLRATEID_2    0x2
#define LDAC_SMPLRATEID_3    0x3
/** Channel **/
#define LDAC_CHCONFIG1BITS   3
#define LDAC_CHCONFIG2BITS   2
#define LDAC_NCHCONFIGID     8
#define LDAC_MAXNCH          2
#define LDAC_CHANNEL_1CH     1
#define LDAC_CHANNEL_2CH     2
#define LDAC_CHCONFIGID_MN   0
#define LDAC_CHCONFIGID_DL   1
#define LDAC_CHCONFIGID_ST   2
/** Frame Length **/
#define LDAC_FRAMELEN1BITS   11
#define LDAC_FRAMELEN2BITS   9
#define LDAC_MAXNBYTES       1024
#define LDAC_MAXSUPNBYTES    512
#define LDAC_MINSUPNBYTES    22
/** Frame Status **/
#define LDAC_FRAMESTATBITS   2
#define LDAC_FRMSTAT_LEV_0   0
#define LDAC_FRMSTAT_LEV_1   1
#define LDAC_FRMSTAT_LEV_2   2
#define LDAC_FRMSTAT_LEV_3   3
/** Other **/
#define LDAC_RESERVE1BITS    2
#define LDAC_RESERVE2BITS    5
#define LDAC_DUMMYCODE       0x00

/* Signal Processing */
#define LDAC_NFRAME          2
#define LDAC_NSFTSTEP        5
/** Frame Samples (log base 2 of) **/
#define LDAC_NUMLNN          2
#define LDAC_MAXLNN          8
#define LDAC_2FSLNN          8
#define LDAC_1FSLNN          7
/** Frame Samples **/
#define LDAC_MAXLSU          (1 << LDAC_MAXLNN)
#define LDAC_2FSLSU          (1 << LDAC_2FSLNN)
#define LDAC_1FSLSU          (1 << LDAC_1FSLNN)
/** Band **/
#define LDAC_MAXNBANDS       16
#define LDAC_2FSNBANDS       16
#define LDAC_1FSNBANDS       12
/** QU **/
#define LDAC_MAXGRADQU       50
#define LDAC_MAXNQUS         34
#define LDAC_2FSNQUS         34
#define LDAC_1FSNQUS         26
#define LDAC_MAXNSPS         16
/** Frame Status Analysis **/
#define LDAC_NSP_PSEUDOANA   128
#define LDAC_NSP_LOWENERGY   12
#define LDAC_TH_ZCROSNUM     90
#define LDAC_MAXCNT_FRMANA   10

/* Stream Syntax */
#define LDAC_BLKID_MONO      0
#define LDAC_BLKID_STEREO    1
#define LDAC_FILLCODE        0x01
/** Band Info **/
#define LDAC_NBANDBITS       4
#define LDAC_BAND_OFFSET     2
/** Gradient Data **/
#define LDAC_GRADMODEBITS    2
#define LDAC_GRADOSBITS      5
#define LDAC_MAXGRADOS       31
#define LDAC_DEFGRADOSH      31
#define LDAC_GRADQU0BITS     6
#define LDAC_GRADQU1BITS     5
#define LDAC_DEFGRADQUH      26
#define LDAC_NADJQUBITS      5
#define LDAC_MAXNADJQUS      32
/** Scale Factor Data **/
#define LDAC_IDSFBITS        5
#define LDAC_NIDSF           32
#define LDAC_SFCMODEBITS     1
#define LDAC_NSFCMODE        2
#define LDAC_SFCWTBLBITS     3
#define LDAC_NSFCWTBL        8
#define LDAC_SFCBLENBITS     2
#define LDAC_MINSFCBLEN_0    3
#define LDAC_MAXSFCBLEN_0    6
#define LDAC_MINSFCBLEN_1    2
#define LDAC_MAXSFCBLEN_1    5
#define LDAC_MINSFCBLEN_2    2
#define LDAC_MAXSFCBLEN_2    5
/** Spectrum/Residual Data **/
#define LDAC_NIDWL           16
#define LDAC_MINIDWL1        1
#define LDAC_MAXIDWL1        15
#define LDAC_MAXIDWL2        15
#define LDAC_2DIMSPECBITS    3
#define LDAC_N2DIMSPECENCTBL 16
#define LDAC_N2DIMSPECDECTBL 8
#define LDAC_4DIMSPECBITS    7
#define LDAC_N4DIMSPECENCTBL 256
#define LDAC_N4DIMSPECDECTBL 81
/** Bit Operation **/
#define LDAC_LOC_SHIFT       3
#define LDAC_LOC_MASK        0x7
#define LDAC_BYTESIZE        8
#define LDAC_MAXBITNUM       8192

/* Flag */
#define LDAC_FLAGBITS        1
#define LDAC_TRUE            1
#define LDAC_FALSE           0

/* Mode */
#define LDAC_MODE_0          0
#define LDAC_MODE_1          1
#define LDAC_MODE_2          2
#define LDAC_MODE_3          3

/***************************************************************************************************
    Structure Definitions
***************************************************************************************************/
typedef struct _sfinfo_ldac SFINFO;
typedef struct _config_info_ldac CFG;
typedef struct _audio_block_ldac AB;
typedef struct _audio_channel_ldac AC;
typedef struct _audio_channel_sub_ldac ACSUB;

/* Configuration Information Structure */
struct _config_info_ldac {
  int syncword;
  int smplrate_id;
  int chconfig_id;
  int ch;
  int frame_length;
  int frame_status;
};

/* Audio Channel (AC) Sub Structure - MEMORY OPTIMIZED */
struct _audio_channel_sub_ldac {
  SCALAR a_time[LDAC_MAXLSU * LDAC_NFRAME];  /* 256*2*4 = 2048 bytes - needed for overlap-add */
  SCALAR a_spec[LDAC_MAXLSU];                 /* 256*4 = 1024 bytes */
};

/* Audio Channel (AC) Structure - MEMORY OPTIMIZED 
 * Reduced int arrays to int8_t/int16_t where value ranges allow:
 * - a_idsf: scale factors 0-31, fits in int8_t
 * - a_idwl1/2: word lengths 0-15, fits in int8_t  
 * - a_addwl: additional word length 0-5, fits in int8_t
 * - a_qspec/rspec: quantized spectrum, fits in int16_t
 */
struct _audio_channel_ldac {
  /* Frequently accessed - keep at top for cache locality */
  ACSUB* p_acsub;
  AB* p_ab;
  
  /* Small integer fields */
  int8_t ich;
  int8_t frmana_cnt;
  int8_t sfc_mode;
  int8_t sfc_bitlen;
  int8_t sfc_offset;
  int8_t sfc_weight;
  int8_t ext_size;
  int8_t _pad1;  /* Alignment padding */
  
  /* Quantization unit arrays - reduced from int to int8_t (saves 5*34*3 = 510 bytes/ch) */
  int8_t a_idsf[LDAC_MAXNQUS];    /* Scale factors: 0-31 */
  int8_t a_idwl1[LDAC_MAXNQUS];   /* Word length 1: 1-15 */
  int8_t a_idwl2[LDAC_MAXNQUS];   /* Word length 2: 0-15 */
  int8_t a_addwl[LDAC_MAXNQUS];   /* Additional word length: 0-5 */
  int8_t a_tmp[LDAC_MAXNQUS];     /* Temp array */
  int8_t _pad2[2];  /* Alignment to 4 bytes */
  
  /* Quantized spectrum - reduced from int to int16_t (saves 256*2*2 = 1024 bytes/ch) */
  int16_t a_qspec[LDAC_MAXLSU];   /* Quantized spectrum */
  int16_t a_rspec[LDAC_MAXLSU];   /* Residual spectrum */
};

/* Audio Block (AB) Structure - MEMORY OPTIMIZED */
struct _audio_block_ldac {
  /* Frequently accessed pointers first */
  AC* ap_ac[2];
  int* p_smplrate_id;
  int* p_error_code;
  
  /* Gradient array - reduced from int to int8_t (saves 50*3 = 150 bytes) */
  int8_t a_grad[LDAC_MAXGRADQU];
  
  /* Small fields - use int8_t where possible */
  int8_t blk_type;
  int8_t blk_nchs;
  int8_t nbands;
  int8_t nqus;
  int8_t ext_flag;
  int8_t ext_mode;
  int8_t grad_mode;
  int8_t grad_qu_l;
  int8_t grad_qu_h;
  int8_t grad_os_l;
  int8_t grad_os_h;
  int8_t nadjqus;
  int8_t abc_status;
  int8_t _pad[3];  /* Alignment */
  
  /* Bit counts - keep as int for calculations */
  int nbits_ab;
  int nbits_band;
  int nbits_grad;
  int nbits_scfc;
  int nbits_spec;
  int nbits_avail;
  int nbits_used;
};

/* Sound Frame Structure */
struct _sfinfo_ldac {
  CFG cfg;
  AB* p_ab;
  AC* ap_ac[LDAC_MAXNCH];
  char* p_mempos;
  int error_code;
};

/* LDAC Handle */
typedef struct _handle_ldac_struct {
  int nlnn;
  int nbands;
  int grad_mode;
  int grad_qu_l;
  int grad_qu_h;
  int grad_os_l;
  int grad_os_h;
  int abc_status;
  int error_code;
  SFINFO sfinfo;
} HANDLE_LDAC_STRUCT;

/* Huffman Codeword */
typedef struct {
  unsigned char word;
  unsigned char len;
} HC;

typedef struct _hcdec_ldac HCDEC;
struct _hcdec_ldac {
  const HC* p_tbl;
  unsigned char ncodes;
  unsigned char wl;
  unsigned char mask;
  unsigned char maxlen;
  const unsigned char* p_dec;
};
/*******************************************************************************
    Function Declarations
*******************************************************************************/
#define npow2_ldac(n)   (1 << (n))
#define min_ldac(a, b)  (((a) < (b)) ? (a) : (b))
#define max_ldac(a, b)  (((a) > (b)) ? (a) : (b))

/* Get Huffman Codeword Property */
#define hc_len_ldac(p)  ((p)->len)
#define hc_word_ldac(p) ((p)->word)

/* Convert a Signed Number with nbits to a Signed Integer */
#define bs_to_int_ldac(bs, nbits) \
  (((bs) & (0x1 << ((nbits)-1))) ? ((bs) | ((~0x0) << (nbits))) : bs)

#include "proto_ldac.h"
#endif /* _LDAC_H */
