#ifndef _PROTO_LDAC_H
#define _PROTO_LDAC_H

/***************************************************************************************************
    Function Declarations
***************************************************************************************************/
/* decode_ldac.c */
DECLFUNC LDAC_RESULT init_decode_ldac(SFINFO*);
DECLFUNC void free_decode_ldac(SFINFO*);
DECLFUNC void decode_ldac(SFINFO*);

/* setpcm_ldac.c */
DECLFUNC void set_output_pcm_ldac(SFINFO*, void*[], LDAC_SMPL_FMT_T, int);

/* mdct_ldac.c */
DECLFUNC void proc_imdct_ldac(SFINFO*, int);

/* bitalloc_sub_ldac.c */
DECLFUNC void calc_add_word_length_ldac(AC*);
DECLFUNC void reconst_gradient_ldac(AB*, int, int);
DECLFUNC void reconst_word_length_ldac(AC*);

/* dequant_ldac.c */
DECLFUNC void clear_spectrum_ldac(AC*, int);
DECLFUNC void dequant_spectrum_ldac(AC*);
DECLFUNC void dequant_residual_ldac(AC*);

/* unpack_ldac.c */
DECLFUNC int unpack_frame_header_ldac(int*, int*, int*, int*, STREAM*);
DECLFUNC int unpack_raw_data_frame_ldac(SFINFO*, STREAM*, int*, int*);
DECLFUNC int pack_null_data_frame_ldac(SFINFO*, STREAM*, int*, int*);

/* tables_ldac.c */
DECLFUNC int get_block_nchs_ldac(int);

/* tables_sigproc_ldac.c */
DECLFUNC void set_imdct_table_ldac(int);

/* memory_ldac.c */
DECLFUNC size_t align_ldac(size_t);
DECLFUNC void* calloc_ldac(SFINFO*, size_t, size_t);

#endif /* _PROTO_LDAC_H */
