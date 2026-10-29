"""Generate bounded, pinned SDK LDAC units for configuration-sized storage."""
import argparse
import hashlib
import json
from pathlib import Path

PINNED={
 "ldac.h":"d68fa82a3fb936be9cc02fd65cebe258d59472ea9404b7130a22de45997bacaa",
 "ldacBT_internal.h":"65f2a85b248f024a86df2e576b195973fd597275f7057d3d8a17770ea568e949",
 "ldacBT_internal.o.c":"b88050482ad67e2a62b9f57ef0cfbd81d264d6fb32d75d78f29c7e57aaeffffd",
 "decode_ldac.o.c":"4e55588f1b60a03efcc193c3b4474b91ffab3cb853d6290c870e37a005da9fa7",
 "unpack_ldac.o.c":"f70db0c37c5f21a5ebcd3264381137573ae4b6f19a4780fac50b892d2111acca",
 "tables_ldac.o.c":"8f3ecdab12dd23e0350c0db7c3951b96b128e38d295c6d9901d4a5afcc1bf114",
 "tables_sigproc_ldac.o.c":"da00a4d0386386e3e0dfa90640d8a2b30ff1bcfbebcdf1d39c0c62fa8a6f8699",
}


def once(s,a,b):
    if s.count(a)!=1:raise RuntimeError("LDAC port site changed: "+a[:80])
    return s.replace(a,b)


def generate(source,output):
    output.mkdir(parents=True,exist_ok=True);receipt={}
    for name,digest in PINNED.items():
        raw=(source/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=digest:raise RuntimeError("Unreviewed SDK LDAC source: "+name)
        text=raw.decode('utf-8')
        if name=='ldac.h':
            text=once(text,'SCALAR a_time[LDAC_MAXLSU * LDAC_NFRAME];','SCALAR *a_time;')
            text=once(text,'SCALAR a_spec[LDAC_MAXLSU];','SCALAR *a_spec;')
            text=once(text,'int16_t a_qspec[LDAC_MAXLSU];','int16_t *a_qspec;')
            text=once(text,'int16_t a_rspec[LDAC_MAXLSU];','int16_t *a_rspec;')
        elif name=='ldacBT_internal.h':
            text=once(text,'  char a_pcm[LDAC_PRCNCH * LDACBT_MAX_LSU * LDACBT_PCM_WLEN_MAX];','  /* PCM is bound to the negotiated-size worker allocation. */')
        elif name=='ldacBT_internal.o.c':
            text=once(text,'''  hLdacBT->ap_pcm[0] = &hLdacBT->a_pcm[0];
  hLdacBT->ap_pcm[1] = &hLdacBT->a_pcm[LDACBT_MAX_LSU * LDACBT_PCM_WLEN_MAX];
  hLdacBT->pp_pcm = hLdacBT->ap_pcm;
  clear_data_ldac(hLdacBT->a_pcm, LDAC_PRCNCH * LDACBT_MAX_LSU * LDACBT_PCM_WLEN_MAX);''',
            '''  hLdacBT->ap_pcm[0] = NULL;
  hLdacBT->ap_pcm[1] = NULL;
  hLdacBT->pp_pcm = hLdacBT->ap_pcm;''')
        elif name=='decode_ldac.o.c':
            text=once(text,'  int ch_offset = 0;','  int ch_offset = 0;\n  int samples = ga_framesmpls_ldac[p_cfg->smplrate_id];')
            text=once(text,'p_sfinfo->ap_ac[ich] = (AC*)calloc_ldac(p_sfinfo, 1u, sizeof(AC));',
                      'p_sfinfo->ap_ac[ich] = (AC*)calloc_ldac(p_sfinfo, 1u, sizeof(AC) + 2u * samples * sizeof(int16_t));')
            text=once(text,'p_sfinfo->ap_ac[ich] = (AC*)calloc_ldac(p_sfinfo, 1u, sizeof(AC) + 2u * samples * sizeof(int16_t));\n    if (p_sfinfo->ap_ac[ich] != (AC*)NULL) {',
                      '''p_sfinfo->ap_ac[ich] = (AC*)calloc_ldac(p_sfinfo, 1u, sizeof(AC) + 2u * samples * sizeof(int16_t));
    if (p_sfinfo->ap_ac[ich] != (AC*)NULL) {
      p_sfinfo->ap_ac[ich]->a_qspec = (int16_t *)(p_sfinfo->ap_ac[ich] + 1);
      p_sfinfo->ap_ac[ich]->a_rspec = p_sfinfo->ap_ac[ich]->a_qspec + samples;''')
            text=once(text,'p_sfinfo->ap_ac[ich]->p_acsub = (ACSUB*)calloc_ldac(p_sfinfo, 1u, sizeof(ACSUB));',
                      'p_sfinfo->ap_ac[ich]->p_acsub = (ACSUB*)calloc_ldac(p_sfinfo, 1u, sizeof(ACSUB) + 3u * samples * sizeof(SCALAR));')
            text=once(text,'      if (p_sfinfo->ap_ac[ich]->p_acsub == (ACSUB*)NULL) {',
                      '''      if (p_sfinfo->ap_ac[ich]->p_acsub != (ACSUB*)NULL) {
        p_sfinfo->ap_ac[ich]->p_acsub->a_time = (SCALAR *)(p_sfinfo->ap_ac[ich]->p_acsub + 1);
        p_sfinfo->ap_ac[ich]->p_acsub->a_spec = p_sfinfo->ap_ac[ich]->p_acsub->a_time + 2 * samples;
      }
      if (p_sfinfo->ap_ac[ich]->p_acsub == (ACSUB*)NULL) {''')
            text=once(text,'clear_spectrum_ldac(p_ac, LDAC_MAXLSU);','clear_spectrum_ldac(p_ac, ga_framesmpls_ldac[p_sfinfo->cfg.smplrate_id]);')
        elif name=='unpack_ldac.o.c':
            start=text.index('static void read_unpack_ldac(');end=text.index('\n/* Helper function',start)
            text=text[:start]+'''static void read_unpack_ldac(int nbits, STREAM* p_block, int* p_loc, int* p_idata) {
  if (!nbits) { *p_idata = 0; return; }
  /* The two zero lookahead bytes permit a speculative Huffman read beginning
   * inside the frame; the original unpacker later rewinds to the used bits. */
  if (nbits < 0 || nbits > 24 || *p_loc < 0 || (uint32_t)*p_loc >= dbt_ldac_port_bit_limit) {
    dbt_ldac_port_overread = 1; *p_idata = 0; return;
  }
  unsigned bpos = (unsigned)*p_loc & LDAC_LOC_MASK;
  STREAM *p_bufptr = p_block + ((unsigned)*p_loc >> LDAC_LOC_SHIFT);
  unsigned tmp = ((unsigned)p_bufptr[2]) | ((unsigned)p_bufptr[1] << 8) | ((unsigned)p_bufptr[0] << 16);
  *p_idata = (int)(((tmp << bpos) & 0xFFFFFFu) >> (24 - nbits));
  *p_loc += nbits;
}
'''+text[end:]
            text=once(text,'memset(p_ac->a_qspec, 0, sizeof(p_ac->a_qspec));',
                      'memset(p_ac->a_qspec, 0, sizeof(int16_t) * ga_framesmpls_ldac[*p_ab->p_smplrate_id]);')
            text=once(text,'memset(p_ac->a_rspec, 0, sizeof(p_ac->a_rspec));',
                      'memset(p_ac->a_rspec, 0, sizeof(int16_t) * ga_framesmpls_ldac[*p_ab->p_smplrate_id]);')
            if text.count('  HCDEC* p_hcsf;')!=2:raise RuntimeError('LDAC Huffman pointer declarations changed')
            text=text.replace('  HCDEC* p_hcsf;','  const HCDEC* p_hcsf;')
        elif name=='tables_ldac.o.c':
            # Xtensa plain char is unsigned; the -1 shift sentinels must stay
            # negative or the SDK's configuration guards silently accept them.
            text=once(text,'DECLFUNC const char gaa_nlnn_shift_ldac[','DECLFUNC const signed char gaa_nlnn_shift_ldac[')
            text=once(text,'DECLFUNC HCDEC ga_hcdec_sf0_ldac[','DECLFUNC const HCDEC ga_hcdec_sf0_ldac[')
            text=once(text,'DECLFUNC HCDEC ga_hcdec_sf1_ldac[','DECLFUNC const HCDEC ga_hcdec_sf1_ldac[')
            text+='''\n#ifdef DBT_LDAC_PORT_SELFTEST
int dbt_ldac_shift_table_test(void) {
  return gaa_nlnn_shift_ldac[0][0] == -1 && gaa_nlnn_shift_ldac[0][2] == 0 &&
         gaa_nlnn_shift_ldac[2][1] == 0 && gaa_nlnn_shift_ldac[2][3] == -1;
}
#endif
'''
        elif name=='tables_sigproc_ldac.o.c':
            for declaration in ('DECLFUNC const SCALAR* gaa_wsin_ldac[LDAC_NUMLNN];',
                                'DECLFUNC const int* gaa_rev_perm_ldac[LDAC_NUMLNN];',
                                'DECLFUNC const SCALAR* gaa_bwin_ldac[LDAC_NUMLNN];',
                                'DECLFUNC const SCALAR* gaa_wcos_ldac[LDAC_NUMLNN];'):
                text=once(text,declaration,'')
            anchor='DECLFUNC const SCALAR ga_rsf_ldac[LDAC_NIDWL] = {'
            definitions='''/* Immutable references to the two flash-resident transform banks. */
DECLFUNC const SCALAR* const gaa_wsin_ldac[LDAC_NUMLNN] = {sa_wsin_1fs_ldac, sa_wsin_2fs_ldac};
DECLFUNC const int* const gaa_rev_perm_ldac[LDAC_NUMLNN] = {sa_rev_perm_1fs_ldac, sa_rev_perm_2fs_ldac};
DECLFUNC const SCALAR* const gaa_bwin_ldac[LDAC_NUMLNN] = {sa_bwin_1fs_ldac, sa_bwin_2fs_ldac};
DECLFUNC const SCALAR* const gaa_wcos_ldac[LDAC_NUMLNN] = {sa_wcos_1fs_ldac, sa_wcos_2fs_ldac};
'''
            text=once(text,anchor,definitions+anchor)
            start=text.index('DECLFUNC void set_imdct_table_ldac(int nlnn) {')
            text=text[:start]+'''DECLFUNC void set_imdct_table_ldac(int nlnn) {
#if CONFIG_USED_CSI_DSP
  uint16_t bank_size = npow2_ldac(nlnn);
  csi_dct4_init_f32(&dct4f32,&rfftf32,&cfftradix4f32,bank_size,bank_size/2,1.0);
#endif
  (void)nlnn;
}
'''
        data=text.encode('utf-8')
        if len(data)>131072:raise RuntimeError("Unexpected LDAC port unit size")
        (output/name).write_bytes(data);receipt[name]={"source_sha256":digest,"generated_sha256":hashlib.sha256(data).hexdigest()}
    (output/'ldac_port.json').write_text(json.dumps(receipt,indent=2)+'\n')
    return receipt


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();print(json.dumps(generate(args.source,args.output),indent=2))
