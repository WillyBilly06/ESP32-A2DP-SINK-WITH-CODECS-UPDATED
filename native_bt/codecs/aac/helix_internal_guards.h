#ifndef DBT_HELIX_INTERNAL_GUARDS_H
#define DBT_HELIX_INTERNAL_GUARDS_H
#include "aaccommon.h"
#include "coder.h"
/* Internal to the pinned AAC-LC port. Raw input is bounded by the owned adapter.
 * Negative cachedBits records a read beyond the available bitstream. */
static inline int dbt_helix_bits_left(const BitStreamInfo *bsi) {
    return bsi->nBytes*8+bsi->cachedBits;
}
/* Keep underflow sticky without invoking SDK shifts with negative cache counts. */
static inline unsigned dbt_helix_read_bits(BitStreamInfo *bsi,int n) {
    if(n<0 || n>31 || dbt_helix_bits_left(bsi)<n) {
        bsi->cachedBits=-(bsi->nBytes*8+1);return 0;
    }
    return GetBits(bsi,n);
}
static inline void dbt_helix_advance_bits(BitStreamInfo *bsi,int n) {
    if(n<0 || n>31 || dbt_helix_bits_left(bsi)<n) {
        bsi->cachedBits=-(bsi->nBytes*8+1);return;
    }
    AdvanceBitstream(bsi,n);
}
static inline void dbt_helix_byte_align(BitStreamInfo *bsi) {
    dbt_helix_advance_bits(bsi,bsi->cachedBits&7);
}
static inline int dbt_helix_ics_valid(const ICSInfo *ics,int rate) {
    if(rate<0 || rate>=NUM_SAMPLE_RATES || !ics->numWinGroup ||
       ics->numWinGroup>MAX_WIN_GROUPS)return 0;
    unsigned bands=ics->winSequence==2?sfBandTotalShort[rate]:sfBandTotalLong[rate];
    if(ics->maxSFB>bands || ics->numWinGroup*ics->maxSFB>MAX_SF_BANDS)return 0;
    unsigned windows=0;
    for(unsigned i=0;i<ics->numWinGroup;++i) {
        if(!ics->winGroupLen[i] || ics->winGroupLen[i]>NWINDOWS_SHORT)return 0;
        windows+=ics->winGroupLen[i];
    }
    return ics->winSequence==2?windows==NWINDOWS_SHORT:
        ics->numWinGroup==1 && windows==1;
}
static inline int dbt_helix_pulse_valid(const ICSInfo *ics,const PulseInfo *pulse,int rate) {
    if(ics->winSequence==2 || rate<0 || rate>=NUM_SAMPLE_RATES ||
       !pulse->numPulse || pulse->numPulse>MAX_PULSES ||
       pulse->startSFB>=sfBandTotalLong[rate])return 0;
    unsigned offset=(unsigned)sfBandTabLong[sfBandTabLongOffset[rate]+pulse->startSFB];
    for(unsigned i=0;i<pulse->numPulse;++i) {
        offset+=pulse->offset[i];
        if(offset>=NSAMPS_LONG)return 0;
    }
    return 1;
}
#endif
