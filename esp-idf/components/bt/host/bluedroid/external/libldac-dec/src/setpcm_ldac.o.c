#include "ldac.h"

/* Fast float to int16 conversion without floor() - uses rounding */
static inline int fast_round_to_int(SCALAR x) {
    return (int)(x + (x >= 0 ? _scalar(0.5) : _scalar(-0.5)));
}

/* Fast clamp to 16-bit range */
static inline short fast_clamp16(int x) {
    if (__builtin_expect(x < -0x8000, 0)) return -0x8000;
    if (__builtin_expect(x > 0x7FFF, 0)) return 0x7FFF;
    return (short)x;
}

/* Fast clamp to 24-bit range */
static inline int fast_clamp24(int x) {
    if (__builtin_expect(x < -0x800000, 0)) return -0x800000;
    if (__builtin_expect(x > 0x7FFFFF, 0)) return 0x7FFFFF;
    return x;
}

DECLFUNC void set_output_pcm_ldac(
    SFINFO* p_sfinfo, void* pp_pcm[], LDAC_SMPL_FMT_T format, int nlnn) {
  const int nchs = p_sfinfo->cfg.ch;
  const int nsmpl = npow2_ldac(nlnn);
  
  if (__builtin_expect(nchs <= 0, 0)) {
    return;
  }
  
  switch (format) {
    case LDAC_SMPL_FMT_S16: {
      for (int ich = 0; ich < nchs; ich++) {
        const SCALAR* __restrict__ p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        short* __restrict__ p_pcm = pp_pcm[ich];
        
        /* Unroll by 4 for better pipelining */
        const int nsmpl4 = nsmpl & ~3;
        int isp;
        for (isp = 0; isp < nsmpl4; isp += 4) {
          int t0 = fast_round_to_int(p_time[isp + 0]);
          int t1 = fast_round_to_int(p_time[isp + 1]);
          int t2 = fast_round_to_int(p_time[isp + 2]);
          int t3 = fast_round_to_int(p_time[isp + 3]);
          p_pcm[isp + 0] = fast_clamp16(t0);
          p_pcm[isp + 1] = fast_clamp16(t1);
          p_pcm[isp + 2] = fast_clamp16(t2);
          p_pcm[isp + 3] = fast_clamp16(t3);
        }
        /* Handle remaining samples */
        for (; isp < nsmpl; isp++) {
          p_pcm[isp] = fast_clamp16(fast_round_to_int(p_time[isp]));
        }
      }
      break;
    }
    case LDAC_SMPL_FMT_S24: {
      for (int ich = 0; ich < nchs; ich++) {
        const SCALAR* __restrict__ p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        char* __restrict__ p_pcm = pp_pcm[ich];
        
        for (int isp = 0; isp < nsmpl; isp++) {
          int temp = fast_round_to_int(p_time[isp] * _scalar(256.0));
          temp = fast_clamp24(temp);
          p_pcm[(isp * 3) + 0] = ((char*)(&temp))[0];
          p_pcm[(isp * 3) + 1] = ((char*)(&temp))[1];
          p_pcm[(isp * 3) + 2] = ((char*)(&temp))[2];
        }
      }
      break;
    }
    case LDAC_SMPL_FMT_S32: {
      for (int ich = 0; ich < nchs; ich++) {
        const SCALAR* __restrict__ p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        int* __restrict__ p_pcm = pp_pcm[ich];
        
        const int nsmpl4 = nsmpl & ~3;
        int isp;
        for (isp = 0; isp < nsmpl4; isp += 4) {
          long long t0 = (long long)(p_time[isp + 0] * _scalar(65536.0) + _scalar(0.5));
          long long t1 = (long long)(p_time[isp + 1] * _scalar(65536.0) + _scalar(0.5));
          long long t2 = (long long)(p_time[isp + 2] * _scalar(65536.0) + _scalar(0.5));
          long long t3 = (long long)(p_time[isp + 3] * _scalar(65536.0) + _scalar(0.5));
          if (t0 < -0x80000000LL) t0 = -0x80000000LL;
          if (t0 >= 0x7FFFFFFFLL) t0 = 0x7FFFFFFFLL;
          if (t1 < -0x80000000LL) t1 = -0x80000000LL;
          if (t1 >= 0x7FFFFFFFLL) t1 = 0x7FFFFFFFLL;
          if (t2 < -0x80000000LL) t2 = -0x80000000LL;
          if (t2 >= 0x7FFFFFFFLL) t2 = 0x7FFFFFFFLL;
          if (t3 < -0x80000000LL) t3 = -0x80000000LL;
          if (t3 >= 0x7FFFFFFFLL) t3 = 0x7FFFFFFFLL;
          p_pcm[isp + 0] = (int)t0;
          p_pcm[isp + 1] = (int)t1;
          p_pcm[isp + 2] = (int)t2;
          p_pcm[isp + 3] = (int)t3;
        }
        for (; isp < nsmpl; isp++) {
          long long temp = (long long)(p_time[isp] * _scalar(65536.0) + _scalar(0.5));
          if (temp < -0x80000000LL) temp = -0x80000000LL;
          if (temp >= 0x7FFFFFFFLL) temp = 0x7FFFFFFFLL;
          p_pcm[isp] = (int)temp;
        }
      }
      break;
    }
    case LDAC_SMPL_FMT_F32: {
      const SCALAR inv_scale = _scalar(1.0) / _scalar(32768.0);
      for (int ich = 0; ich < nchs; ich++) {
        const SCALAR* __restrict__ p_time = p_sfinfo->ap_ac[ich]->p_acsub->a_time;
        float* __restrict__ p_pcm = pp_pcm[ich];
        
        const int nsmpl4 = nsmpl & ~3;
        int isp;
        for (isp = 0; isp < nsmpl4; isp += 4) {
          float t0 = p_time[isp + 0] * inv_scale;
          float t1 = p_time[isp + 1] * inv_scale;
          float t2 = p_time[isp + 2] * inv_scale;
          float t3 = p_time[isp + 3] * inv_scale;
          /* Clamp to [-1, 1] */
          t0 = (t0 < -1.0f) ? -1.0f : ((t0 > 1.0f) ? 1.0f : t0);
          t1 = (t1 < -1.0f) ? -1.0f : ((t1 > 1.0f) ? 1.0f : t1);
          t2 = (t2 < -1.0f) ? -1.0f : ((t2 > 1.0f) ? 1.0f : t2);
          t3 = (t3 < -1.0f) ? -1.0f : ((t3 > 1.0f) ? 1.0f : t3);
          p_pcm[isp + 0] = t0;
          p_pcm[isp + 1] = t1;
          p_pcm[isp + 2] = t2;
          p_pcm[isp + 3] = t3;
        }
        for (; isp < nsmpl; isp++) {
          float temp = p_time[isp] * inv_scale;
          temp = (temp < -1.0f) ? -1.0f : ((temp > 1.0f) ? 1.0f : temp);
          p_pcm[isp] = temp;
        }
      }
      break;
    }
    default:
      break;
  }
}
