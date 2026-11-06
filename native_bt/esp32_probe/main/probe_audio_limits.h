#ifndef PROBE_AUDIO_LIMITS_H
#define PROBE_AUDIO_LIMITS_H
/* Experimental owned-host receive capacity, including RTP/codec headers.
 * Advertise a full 1800-byte media SDU. The selected codec policy allocates
 * four or eight input slots; this is their upper bound, not PCM scratch size.
 * HCI controller ACL buffers remain unchanged; L2CAP reassembles fragments. */
#define PROBE_A2DP_RX_MTU 1800u
#define PROBE_A2DP_MEDIA_SLOTS 8u
#endif
