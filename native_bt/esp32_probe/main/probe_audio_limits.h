#ifndef PROBE_AUDIO_LIMITS_H
#define PROBE_AUDIO_LIMITS_H
/* Experimental owned-host receive capacity, including RTP/codec headers.
 * Reserve eight 1800-byte decoder input slots and advertise the same L2CAP
 * media limit, so a larger SDU is not rejected before reaching the worker.
 * HCI controller ACL buffers remain unchanged; L2CAP reassembles fragments. */
#define PROBE_A2DP_RX_MTU 1800u
#define PROBE_A2DP_MEDIA_SLOTS 8u
#endif
