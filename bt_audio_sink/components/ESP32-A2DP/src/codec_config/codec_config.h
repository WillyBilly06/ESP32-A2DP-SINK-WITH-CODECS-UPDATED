/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CODEC_CONFIG__
#define __CODEC_CONFIG__

#include <stdint.h>
#include "esp_a2dp_api.h"

#define CODEC_CONFIG_TAG       "CODEC_CONFIG"

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

bool get_codec_config(esp_a2d_cb_param_t *a2d, uint32_t* sr, uint8_t* bps,
                      uint8_t* ch);

#ifdef __cplusplus
}   /* extern "C" */
#endif /* __cplusplus */

#endif /* __CODEC_CONFIG__ */
