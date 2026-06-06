/*
 * SPDX-FileCopyrightText: 2026 (local extension)
 * SPDX-License-Identifier: Apache-2.0
 *
 * esp_wifi_ecast.c — EspCastBR low-latency stack tuning + raw TX helper.
 *
 * This file sits inside components/esp_wifi (the Wi-Fi stack itself),
 * not in any application. It uses APIs that are NOT exposed at the
 * public esp_now level:
 *
 *   - esp_wifi_internal_set_fix_rate() — pins PHY rate on a whole
 *     interface so every ESP-NOW broadcast is transmitted at a known
 *     rate. Without this, the driver's internal rate table decides,
 *     and the broadcast copies of a single ECast frame can end up on
 *     different rates, breaking the airtime budget calculations.
 *
 *   - esp_wifi_80211_tx() — raw 802.11 injection bypassing ESP-NOW
 *     queue management. Used by ecast_tx_raw() for tight RTN scheduling.
 *
 *   - esp_wifi_internal_set_log_level() — mute driver noise during
 *     the low-latency session so the log output doesn't preempt
 *     the Wi-Fi task.
 */

#include "esp_wifi_ecast.h"
#include "esp_private/wifi.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "wifi_ecast";

/* One bit per interface (STA=0, AP=1, …). Cleared on esp_wifi_deinit. */
static uint8_t s_active_mask = 0;

esp_err_t esp_wifi_ecast_apply_low_latency(wifi_interface_t ifx,
                                           wifi_phy_rate_t  fixed_rate)
{
#if CONFIG_ESP_WIFI_ECAST_LOW_LATENCY
    esp_err_t err;
    esp_err_t first_err = ESP_OK;

#define TRY(call) do { err = (call); if (err != ESP_OK && first_err == ESP_OK) { \
    ESP_LOGW(TAG, "%s -> %s", #call, esp_err_to_name(err)); first_err = err; } } while (0)

    /* 1. Power-save off: WIFI_PS_NONE keeps the radio fully awake so we
     *    never sleep between RTN retransmits. */
    TRY(esp_wifi_set_ps(WIFI_PS_NONE));

    /* 2. Max TX power: 21 dBm on most chips (value in 0.25 dBm units). */
    TRY(esp_wifi_set_max_tx_power(84));

    /* 3. Pin PHY rate. This is *the* knob that turns every broadcast into
     *    a predictable-airtime frame. */
    if (fixed_rate != 0) {
        TRY(esp_wifi_internal_set_fix_rate(ifx, true, fixed_rate));
    }

    /* 4. Log-level: mute the wifi driver during streaming; keeps the
     *    Wi-Fi task from being preempted by printf. App can override. */
    esp_log_level_set("wifi", ESP_LOG_WARN);

    /* 5. Lower protocol set to basic-rates only (no HT/VHT). This stops
     *    the driver from attempting rate up-shifts during ACKs and keeps
     *    the broadcast air-frame format stable. */
    TRY(esp_wifi_set_protocol(ifx, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G));

    if (first_err == ESP_OK) {
        s_active_mask |= (uint8_t)(1u << (unsigned)ifx);
        ESP_LOGI(TAG, "ECast low-latency profile applied on ifx=%d (rate=%d)",
                 (int)ifx, (int)fixed_rate);
    }
    return first_err;
#else
    (void)ifx; (void)fixed_rate;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t esp_wifi_ecast_tx_raw(wifi_interface_t ifx, const void *frame, int len)
{
#if CONFIG_ESP_WIFI_ECAST_LOW_LATENCY
    if (!frame || len <= 0) return ESP_ERR_INVALID_ARG;
    return esp_wifi_80211_tx(ifx, frame, len, true);
#else
    (void)ifx; (void)frame; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

bool esp_wifi_ecast_is_active(wifi_interface_t ifx)
{
#if CONFIG_ESP_WIFI_ECAST_LOW_LATENCY
    return (s_active_mask & (uint8_t)(1u << (unsigned)ifx)) != 0;
#else
    (void)ifx;
    return false;
#endif
}
