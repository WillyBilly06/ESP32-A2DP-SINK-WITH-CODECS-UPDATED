/*
 * SPDX-FileCopyrightText: 2026 (local extension)
 * SPDX-License-Identifier: Apache-2.0
 *
 * esp_wifi_ecast.h — Wi-Fi stack extensions for the EspCastBR
 * (Auracast-style broadcast audio) protocol.
 *
 * These APIs live in the esp_wifi component (not in application code)
 * because they manipulate stack-internal state:
 *   - AMPDU TX/RX enable
 *   - Power save mode
 *   - Fixed PHY rate (via esp_wifi_internal_set_fix_rate)
 *   - Max TX power
 *   - Raw 80211 TX fast-path that bypasses ESP-NOW queueing
 *
 * They are provided as a single point of truth so an application using
 * EspCastBR just calls esp_wifi_ecast_apply_low_latency() once, instead
 * of duplicating the stack configuration sequence in every project.
 *
 * The code compiles out to nothing if ESP_WIFI_ECAST_LOW_LATENCY is not
 * selected in Kconfig.
 */

#pragma once

#include "esp_err.h"
#include "esp_wifi_types.h"
#include "esp_wifi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Apply the EspCastBR low-latency broadcast profile to the Wi-Fi driver.
 *
 * Performs the following on the given interface:
 *   - Disables Wi-Fi power save (WIFI_PS_NONE).
 *   - Disables AMPDU TX and RX (they add tens of ms of batching latency
 *     which is unacceptable for 7.5 ms audio frames).
 *   - Sets a fixed PHY rate (default WIFI_PHY_RATE_24M) via the internal
 *     rate-fix API. 24 Mbps is the sweet spot between link margin and
 *     airtime per broadcast copy.
 *   - Sets max TX power.
 *
 * Must be called AFTER esp_wifi_start() but BEFORE audio streaming
 * begins. Safe to call multiple times; subsequent calls re-assert.
 *
 * @param ifx        interface to configure (usually WIFI_IF_STA on sinks,
 *                   WIFI_IF_AP on sources).
 * @param fixed_rate PHY rate to pin. Pass 0 (i.e. WIFI_PHY_RATE_1M_L) to
 *                   let the driver pick; WIFI_PHY_RATE_24M is recommended.
 * @return ESP_OK on success, the first non-OK error otherwise.
 */
esp_err_t esp_wifi_ecast_apply_low_latency(wifi_interface_t ifx,
                                           wifi_phy_rate_t  fixed_rate);

/**
 * @brief Send a raw 802.11 action frame via the low-level injection path.
 *
 * Bypasses ESP-NOW's internal queue/backoff by calling esp_wifi_80211_tx()
 * directly. Use this to emit BIS-style retransmit copies with tight,
 * predictable timing. Caller is responsible for framing (802.11 header
 * must be included in `frame`).
 *
 * @param ifx    interface.
 * @param frame  fully-formed 802.11 frame (incl. hdr).
 * @param len    frame length.
 * @return ESP_OK on success.
 */
esp_err_t esp_wifi_ecast_tx_raw(wifi_interface_t ifx, const void *frame, int len);

/**
 * @brief Query whether the low-latency profile is active on an interface.
 *
 * Useful for runtime asserts or log banners.
 */
bool esp_wifi_ecast_is_active(wifi_interface_t ifx);

#ifdef __cplusplus
}
#endif
