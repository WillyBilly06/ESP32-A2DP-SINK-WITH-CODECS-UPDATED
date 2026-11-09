idf_component_register(
    SRCS "../../../src/dbt_boot_policy.c" "../../../src/dbt_queue.c" "../../../src/dbt_hci.c" "../../../src/dbt_host.c" "../../../src/dbt_acl.c" "../../../src/dbt_l2cap.c" "../../../src/dbt_channels.c" "../../../src/dbt_runtime.c" "../../../src/dbt_gap.c" "../../../src/dbt_att.c" "../../../src/dbt_att_bearer.c" "../../../src/dbt_classic_security.c" "../../../src/dbt_sdp.c" "../../../src/dbt_sdp_bearer.c" "../../../src/dbt_rx_flow.c"
         "../../../src/dbt_avdtp.c" "../../../src/dbt_avdtp_bearer.c"
         "../../../src/dbt_avdtp_caps.c" "../../../src/dbt_avdtp_query.c"
         "../../../src/dbt_avdtp_sink.c"
         "../../../src/dbt_a2dp_sbc.c"
         "../../../src/dbt_a2dp_acceptor.c" "../../../src/dbt_a2dp_connect.c" "../../../src/dbt_a2dp_select.c" "../../../src/dbt_a2dp_generate.c" "../../../src/dbt_a2dp_rank.c" "../../../src/dbt_sbc_media.c"
         "../../../src/dbt_credit.c"
         "../../../src/dbt_page.c"
         "../../../src/dbt_sdp_client.c"
         "../../../src/dbt_sdp_client_bearer.c"
         "../../../src/dbt_sdp_discovery.c"
         "../../../src/dbt_ertm.c"
         "../../../src/dbt_ertm_engine.c"
         "../../../src/dbt_avctp.c"
         "../../../src/dbt_avctp_bearer.c"
         "../../../src/dbt_avrcp.c"
         "../../../src/dbt_avrcp_bearer.c"
         "../../../src/dbt_avrcp_browse.c"
         "../../../src/dbt_smp.c"
         "../../../src/dbt_smp_crypto.c"
         "../../../src/dbt_smp_bond.c"
         "../../../src/dbt_smp_dist.c"
         "../../../src/dbt_smp_guard.c"
         "../../../src/dbt_smp_bearer.c"
         "../../../src/dbt_smp_sc.c"
         "../../../src/dbt_smp_features.c"
         "../../../src/dbt_smp_session.c"
         "../../../src/dbt_product_peer.c"
         "../../../src/dbt_ota.c"
         "../../../src/dbt_sound.c"
         "../../../src/dbt_sound_mix.c"
         "../../../src/dbt_led.c"
         "../../../src/dbt_input.c"
         "../../../src/dbt_rfcomm.c"
         "../../../src/dbt_rfcomm_mux.c"
         "../../../src/dbt_rfcomm_credit.c"
         "../../../src/dbt_rfcomm_dlc.c"
         "../../../src/dbt_rfcomm_bearer.c"
         "../../../src/dbt_rfcomm_spp.c"
         "../../../src/dbt_ble_control.c"
         "../../../src/dbt_audio.c" "../../../src/dbt_storage_reserve.c" "../../../src/dbt_dsp_job.c" "../../../src/dbt_pcm_snapshot.c"
         "../../../src/dbt_resampler.c" "../../../../bt_audio_sink/main/dsp/fixed_dsp.c"
    REQUIRES partition_table
    INCLUDE_DIRS "../../../include" "../../../../bt_audio_sink/main/dsp"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -Wall -Wextra -Werror)
if(DBT_PROBE_BLOCK_DSP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_AUDIO_BLOCK_DSP=1 FIXED_DSP_BLOCK_API=1)
endif()
if(DBT_PROBE_WORD_DSP)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_WORD_QUANT=1)
endif()
if(DBT_PROBE_AUDIO_STAGE_PROFILE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_AUDIO_STAGE_PROFILE=1)
    # The public ESP CPU header uses GNU asm in its inline Xtensa helpers.
    # Keep the portable core strict C11; enable GNU C11 only for this diagnostic.
    get_filename_component(profile_audio_source "${CMAKE_CURRENT_LIST_DIR}/../../../src/dbt_audio.c" ABSOLUTE)
    set_source_files_properties("${profile_audio_source}" PROPERTIES COMPILE_OPTIONS "-std=gnu11")
    idf_component_optional_requires(PRIVATE esp_hw_support)
endif()

# Profiled 192 kHz rendering exceeds its frame budget in both FIR and DSP.
# Conversion/FIR improved at O3. Controlled identical-input target runs show
# Os is faster for the DSP kernel, so give each measured path its own policy.
foreach(audio_hot_source "../../../src/dbt_audio.c" "../../../src/dbt_resampler.c")
    get_filename_component(audio_hot_absolute "${CMAKE_CURRENT_LIST_DIR}/${audio_hot_source}" ABSOLUTE)
    set_property(SOURCE "${audio_hot_absolute}" APPEND PROPERTY COMPILE_OPTIONS "-O3")
endforeach()
get_filename_component(fixed_dsp_source "${CMAKE_CURRENT_LIST_DIR}/../../../../bt_audio_sink/main/dsp/fixed_dsp.c" ABSOLUTE)
set_property(SOURCE "${fixed_dsp_source}" APPEND PROPERTY COMPILE_OPTIONS "-Os")

if(DBT_PROBE_APTX_SNAPSHOT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_APTX_SNAPSHOT=1)
endif()

if(DBT_PROBE_UNITY_PCM_COPY)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DBT_AUDIO_UNITY_COPY=1)
endif()

if(DBT_PROBE_DSP_BLOCK_IRAM)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_BLOCK_IRAM=1)
endif()

if(DBT_PROBE_VOLUME_RAMP_BLOCK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_VOLUME_RAMP_BLOCK=1)
endif()
if(DBT_PROBE_VOLUME_RAMP_COMPARE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_VOLUME_RAMP_COMPARE=1)
    set_property(SOURCE "${fixed_dsp_source}" APPEND PROPERTY COMPILE_OPTIONS "-std=gnu11")
    idf_component_optional_requires(PRIVATE esp_hw_support freertos)
endif()

if(DBT_PROBE_MATCHED_FEEDBACK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_MATCHED_FEEDBACK=1)
endif()
if(DBT_PROBE_MATCHED_FEEDBACK_COMPARE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FIXED_DSP_MATCHED_FEEDBACK_COMPARE=1)
    set_property(SOURCE "${fixed_dsp_source}" APPEND PROPERTY COMPILE_OPTIONS "-std=gnu11")
    idf_component_optional_requires(PRIVATE esp_hw_support freertos)
endif()
