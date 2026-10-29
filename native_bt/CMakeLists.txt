cmake_minimum_required(VERSION 3.16)
project(native_bt C)
add_library(native_bt STATIC src/dbt_queue.c src/dbt_hci.c src/dbt_host.c src/dbt_acl.c src/dbt_l2cap.c src/dbt_channels.c src/dbt_runtime.c src/dbt_gap.c src/dbt_att.c src/dbt_att_bearer.c src/dbt_classic_security.c src/dbt_sdp.c src/dbt_sdp_bearer.c src/dbt_rx_flow.c src/dbt_avdtp.c src/dbt_avdtp_bearer.c)
target_sources(native_bt PRIVATE src/dbt_avdtp_caps.c src/dbt_avdtp_sink.c src/dbt_a2dp_sbc.c)
target_sources(native_bt PRIVATE src/dbt_a2dp_acceptor.c src/dbt_sbc_media.c)
target_sources(native_bt PRIVATE src/dbt_credit.c)

target_sources(native_bt PRIVATE
    src/dbt_ertm.c
    src/dbt_ertm_engine.c
    src/dbt_avctp.c
    src/dbt_avctp_bearer.c
    src/dbt_avrcp.c
    src/dbt_avrcp_bearer.c
    src/dbt_avrcp_browse.c
    src/dbt_smp.c
    src/dbt_smp_crypto.c
    src/dbt_smp_bond.c
    src/dbt_smp_dist.c
    src/dbt_smp_guard.c
    src/dbt_smp_bearer.c
    src/dbt_smp_sc.c
    src/dbt_smp_features.c
    src/dbt_smp_session.c
)
target_sources(native_bt PRIVATE src/dbt_ble_control.c
    src/dbt_audio.c
    src/dbt_resampler.c ../bt_audio_sink/main/dsp/fixed_dsp.c)
target_include_directories(native_bt PUBLIC ../bt_audio_sink/main/dsp)
target_include_directories(native_bt PUBLIC include)
target_compile_features(native_bt PUBLIC c_std_11)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(native_bt PRIVATE -Wall -Wextra -Werror)
endif()
