idf_component_register(
    SRCS "../../../src/dbt_queue.c" "../../../src/dbt_hci.c" "../../../src/dbt_host.c" "../../../src/dbt_acl.c" "../../../src/dbt_l2cap.c" "../../../src/dbt_channels.c" "../../../src/dbt_runtime.c" "../../../src/dbt_gap.c" "../../../src/dbt_att.c" "../../../src/dbt_att_bearer.c" "../../../src/dbt_classic_security.c" "../../../src/dbt_sdp.c" "../../../src/dbt_sdp_bearer.c" "../../../src/dbt_rx_flow.c"
         "../../../src/dbt_avdtp.c" "../../../src/dbt_avdtp_bearer.c"
         "../../../src/dbt_avdtp_caps.c"
         "../../../src/dbt_avdtp_sink.c"
         "../../../src/dbt_a2dp_sbc.c"
    INCLUDE_DIRS "../../../include"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c11 -Wall -Wextra -Werror)
