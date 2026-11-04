# Project multi-codec A2DP extensions. Paths are relative to components/bt.
if(CONFIG_BT_BLUEDROID_ENABLED AND CONFIG_BT_A2DP_ENABLE)
    list(APPEND srcs
        "host/bluedroid/stack/a2dp/a2d_sbc_decoder.c"
        "host/bluedroid/stack/a2dp/a2dp_codec_config.c"
        "host/bluedroid/stack/a2dp/a2dp_vendor.c")
endif()

if(CONFIG_BT_A2DP_APTX_DECODER)
    list(APPEND priv_include_dirs host/bluedroid/external/libfreeaptx/main)
    list(APPEND aptx_dec_srcs "host/bluedroid/external/libfreeaptx/main/freeaptx.c")
    list(APPEND srcs ${aptx_dec_srcs})

    list(APPEND srcs "host/bluedroid/stack/a2dp/a2dp_vendor_aptx.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_aptx_hd.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_aptx_ll.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_aptx_decoder.c")
endif()

if(CONFIG_BT_A2DP_LDAC_DECODER)
    list(APPEND priv_include_dirs host/bluedroid/external/libldac-dec/src
                                  host/bluedroid/external/libldac-dec/inc)
    list(APPEND ldacbt_dec_srcs "host/bluedroid/external/libldac-dec/src/ldacBT.c"
                                "host/bluedroid/external/libldac-dec/src/ldaclib.c")
    list(APPEND srcs ${ldacbt_dec_srcs})

    list(APPEND srcs "host/bluedroid/stack/a2dp/a2dp_vendor_ldac.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_ldacbt_decoder.c")
endif()

if(CONFIG_BT_A2DP_OPUS_DECODER)

    list(APPEND priv_include_dirs   "host/bluedroid/external/opus/"
                                    "host/bluedroid/external/opus/include"
                                    "host/bluedroid/external/opus/silk"
                                    "host/bluedroid/external/opus/silk/fixed"
                                    "host/bluedroid/external/opus/silk/float"
                                    "host/bluedroid/external/opus/celt"
                                    "host/bluedroid/external/opus/dnn")

    list(APPEND priv_include_dirs   "host/bluedroid/external/opus_config")

    file(GLOB opus_srcs "host/bluedroid/external/opus/src/*.c")
    file(GLOB opus_silk "host/bluedroid/external/opus/silk/*.c")
    file(GLOB opus_celt "host/bluedroid/external/opus/celt/*.c")
    file(GLOB opus_fixed "host/bluedroid/external/opus/slik/fixed/*.c")

    list(APPEND srcs ${opus_srcs})
    list(APPEND srcs ${opus_silk})
    list(APPEND srcs ${opus_celt})
    list(APPEND srcs ${opus_fixed})

    set_source_files_properties(host/bluedroid/external/opus/silk/quant_LTP_gains.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
    set_source_files_properties(host/bluedroid/external/opus/silk/enc_API.c PROPERTIES COMPILE_FLAGS -Wno-error=stringop-overflow)
    set_source_files_properties(host/bluedroid/external/opus/silk/enc_API.c PROPERTIES COMPILE_FLAGS -Wno-stringop-overflow)
    set_source_files_properties(host/bluedroid/external/opus/silk/NLSF2A.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
    set_source_files_properties(host/bluedroid/external/opus/celt/celt_lpc.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)

    list(APPEND srcs "host/bluedroid/stack/a2dp/a2dp_vendor_opus.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_opus_decoder.c")
endif()

if(CONFIG_BT_A2DP_LC3PLUS_DECODER)
    list(APPEND priv_include_dirs   "host/bluedroid/external/liblc3/include")

    file(GLOB lc3_srcs "host/bluedroid/external/liblc3/src/*.c")
    list(APPEND srcs ${lc3_srcs})
endif()

if(CONFIG_BT_A2DP_LC3PLUS_DECODER)
    list(APPEND srcs "host/bluedroid/stack/a2dp/a2dp_vendor_lc3plus.c"
                     "host/bluedroid/stack/a2dp/a2dp_vendor_lc3plus_decoder.c")
endif()

if(CONFIG_BT_A2DP_AAC_DECODER)

    # =========================================================================
    # Helix AAC Decoder - RealNetworks libhelix-aac
    # =========================================================================
    # Architecture:
    #   1. Lightweight LATM parser (latm_parser.c) strips transport headers
    #   2. Helix AAC decoder for raw AAC frame decoding
    #
    # Benefits:
    #   - Proven, well-tested AAC decoder from RealNetworks
    #   - Fixed-point 16-bit math, no SBR (saves memory)
    #   - AAC-LC profile only (perfect for A2DP)
    # =========================================================================

    # Helix AAC decoder include directories
    list(APPEND priv_include_dirs
        "host/bluedroid/external/libaac-lc"
        "host/bluedroid/external/libaac-lc/libhelix_aac"
    )

    # Helix AAC decoder source files (core - no SBR)
    list(APPEND srcs
        "host/bluedroid/external/libaac-lc/libhelix_aac/aacdec.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/aactabs.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/bitstream.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/buffers.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/dct4.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/decelmnt.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/dequant.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/fft.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/filefmt.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/huffman.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/hufftabs.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/imdct.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/noiseless.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/pns.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/stproc.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/tns.c"
        "host/bluedroid/external/libaac-lc/libhelix_aac/trigtabs.c"
    )

    # Wrapper that adapts Helix AAC to our A2DP interface
    list(APPEND srcs
        "host/bluedroid/external/libaac-lc/aac_decoder_helix.c"
    )

    # LATM parser + decoder wrapper
    list(APPEND srcs
        "host/bluedroid/stack/a2dp/a2dp_aac.c"
        "host/bluedroid/stack/a2dp/latm_parser.c"
        "host/bluedroid/stack/a2dp/a2dp_aac_decoder_custom.c"
    )

endif()


