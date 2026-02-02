/**
 * Transport type identifiers.
 */

namespace aac_fdk {

typedef enum {
  TT_UNKNOWN = -1, /**< Unknown format.            */
  TT_MP4_RAW = 0,  /**< "as is" access units (packet based since there is
                      obviously no sync layer) */
  TT_MP4_ADIF = 1, /**< ADIF bitstream format.     */
  TT_MP4_ADTS = 2, /**< ADTS bitstream format.     */

  TT_MP4_LATM_MCP1 = 6, /**< Audio Mux Elements with muxConfigPresent = 1 */
  TT_MP4_LATM_MCP0 = 7, /**< Audio Mux Elements with muxConfigPresent = 0, out
                           of band StreamMuxConfig */

  TT_MP4_LOAS = 10, /**< Audio Sync Stream.         */

  TT_DRM = 12 /**< Digital Radio Mondial (DRM30/DRM+) bitstream format. */

} TRANSPORT_TYPE;

/**
 * \brief AAC decoder setting parameters
 */
typedef enum {
  AAC_PCM_DUAL_CHANNEL_OUTPUT_MODE =
      0x0002, /*!< Defines how the decoder processes two channel signals: \n
                   0: Leave both signals as they are (default). \n
                   1: Create a dual mono output signal from channel 1. \n
                   2: Create a dual mono output signal from channel 2. \n
                   3: Create a dual mono output signal by mixing both channels
                 (L' = R' = 0.5*Ch1 + 0.5*Ch2). */
  AAC_PCM_OUTPUT_CHANNEL_MAPPING =
      0x0003, /*!< Output buffer channel ordering. 0: MPEG PCE style order, 1:
                 WAV file channel order (default). */
  AAC_PCM_LIMITER_ENABLE =
      0x0004,                           /*!< Enable signal level limiting. \n
                                             -1: Auto-config. Enable limiter for all
                                           non-lowdelay configurations by default. \n
                                              0: Disable limiter in general. \n
                                              1: Enable limiter always.
                                             It is recommended to call the decoder
                                           with a AACDEC_CLRHIST flag to reset all
                                           states when      the limiter switch is changed
                                           explicitly. */
  AAC_PCM_LIMITER_ATTACK_TIME = 0x0005, /*!< Signal level limiting attack time
                                           in ms. Default configuration is 15
                                           ms. Adjustable range from 1 ms to 15
                                           ms. */
  AAC_PCM_LIMITER_RELEAS_TIME = 0x0006, /*!< Signal level limiting release time
                                           in ms. Default configuration is 50
                                           ms. Adjustable time must be larger
                                           than 0 ms. */
  AAC_PCM_MIN_OUTPUT_CHANNELS =
      0x0011, /*!< Minimum number of PCM output channels. If higher than the
                 number of encoded audio channels, a simple channel extension is
                 applied (see note 4 for exceptions). \n -1, 0: Disable channel
                 extension feature. The decoder output contains the same number
                 of channels as the encoded bitstream. \n 1:    This value is
                 currently needed only together with the mix-down feature. See
                          ::AAC_PCM_MAX_OUTPUT_CHANNELS and note 2 below. \n
                    2:    Encoded mono signals will be duplicated to achieve a
                 2/0/0.0 channel output configuration. \n 6:    The decoder
                 tries to reorder encoded signals with less than six channels to
                 achieve a 3/0/2.1 channel output signal. Missing channels will
                 be filled with a zero signal. If reordering is not possible the
                 empty channels will simply be appended. Only available if
                 instance is configured to support multichannel output. \n 8:
                 The decoder tries to reorder encoded signals with less than
                 eight channels to achieve a 3/0/4.1 channel output signal.
                 Missing channels will be filled with a zero signal. If
                 reordering is not possible the empty channels will simply be
                          appended. Only available if instance is configured to
                 support multichannel output.\n NOTE: \n
                     1. The channel signaling (CStreamInfo::pChannelType and
                 CStreamInfo::pChannelIndices) will not be modified. Added empty
                 channels will be signaled with channel type
                        AUDIO_CHANNEL_TYPE::ACT_NONE. \n
                     2. If the parameter value is greater than that of
                 ::AAC_PCM_MAX_OUTPUT_CHANNELS both will be set to the same
                 value. \n
                     3. This parameter will be ignored if the number of encoded
                 audio channels is greater than 8. */
  AAC_PCM_MAX_OUTPUT_CHANNELS =
      0x0012, /*!< Maximum number of PCM output channels. If lower than the
                 number of encoded audio channels, downmixing is applied
                 accordingly (see note 5 for exceptions). If dedicated metadata
                 is available in the stream it will be used to achieve better
                 mixing results. \n -1, 0: Disable downmixing feature. The
                 decoder output contains the same number of channels as the
                 encoded bitstream. \n 1:    All encoded audio configurations
                 with more than one channel will be mixed down to one mono
                 output signal. \n 2:    The decoder performs a stereo mix-down
                 if the number encoded audio channels is greater than two. \n 6:
                 If the number of encoded audio channels is greater than six the
                 decoder performs a mix-down to meet the target output
                 configuration of 3/0/2.1 channels. Only available if instance
                 is configured to support multichannel output. \n 8:    This
                 value is currently needed only together with the channel
                 extension feature. See ::AAC_PCM_MIN_OUTPUT_CHANNELS and note 2
                 below. Only available if instance is configured to support
                 multichannel output. \n NOTE: \n
                     1. Down-mixing of any seven or eight channel configuration
                 not defined in ISO/IEC 14496-3 PDAM 4 is not supported by this
                 software version. \n
                     2. If the parameter value is greater than zero but smaller
                 than ::AAC_PCM_MIN_OUTPUT_CHANNELS both will be set to same
                 value. \n
                     3. This parameter will be ignored if the number of encoded
                 audio channels is greater than 8. */
  AAC_METADATA_PROFILE =
      0x0020, /*!< See ::AAC_MD_PROFILE for all available values. */
  AAC_METADATA_EXPIRY_TIME = 0x0021, /*!< Defines the time in ms after which all
                                        the bitstream associated meta-data (DRC,
                                        downmix coefficients, ...) will be reset
                                        to default if no update has been
                                        received. Negative values disable the
                                        feature. */

  AAC_CONCEAL_METHOD = 0x0100, /*!< Error concealment: Processing method. \n
                                    0: Spectral muting. \n
                                    1: Noise substitution (see ::CONCEAL_NOISE).
                                  \n 2: Energy interpolation (adds additional
                                  signal delay of one frame, see
                                  ::CONCEAL_INTER. only some AOTs are
                                  supported). \n */
  AAC_DRC_BOOST_FACTOR =
      0x0200, /*!< MPEG-4 / MPEG-D Dynamic Range Control (DRC): Scaling factor
                 for boosting gain values. Defines how the boosting DRC factors
                 (conveyed in the bitstream) will be applied to the decoded
                 signal. The valid values range from 0 (don't apply boost
                 factors) to 127 (fully apply boost factors). Default value is 0
                 for MPEG-4 DRC and 127 for MPEG-D DRC. */
  AAC_DRC_ATTENUATION_FACTOR = 0x0201, /*!< MPEG-4 / MPEG-D DRC: Scaling factor
                                          for attenuating gain values. Same as
                                            ::AAC_DRC_BOOST_FACTOR but for
                                          attenuating DRC factors. */
  AAC_DRC_REFERENCE_LEVEL =
      0x0202, /*!< MPEG-4 / MPEG-D DRC: Target reference level / decoder target
                 loudness.\n Defines the level below full-scale (quantized in
                 steps of 0.25dB) to which the output audio signal will be
                 normalized to by the DRC module.\n The parameter controls
                 loudness normalization for both MPEG-4 DRC and MPEG-D DRC. The
                 valid values range from 40 (-10 dBFS) to 127 (-31.75 dBFS).\n
                   Example values:\n
                   124 (-31 dBFS) for audio/video receivers (AVR) or other
                 devices allowing audio playback with high dynamic range,\n 96
                 (-24 dBFS) for TV sets or equivalent devices (default),\n 64
                 (-16 dBFS) for mobile devices where the dynamic range of audio
                 playback is restricted.\n Any value smaller than 0 switches off
                 loudness normalization and MPEG-4 DRC. */
  AAC_DRC_HEAVY_COMPRESSION =
      0x0203, /*!< MPEG-4 DRC: En-/Disable DVB specific heavy compression (aka
                 RF mode). If set to 1, the decoder will apply the compression
                 values from the DVB specific ancillary data field. At the same
                 time the MPEG-4 Dynamic Range Control tool will be disabled. By
                   default, heavy compression is disabled. */
  AAC_DRC_DEFAULT_PRESENTATION_MODE =
      0x0204, /*!< MPEG-4 DRC: Default presentation mode (DRC parameter
                 handling). \n Defines the handling of the DRC parameters boost
                 factor, attenuation factor and heavy compression, if no
                 presentation mode is indicated in the bitstream.\n For options,
                 see ::AAC_DRC_DEFAULT_PRESENTATION_MODE_OPTIONS.\n Default:
                 ::AAC_DRC_PARAMETER_HANDLING_DISABLED */
  AAC_DRC_ENC_TARGET_LEVEL =
      0x0205, /*!< MPEG-4 DRC: Encoder target level for light (i.e. not heavy)
                 compression.\n If known, this declares the target reference
                 level that was assumed at the encoder for calculation of
                 limiting gains. The valid values range from 0 (full-scale) to
                 127 (31.75 dB below full-scale). This parameter is used only
                 with ::AAC_DRC_PARAMETER_HANDLING_ENABLED and ignored
                 otherwise.\n Default: 127 (worst-case assumption).\n */
  AAC_UNIDRC_SET_EFFECT = 0x0206, /*!< MPEG-D DRC: Request a DRC effect type for
                                     selection of a DRC set.\n Supported indices
                                     are:\n -1: DRC off. Completely disables
                                     MPEG-D DRC.\n 0: None (default). Disables
                                     MPEG-D DRC, but automatically enables DRC
                                     if necessary to prevent clipping.\n 1: Late
                                     night\n 2: Noisy environment\n 3: Limited
                                     playback range\n 4: Low playback level\n 5:
                                     Dialog enhancement\n 6: General
                                     compression. Used for generally enabling
                                     MPEG-D DRC without particular request.\n */
  AAC_UNIDRC_ALBUM_MODE =
      0x0207, /*!<  MPEG-D DRC: Enable album mode. 0: Disabled (default), 1:
                 Enabled.\n Disabled album mode leads to application of gain
                 sequences for fading in and out, if provided in the
                 bitstream.\n Enabled album mode makes use of dedicated album
                 loudness information, if provided in the bitstream.\n */
  AAC_QMF_LOWPOWER =
      0x0300, /*!< Quadrature Mirror Filter (QMF) Bank processing mode. \n
                   -1: Use internal default. \n
                    0: Use complex QMF data mode. \n
                    1: Use real (low power) QMF data mode. \n */
  AAC_TPDEC_CLEAR_BUFFER =
      0x0603 /*!< Clear internal bit stream buffer of transport layers. The
                decoder will start decoding at new data passed after this event
                and any previous data is discarded. */

} AACDEC_PARAM;

/**
 * \brief  AAC encoder setting parameters.
 *
 * Use aacEncoder_SetParam() function to configure, or use aacEncoder_GetParam()
 * function to read the internal status of the following parameters.
 */
typedef enum {
  AACENC_AOT =
      0x0100, /*!< Audio object type. See ::AUDIO_OBJECT_TYPE in FDK_audio.h.
                   - 2: MPEG-4 AAC Low Complexity.
                   - 5: MPEG-4 AAC Low Complexity with Spectral Band Replication
                 (HE-AAC).
                   - 29: MPEG-4 AAC Low Complexity with Spectral Band
                 Replication and Parametric Stereo (HE-AAC v2). This
                 configuration can be used only with stereo input audio data.
                   - 23: MPEG-4 AAC Low-Delay.
                   - 39: MPEG-4 AAC Enhanced Low-Delay. Since there is no
                 ::AUDIO_OBJECT_TYPE for ELD in combination with SBR defined,
                 enable SBR explicitely by ::AACENC_SBR_MODE parameter. The ELD
                 v2 212 configuration can be configured by ::AACENC_CHANNELMODE
                 parameter.
                   - 129: MPEG-2 AAC Low Complexity.
                   - 132: MPEG-2 AAC Low Complexity with Spectral Band
                 Replication (HE-AAC).

                   Please note that the virtual MPEG-2 AOT's basically disables
                 non-existing Perceptual Noise Substitution tool in AAC encoder
                 and controls the MPEG_ID flag in adts header. The virtual
                 MPEG-2 AOT doesn't prohibit specific transport formats. */

  AACENC_BITRATE = 0x0101, /*!< Total encoder bitrate. This parameter is
                              mandatory and interacts with ::AACENC_BITRATEMODE.
                                - CBR: Bitrate in bits/second.
                                - VBR: Variable bitrate. Bitrate argument will
                              be ignored. See \ref suppBitrates for details. */

  AACENC_BITRATEMODE = 0x0102, /*!< Bitrate mode. Configuration can be different
                                  kind of bitrate configurations:
                                    - 0: Constant bitrate, use bitrate according
                                  to ::AACENC_BITRATE. (default) Within none
                                  LD/ELD ::AUDIO_OBJECT_TYPE, the CBR mode makes
                                  use of full allowed bitreservoir. In contrast,
                                  at Low-Delay ::AUDIO_OBJECT_TYPE the
                                  bitreservoir is kept very small.
                                    - 1: Variable bitrate mode, \ref vbrmode
                                  "very low bitrate".
                                    - 2: Variable bitrate mode, \ref vbrmode
                                  "low bitrate".
                                    - 3: Variable bitrate mode, \ref vbrmode
                                  "medium bitrate".
                                    - 4: Variable bitrate mode, \ref vbrmode
                                  "high bitrate".
                                    - 5: Variable bitrate mode, \ref vbrmode
                                  "very high bitrate". */

  AACENC_SAMPLERATE = 0x0103, /*!< Audio input data sampling rate. Encoder
                                 supports following sampling rates: 8000, 11025,
                                 12000, 16000, 22050, 24000, 32000, 44100,
                                 48000, 64000, 88200, 96000 */

  AACENC_SBR_MODE = 0x0104, /*!< Configure SBR independently of the chosen Audio
                               Object Type ::AUDIO_OBJECT_TYPE. This parameter
                               is for ELD audio object type only.
                                 - -1: Use ELD SBR auto configurator (default).
                                 - 0: Disable Spectral Band Replication.
                                 - 1: Enable Spectral Band Replication. */

  AACENC_GRANULE_LENGTH =
      0x0105, /*!< Core encoder (AAC) audio frame length in samples:
                   - 1024: Default configuration.
                   - 512: Default length in LD/ELD configuration.
                   - 480: Length in LD/ELD configuration.
                   - 256: Length for ELD reduced delay mode (x2).
                   - 240: Length for ELD reduced delay mode (x2).
                   - 128: Length for ELD reduced delay mode (x4).
                   - 120: Length for ELD reduced delay mode (x4). */

  AACENC_CHANNELMODE = 0x0106, /*!< Set explicit channel mode. Channel mode must
                                  match with number of input channels.
                                    - 1-7, 11,12,14 and 33,34: MPEG channel
                                  modes supported, see ::CHANNEL_MODE in
                                  FDK_audio.h. */

  AACENC_CHANNELORDER =
      0x0107, /*!< Input audio data channel ordering scheme:
                   - 0: MPEG channel ordering (e. g. 5.1: C, L, R, SL, SR, LFE).
                 (default)
                   - 1: WAVE file format channel ordering (e. g. 5.1: L, R, C,
                 LFE, SL, SR). */

  AACENC_SBR_RATIO =
      0x0108, /*!<  Controls activation of downsampled SBR. With downsampled
                 SBR, the delay will be shorter. On the other hand, for
                 achieving the same quality level, downsampled SBR needs more
                 bits than dual-rate SBR. With downsampled SBR, the AAC encoder
                 will work at the same sampling rate as the SBR encoder (single
                 rate). Downsampled SBR is supported for AAC-ELD and HE-AACv1.
                    - 1: Downsampled SBR (default for ELD).
                    - 2: Dual-rate SBR   (default for HE-AAC). */

  AACENC_AFTERBURNER =
      0x0200, /*!< This parameter controls the use of the afterburner feature.
                   The afterburner is a type of analysis by synthesis algorithm
                 which increases the audio quality but also the required
                 processing power. It is recommended to always activate this if
                 additional memory consumption and processing power consumption
                   is not a problem. If increased MHz and memory consumption are
                 an issue then the MHz and memory cost of this optional module
                 need to be evaluated against the improvement in audio quality
                 on a case by case basis.
                   - 0: Disable afterburner (default).
                   - 1: Enable afterburner. */

  AACENC_BANDWIDTH = 0x0203, /*!< Core encoder audio bandwidth:
                                  - 0: Determine audio bandwidth internally
                                (default, see chapter \ref BEHAVIOUR_BANDWIDTH).
                                  - 1 to fs/2: Audio bandwidth in Hertz. Limited
                                to 20kHz max. Not usable if SBR is active. This
                                setting is for experts only, better do not touch
                                this value to avoid degraded audio quality. */

  AACENC_PEAK_BITRATE =
      0x0207, /*!< Peak bitrate configuration parameter to adjust maximum bits
                 per audio frame. Bitrate is in bits/second. The peak bitrate
                 will internally be limited to the chosen bitrate
                 ::AACENC_BITRATE as lower limit and the
                 number_of_effective_channels*6144 bit as upper limit.

                   Setting the peak bitrate equal to ::AACENC_BITRATE does not
                 necessarily mean that the audio frames will be of constant
                 size. Since the peak bitate is in bits/second, the frame sizes
                 can vary by one byte in one or the other direction over various
                 frames. However, it is not recommended to reduce the peak
                 pitrate to ::AACENC_BITRATE - it would disable the
                 bitreservoir, which would affect the audio quality by a large
                 amount. */

  AACENC_TRANSMUX = 0x0300, /*!< Transport type to be used. See ::TRANSPORT_TYPE
                               in FDK_audio.h. Following types can be configured
                               in encoder library:
                                 - 0: raw access units
                                 - 1: ADIF bitstream format
                                 - 2: ADTS bitstream format
                                 - 6: Audio Mux Elements (LATM) with
                               muxConfigPresent = 1
                                 - 7: Audio Mux Elements (LATM) with
                               muxConfigPresent = 0, out of band StreamMuxConfig
                                 - 10: Audio Sync Stream (LOAS) */

  AACENC_HEADER_PERIOD =
      0x0301, /*!< Frame count period for sending in-band configuration buffers
                 within LATM/LOAS transport layer. Additionally this parameter
                 configures the PCE repetition period in raw_data_block(). See
                 \ref encPCE.
                   - 0xFF: auto-mode default 10 for TT_MP4_ADTS, TT_MP4_LOAS and
                 TT_MP4_LATM_MCP1, otherwise 0.
                   - n: Frame count period. */

  AACENC_SIGNALING_MODE =
      0x0302, /*!< Signaling mode of the extension AOT:
                   - 0: Implicit backward compatible signaling (default for
                 non-MPEG-4 based AOT's and for the transport formats ADIF and
                 ADTS)
                        - A stream that uses implicit signaling can be decoded
                 by every AAC decoder, even AAC-LC-only decoders
                        - An AAC-LC-only decoder will only decode the
                 low-frequency part of the stream, resulting in a band-limited
                 output
                        - This method works with all transport formats
                        - This method does not work with downsampled SBR
                   - 1: Explicit backward compatible signaling
                        - A stream that uses explicit backward compatible
                 signaling can be decoded by every AAC decoder, even AAC-LC-only
                 decoders
                        - An AAC-LC-only decoder will only decode the
                 low-frequency part of the stream, resulting in a band-limited
                 output
                        - A decoder not capable of decoding PS will only decode
                 the AAC-LC+SBR part. If the stream contained PS, the result
                 will be a a decoded mono downmix
                        - This method does not work with ADIF or ADTS. For
                 LOAS/LATM, it only works with AudioMuxVersion==1
                        - This method does work with downsampled SBR
                   - 2: Explicit hierarchical signaling (default for MPEG-4
                 based AOT's and for all transport formats excluding ADIF and
                 ADTS)
                        - A stream that uses explicit hierarchical signaling can
                 be decoded only by HE-AAC decoders
                        - An AAC-LC-only decoder will not decode a stream that
                 uses explicit hierarchical signaling
                        - A decoder not capable of decoding PS will not decode
                 the stream at all if it contained PS
                        - This method does not work with ADIF or ADTS. It works
                 with LOAS/LATM and the MPEG-4 File format
                        - This method does work with downsampled SBR

                    For making sure that the listener always experiences the
                 best audio quality, explicit hierarchical signaling should be
                 used. This makes sure that only a full HE-AAC-capable decoder
                 will decode those streams. The audio is played at full
                 bandwidth. For best backwards compatibility, it is recommended
                 to encode with implicit SBR signaling. A decoder capable of
                 AAC-LC only will then only decode the AAC part, which means the
                 decoded audio will sound band-limited.

                    For MPEG-2 transport types (ADTS,ADIF), only implicit
                 signaling is possible.

                    For LOAS and LATM, explicit backwards compatible signaling
                 only works together with AudioMuxVersion==1. The reason is
                 that, for explicit backwards compatible signaling, additional
                 information will be appended to the ASC. A decoder that is only
                 capable of decoding AAC-LC will skip this part. Nevertheless,
                 for jumping to the end of the ASC, it needs to know the ASC
                 length. Transmitting the length of the ASC is a feature of
                 AudioMuxVersion==1, it is not possible to transmit the length
                 of the ASC with AudioMuxVersion==0, therefore an AAC-LC-only
                 decoder will not be able to parse a LOAS/LATM stream that was
                 being encoded with AudioMuxVersion==0.

                    For downsampled SBR, explicit signaling is mandatory. The
                 reason for this is that the extension sampling frequency (which
                 is in case of SBR the sampling frequqncy of the SBR part) can
                 only be signaled in explicit mode.

                    For AAC-ELD, the SBR information is transmitted in the
                 ELDSpecific Config, which is part of the AudioSpecificConfig.
                 Therefore, the settings here will have no effect on AAC-ELD.*/

  AACENC_TPSUBFRAMES =
      0x0303, /*!< Number of sub frames in a transport frame for LOAS/LATM or
                 ADTS (default 1).
                   - ADTS: Maximum number of sub frames restricted to 4.
                   - LOAS/LATM: Maximum number of sub frames restricted to 2.*/

  AACENC_AUDIOMUXVER =
      0x0304, /*!< AudioMuxVersion to be used for LATM. (AudioMuxVersionA,
                 currently not implemented):
                   - 0: Default, no transmission of tara Buffer fullness, no ASC
                 length and including actual latm Buffer fullnes.
                   - 1: Transmission of tara Buffer fullness, ASC length and
                 actual latm Buffer fullness.
                   - 2: Transmission of tara Buffer fullness, ASC length and
                 maximum level of latm Buffer fullness. */

  AACENC_PROTECTION = 0x0306, /*!< Configure protection in transport layer:
                                   - 0: No protection. (default)
                                   - 1: CRC active for ADTS transport format. */

  AACENC_ANCILLARY_BITRATE =
      0x0500, /*!< Constant ancillary data bitrate in bits/second.
                   - 0: Either no ancillary data or insert exact number of
                 bytes, denoted via input parameter, numAncBytes in
                 AACENC_InArgs.
                   - else: Insert ancillary data with specified bitrate. */

  AACENC_METADATA_MODE = 0x0600, /*!< Configure Meta Data. See ::AACENC_MetaData
                                    for further details:
                                      - 0: Do not embed any metadata.
                                      - 1: Embed dynamic_range_info metadata.
                                      - 2: Embed dynamic_range_info and
                                    ancillary_data metadata.
                                      - 3: Embed ancillary_data metadata. */

  AACENC_CONTROL_STATE =
      0xFF00, /*!< There is an automatic process which internally reconfigures
                 the encoder instance when a configuration parameter changed or
                 an error occured. This paramerter allows overwriting or getting
                 the control status of this process. See ::AACENC_CTRLFLAGS. */

  AACENC_NONE = 0xFFFF /*!< ------ */

} AACENC_PARAM;


}