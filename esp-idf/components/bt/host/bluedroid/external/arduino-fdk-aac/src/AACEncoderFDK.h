#pragma once

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include "fdk_log.h"
#include "libAACenc/aacenc_lib.h"

namespace aac_fdk {

typedef void (*AACCallbackFDK)(uint8_t *aac_data, size_t len);

struct AudioInfo  {
    AudioInfo () = default;
    AudioInfo (const AudioInfo  &) = default;
    int sample_rate = 44100;    
    int channels = 1;      
    int bits_per_sample=16; // we assume int16_t
};

/**
 * @brief Encodes PCM data to the AAC format and writes the result to a stream
 * 
 */
class AACEncoderFDK  {

public:
	/// Empty Constructor. 
	AACEncoderFDK(){
	}

	/// Default constructor
	AACEncoderFDK(AACCallbackFDK cb){
        setDataCallback(cb);
	}

	/// Destructor
	~AACEncoderFDK(){
		 end();
	}


#ifdef ARDUINO

	AACEncoderFDK(Stream &out_stream){
		this->out = &out_stream;
	}

	/// Defines the output stream
	void setOutput(Print &out_stream){
		this->out = &out_stream;
	}

#endif

	/// Defines the callback method
	void setDataCallback(AACCallbackFDK cb){
		this->aacCallback = cb;
	}

	/** 
	 * @brief Total encoder bitrate. This parameter is	
				mandatory and interacts with ::AACENC_BITRATEMODE.
				- CBR: Bitrate in bits/second.
				- VBR: Variable bitrate. Bitrate argument will
				be ignored. See \ref suppBitrates for details. 
	*/	
	void setBitrate(int bitrate){
		this->bitrate = bitrate;
	}

	/** 
	 * @brief  Audio object type. See ::AUDIO_OBJECT_TYPE in FDK_audio.h.
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

                   Please note that the  MPEG-2 AOT's basically disables
                 non-existing Perceptual Noise Substitution tool in AAC encoder
                 and controls the MPEG_ID flag in adts header. The 
                 MPEG-2 AOT doesn't prohibit specific transport formats. 
			*/
	void setAudioObjectType(int aot){
		this->aot = aot;
	}

	/** 
	 * @brief  This parameter controls the use of the afterburner feature.
                   The afterburner is a type of analysis by synthesis algorithm
                 which increases the audio quality but also the required
                 processing power. It is recommended to always activate this if
                 additional memory consumption and processing power consumption
                   is not a problem. If increased MHz and memory consumption are
                 an issue then the MHz and memory cost of this optional module
                 need to be evaluated against the improvement in audio quality
                 on a case by case basis.
                   - 0: Disable afterburner (default).
                   - 1: Enable afterburner. 
			*/
	void setAfterburner(bool afterburner){
		this->afterburner = afterburner;
	}

	/** 
	 * @brief  Configure SBR independently of the chosen Audio
				Object Type ::AUDIO_OBJECT_TYPE. This parameter
				is for ELD audio object type only.
					- -1: Use ELD SBR auto configurator (default).
					- 0: Disable Spectral Band Replication.
					- 1: Enable Spectral Band Replication. */	
	void setSpectralBandReplication(int eld_sbr){
		this->eld_sbr = eld_sbr;
	}

 	/** 
	  * @brief  Bitrate mode. Configuration can be different
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
				"very high bitrate". 
		*/	
	void setVariableBitrateMode(int vbr){
		this->vbr = vbr;
	}
	
	/**
	 * @brief Set the Output Buffer Size object
	 * 
	 * @param outbuf_size 
	 */
	void setOutputBufferSize(int outbuf_size){
		this->out_size = outbuf_size;
	}


	/**
	 * @brief Defines/Updates the Audio Info 
	 * @param from 
	 */ 
     void setAudioInfo(AudioInfo  from) {
		LOG_FDK(FDKDebug,__FUNCTION__);
		this->channels = from.channels;
		this->sample_rate = from.sample_rate;
		this->bits_per_sample = from.bits_per_sample;
    }
	
	/**
	 * @brief Opens the encoder - Please call setAudioInfo before
	 * 
	 */
	 bool begin() {
		LOG_FDK(FDKDebug,__FUNCTION__);
		return setup();
	}

	/**
	 * @brief Opens the encoder  
	 * 
	 * @param info 
	 * @return int 
	 */
	 bool begin(AudioInfo  info) {
		LOG_FDK(FDKDebug,__FUNCTION__);
		setAudioInfo(info);
		return setup();
	}

	/**
	 * @brief Opens the encoder  
	 * 
	 * @param input_channels 
	 * @param input_sample_rate 
	 * @param input_bits_per_sample 
	 * @return int 0 => ok; error with negative number
	 */
	 bool begin(int input_channels, int input_sample_rate, int input_bits_per_sample) {
		LOG_FDK(FDKDebug,__FUNCTION__);
		AudioInfo ai;
		ai.channels = input_channels;
		ai.sample_rate = input_sample_rate;
		ai.bits_per_sample = input_bits_per_sample;
		setAudioInfo(ai);
		return setup();
	}

	/// write PCM data to be converted to AAC - The size is in bytes
	int32_t write(uint8_t *in_ptr, int in_size){
		LOG_FDK(FDKDebug,"write %d bytes", in_size);
		in_elem_size = this->bits_per_sample <= 16 ? 2 : 4;

		in_args.numInSamples = in_size <= 0 ? -1 : in_size / 2;
		in_buf.numBufs = 1;
		in_buf.bufs = (void**) &in_ptr;
		in_buf.bufferIdentifiers = &in_identifier;
		in_buf.bufSizes = &in_size;
		in_buf.bufElSizes = &in_elem_size;

		out_elem_size = 1;
		out_buf.numBufs = 1;
		out_buf.bufs = (void**) &outbuf;
		out_buf.bufferIdentifiers = &out_identifier;
		out_buf.bufSizes = &out_size;
		out_buf.bufElSizes = &out_elem_size;
		
		err = aacEncEncode(handle, &in_buf, &out_buf, &in_args, &out_args);
		if (err != AACENC_OK) {
			// error
			if (err != AACENC_ENCODE_EOF) {
				LOG_FDK(FDKError,"Encoding failed: %s\n", setupErrorText(err));
				return 0;
			}
		}

		// output to Arduino Stream	
		provideResult((uint8_t*)outbuf, out_args.numOutBytes);
		return in_size;
	}

	/// closes the processing and release resources
	void end(){
		LOG_FDK(FDKDebug,__FUNCTION__);
		active = false;

		if (outbuf!=nullptr){
			delete []outbuf;
			outbuf=nullptr;
		}

		aacEncClose(&handle);
	}

	/// determines a decoder parameter
	UINT getParameter(const AACENC_PARAM param) {
		return aacEncoder_GetParam(handle, param);
	}

	/// sets an encoder parameter
	int setParameter(AACENC_PARAM param, uint32_t value){
		return aacEncoder_SetParam(handle, param, value);
	}

	operator boolean(){
		return active;
	}

protected:
	// common variables
	int vbr = 1; // variable bitrate mode
	int bitrate = 0; // automatic determination
	int ch = 0;
	int format, sample_rate, channels=2, bits_per_sample;
	int aot = 2;
	bool afterburner = false;
	int eld_sbr = 0;
	HANDLE_AACENCODER handle;
	CHANNEL_MODE mode;
	AACENC_InfoStruct info = { 0 };
	// loop variables
	AACENC_BufDesc in_buf = { 0 }, out_buf = { 0 };
	AACENC_InArgs in_args = { 0 };
	AACENC_OutArgs out_args = { 0 };
	int in_identifier = IN_AUDIO_DATA;
	int in_elem_size;
	int out_identifier = OUT_BITSTREAM_DATA;
	int out_elem_size=1;
	uint8_t* outbuf = nullptr;
	int out_size = 2048;
	AACENC_ERROR err;
	bool active = false;
	AACCallbackFDK aacCallback=nullptr;
	UINT encModules = 0x01; 
	UINT openEncModules = 0; 
	int openChannels = 0;
	int sce=0, cpe=0; // for bitrate determination

#ifdef ARDUINO
	Print *out;
#endif

	/// starts the processing
	bool setup() {
		LOG_FDK(FDKDebug,__FUNCTION__);
		AACENC_ERROR rc = AACENC_OK;

		// determine mode
		switch (channels) {
		case 1: mode = MODE_1;       sce = 1; cpe = 0; break;
		case 2: mode = MODE_2;       sce = 0; cpe = 1; break;
		case 3: mode = MODE_1_2;     sce = 1; cpe = 1; break;
		case 4: mode = MODE_1_2_1;   sce = 2; cpe = 1; break;
		case 5: mode = MODE_1_2_2;   sce = 1; cpe = 2; break;
		case 6: mode = MODE_1_2_2_1; sce = 2; cpe = 2; break;
		default:
			LOG_FDK(FDKError,"Unsupported WAV channels\n");
			return false;
		}

		// reopen encoder if modules or channels have changed
		if (active && (channels>openChannels || openEncModules!=encModules)){
			LOG_FDK(FDKWarning,"Basic Info has changed: we reopen the encoder\n");
			rc = aacEncClose(&handle);
			if (rc != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to close encoder: %s\n",setupErrorText(rc));
			} 
			active = false;
		}

		// open only once !
		if (!active) {
			LOG_FDK(FDKInfo,"aacEncOpen\n");
			rc = aacEncOpen(&handle, encModules, channels);

			if (rc != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to open encoder: %s\n",setupErrorText(rc));
				return false;
			} 
			// record open channels and modules
			openChannels = channels;
			openEncModules = encModules;

			if (updateParams()<0) {
				LOG_FDK(FDKError,"Unable to update parameters\n");
				return false;
			}

			if (aacEncEncode(handle, NULL, NULL, NULL, NULL) != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to initialize the encoder\n");
				return false;
			}

			if (aacEncInfo(handle, &info) != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to get the encoder info\n");
				return false;
			}

			if (outbuf==nullptr){
				outbuf = new uint8_t[out_size];
				if (outbuf==nullptr){
					LOG_FDK(FDKError,"Unable to allocate memory for output buffer\n");
					return false;
				}
			}
		} else {
			// we might need to update the parameters
			if (updateParams()<0) {
				LOG_FDK(FDKError,"Unable to update parameters\n");
				return false;
			}
		}

		active = true;
		return true;
	}
	
	/// Updates all parameters with the values defined by the setter methods
	int updateParams() {
		LOG_FDK(FDKDebug,__FUNCTION__);

		if (setParameter(AACENC_AOT, aot) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the AOT (Audio Object Type)\n");
			return -1;
		}
		if (aot == 39 && eld_sbr) {
			if (setParameter(AACENC_SBR_MODE, 1) != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to set SBR (Spectral Band Replication) mode for ELD\n");
				return -1;
			}
		}
		if (setParameter(AACENC_SAMPLERATE, sample_rate) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the AACENC_SAMPLERATE\n");
			return -1;
		}
		if (setParameter(AACENC_CHANNELMODE, mode) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the channel mode\n");
			return -1;
		}
		if (setParameter(AACENC_CHANNELORDER, 1) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the wav channel order\n");
			return -1;
		}

		if (vbr) {
			if (setParameter(AACENC_BITRATEMODE, vbr) != AACENC_OK) {
				LOG_FDK(FDKError,"Unable to set the VBR bitrate mode\n");
				return -1;
			}
		} else {
			// determine bitrate
			if (bitrate<=0){
			   if (aot==29){
					sce = 1;
  					cpe = 0;
			   }	
               bitrate = (96*sce + 128*cpe) * sample_rate / 44;
               if (aot==5 || aot== 29 || aot==132 || eld_sbr){
                   bitrate /= 2;
               }
				LOG_FDK(FDKWarning,"Determined bitrate: %d for sample rate %d\n", bitrate, sample_rate);
			}

			if (bitrate>0) {
				if (setParameter(AACENC_BITRATE, bitrate) != AACENC_OK) {
					LOG_FDK(FDKError,"Unable to set the bitrate\n");
					return -1;
				}
			}
		}
		if (setParameter(AACENC_TRANSMUX, TT_MP4_ADTS) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the ADTS transmux\n");
			return -1;
		}
		if (setParameter(AACENC_AFTERBURNER, afterburner) != AACENC_OK) {
			LOG_FDK(FDKError,"Unable to set the afterburner mode\n");
			return -1;
		}
		return 0;
	}

	/// return the result PWM data
	void provideResult(uint8_t *data, size_t len){
		if (len>0){
			LOG_FDK(FDKDebug, "provideResult: %zu samples",len);
			// provide result
			if(aacCallback!=nullptr){
				// output via callback
				aacCallback(data, len);
			} 
#ifdef ARDUINO
			if (out!=nullptr){
				out->write(data, len);
			}
#endif
		}
	} 

	/// Specify encoder modules to be supported in this encoder  (0x01: AAC module,0x02: SBR module,0x04: PS module,0x10: Metadata module). E.g. (0x01|0x02|0x04|0x10) - Use 0 for all modules
	void setEncoderModules(UINT encModules){
		this->encModules = encModules;
	}

	/// convert error code to error text
	const char* setupErrorText(int no) {
		switch(no){
			case AACENC_OK: 
				return "No error";
			case AACENC_INVALID_HANDLE:
				return "Handle passed to function call was invalid";
			case AACENC_MEMORY_ERROR:
				return "Memory allocation failed";
			case AACENC_UNSUPPORTED_PARAMETER: 
				return "Parameter not available";
			case AACENC_INVALID_CONFIG:
				return "Configuration not provided";
			case AACENC_INIT_ERROR: 
				return "General initialization error";
			case AACENC_INIT_AAC_ERROR: 
				return "AAC library initialization error";
			case AACENC_INIT_SBR_ERROR: 
				return "SBR library initialization error";
			case AACENC_INIT_TP_ERROR:
				return "Transport library initialization error";
			case AACENC_INIT_META_ERROR: 
				return "Meta data library initialization error";
			case AACENC_INIT_MPS_ERROR:
				return " MPS library initialization error";
			case AACENC_ENCODE_ERROR:
				return "The encoding process was interrupted by an unexpected error";
			case AACENC_ENCODE_EOF:
				return "End of file reached";

		}   
		return "n/a";	
	}

};

}
