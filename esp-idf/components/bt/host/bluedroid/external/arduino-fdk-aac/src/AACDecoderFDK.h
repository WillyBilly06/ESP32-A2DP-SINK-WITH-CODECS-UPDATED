#pragma once

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include "fdk_log.h"
#include "libAACdec/aacdecoder_lib.h"

#define FDK_OUT_BUFFER_DEFAULT_SIZE 1024*4

namespace aac_fdk {

typedef void (*AACInfoCallbackFDK)(CStreamInfo &info);
typedef void (*AACDataCallbackFDK)(CStreamInfo &info,INT_PCM *pcm_data, size_t len);

/**
 * @brief Audio Decoder which decodes AAC into a PCM stream
 * 
 */
class AACDecoderFDK  {
    public:
		/**
		 * @brief Construct a new AACDecoderFDK object
		 * 
		 * @param output_buffer_size 
		 */
        AACDecoderFDK(int output_buffer_size=FDK_OUT_BUFFER_DEFAULT_SIZE){
        	LOG_FDK(FDKDebug,__FUNCTION__);
            this->output_buffer_size = output_buffer_size;
		}

		/**
		 * @brief Construct a new AACDecoderFDK object
		 * 
		 * @param dataCallback 
		 * @param infoCallback (default nullptr)
		 * @param output_buffer_size (default 2048)
		 */
        AACDecoderFDK(AACDataCallbackFDK dataCallback, AACInfoCallbackFDK infoCallback=nullptr, int output_buffer_size=FDK_OUT_BUFFER_DEFAULT_SIZE){
            this->output_buffer_size = output_buffer_size;
            setDataCallback(dataCallback);
            setInfoCallback(infoCallback);
        }

#ifdef ARDUINO

		/**
		 * @brief Construct a new AACDecoderFDK object
		 * 
		 * @param out_stream 
		 * @param output_buffer_size 
		 */
        AACDecoderFDK(Print &out_stream, int output_buffer_size=FDK_OUT_BUFFER_DEFAULT_SIZE){
        	LOG_FDK(FDKDebug,__FUNCTION__);
            this->output_buffer_size = output_buffer_size;
			setOutput(out_stream);
        }

		/// Defines the output stream
		void setOutput(Print &out_stream){
            this->out = &out_stream;
		}

#endif
        virtual ~AACDecoderFDK(){
            end();
        }

        void setInfoCallback(AACInfoCallbackFDK cb){
            this->infoCallback = cb;
        }

        void setDataCallback(AACDataCallbackFDK cb){
            this->pwmCallback = cb;
        }

		/**
		 * \brief Explicitly configure the decoder by passing a raw AudioSpecificConfig
		 * (ASC) or a StreamMuxConfig (SMC), contained in a binary buffer. This is
		 * required for MPEG-4 and Raw Packets file format bitstreams as well as for
		 * LATM bitstreams with no in-band SMC. If the transport format is LATM with or
		 * without LOAS, configuration is assumed to be an SMC, for all other file
		 * formats an ASC.
		 *
		 * \param cfg    Pointer to an unsigned char buffer containing the binary
		 * configuration buffer (either ASC or SMC).
		 * \param len  Length of the configuration buffer in bytes.
		 * \return        Error code.
		 */
		int setRawConfig(void* cfg, UINT &len){
			uint8_t *ptr = (uint8_t *)cfg;
			return aacDecoder_ConfigRaw(aacDecoderInfo, &ptr, &len);
		}

		/**
		* @brief Set the Decoder Flags object
		* Bit field with flags for the decoder: \n
		*                      (flags & AACDEC_CONCEAL) == 1: Do concealment. \n
		*                      (flags & AACDEC_FLUSH) == 2: Discard input data. Flush
		* filter banks (output delayed audio). \n (flags & AACDEC_INTR) == 4: Input
		* data is discontinuous. Resynchronize any internals as
		* necessary. \n (flags & AACDEC_CLRHIST) == 8: Clear all signal delay lines and
		* history buffers.
		 * @param flags 
		 */
		void setDecoderFlags(int flags){
			decoder_flags = flags;
		}

        // opens the decoder
        bool begin(TRANSPORT_TYPE transportType=TT_UNKNOWN, UINT nrOfLayers=1){
			LOG_FDK(FDKDebug,__FUNCTION__);

			// call aacDecoder_Open only once
			if (aacDecoderInfo == nullptr){ 
				LOG_FDK(FDKDebug,__FUNCTION__);
				aacDecoderInfo = aacDecoder_Open(transportType, nrOfLayers);
				//assert(aacDecoderInfo!=nullptr);
				if (aacDecoderInfo == nullptr){
					LOG_FDK(FDKError,"aacDecoder_Open -> Error");
					return false;
				}
			}

			// allocate buffer only once
			if (output_buffer == nullptr){
				LOG_FDK(FDKDebug,__FUNCTION__);
				output_buffer = (INT_PCM*) FDKcalloc(output_buffer_size, sizeof(INT_PCM));
				assert(output_buffer != nullptr);
				if (output_buffer == nullptr){
					LOG_FDK(FDKError,"aacDecoder_Open -> Error");
					return false;
				} 
			}

			// if we decode 1 channel aac files we return output to 2 channels
			aacDecoder_SetParam(aacDecoderInfo, AAC_PCM_MIN_OUTPUT_CHANNELS, 2);
			is_open = true;
            return true;
        }

        /**
         * @brief Explicitly configure the decoder by passing a raw AudioSpecificConfig (ASC) or a StreamMuxConfig
         * (SMC), contained in a binary buffer. This is required for MPEG-4 and Raw Packets file format bitstreams
         * as well as for LATM bitstreams with no in-band SMC. If the transport format is LATM with or without
         * LOAS, configuration is assumed to be an SMC, for all other file formats an ASC.
         * 
         **/
        AAC_DECODER_ERROR configure(uint8_t *conf, const UINT &length) {
            return aacDecoder_ConfigRaw (aacDecoderInfo, &conf, &length );
        }

        // write AAC data to be converted to PCM data - we feed the decoder whitch batches of max 1k
      	virtual size_t write(const void *in_ptr, size_t in_size) {
			if (aacDecoderInfo==nullptr) return 0;
			LOG_FDK(FDKDebug,"write %zu bytes", in_size);
			uint8_t *byte_ptr = (uint8_t *)in_ptr;
			size_t open = in_size;
			int pos = 0;
			while(open>0){
				// a frame is between 1 and 768 bytes => so we feed the decoder with small chunks
				size_t len = std::min<int>(open, 256);
				int decoded = decode(byte_ptr+pos, len);
				pos+=decoded;
				open-=decoded;
			}
            return pos;
        }

        // provides detailed information about the stream
        CStreamInfo audioInfo(){
            return *aacDecoder_GetStreamInfo(aacDecoderInfo);
        }

        // release the resources
        void end(){
	 		LOG_FDK(FDKDebug,__FUNCTION__);
            if (aacDecoderInfo!=nullptr){
                aacDecoder_Close(aacDecoderInfo); 
                aacDecoderInfo = nullptr;
            }
            if (output_buffer!=nullptr){
                FDKfree(output_buffer);
                output_buffer = nullptr;
            }
			is_open = false;
        }

	   /// returns true if the decoder is open
       virtual operator boolean() {
		   return is_open;
	   }

    protected:
		bool is_cleanup_stream = false;
        HANDLE_AACDECODER aacDecoderInfo = nullptr;
        int output_buffer_size = 0;
        INT_PCM* output_buffer = nullptr;
		bool is_open = false;
		CStreamInfo aacFrameInfo;
        AACDataCallbackFDK pwmCallback = nullptr;
        AACInfoCallbackFDK infoCallback = nullptr;
		int decoder_flags = AACDEC_INTR;

#ifdef ARDUINO
        Print *out = nullptr;
#endif

		/// decodes the data
      	virtual size_t decode(const void *in_ptr, UINT in_size) {
			LOG_FDK(FDKDebug,"decode %u bytes", in_size);
			size_t result = 0;
			UINT inSize = in_size;
			UINT bytesValid = in_size;
			
			AAC_DECODER_ERROR error; 
			if (aacDecoderInfo!=nullptr) {
				const void *start = in_ptr;
				LOG_FDK(FDKDebug,"aacDecoder_Fill %u bytes", inSize);
				error = aacDecoder_Fill(aacDecoderInfo, (UCHAR **)&start, &inSize, &bytesValid); 
				while (error == AAC_DEC_OK) {
					LOG_FDK(FDKDebug,"aacDecoder_DecodeFrame");
					error = aacDecoder_DecodeFrame(aacDecoderInfo, output_buffer, output_buffer_size, decoder_flags); 
					// write pcm to output stream
					if (error == AAC_DEC_OK){
						LOG_FDK(FDKDebug,"provideResult %d", output_buffer_size);
						provideResult(output_buffer, output_buffer_size);
					} else {
						if (error != AAC_DEC_NOT_ENOUGH_BITS){
							LOG_FDK(FDKError,"Decoding error: 0x%x",error);
						}
					}
				}
			}
			result = inSize - bytesValid;
            return result;
        }


        /// return the result PWM data
        void provideResult(INT_PCM *data, size_t len){
            LOG_FDK(FDKDebug, "provideResult: %zu samples",len);
             if (len>0){
				 CStreamInfo info = audioInfo();
            	// provide result
                if(pwmCallback!=nullptr){
                    // output via callback
                    pwmCallback(info, data,len);
                } else {
                    // output to stream
                    if (info.aacSampleRate!=aacFrameInfo.aacSampleRate && infoCallback!=nullptr){
                        infoCallback(info);
                    }
#ifdef ARDUINO
                    out->write((uint8_t*) data, len*sizeof(INT_PCM));
#endif
                }
                aacFrameInfo = info;
            }
        }            

};

}
