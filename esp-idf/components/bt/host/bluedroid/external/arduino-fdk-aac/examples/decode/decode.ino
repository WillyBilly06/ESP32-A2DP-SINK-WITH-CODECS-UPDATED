
/**
 * @file output_aac.ino
 * @author Phil Schatzmann
 * @brief We just display the decoded audio data on the serial monitor
 * @version 0.1
 * @date 2021-07-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "AACDecoderFDK.h"
#include "BabyElephantWalk60_aac.h"

using namespace aac_fdk;

void dataCallback(CStreamInfo &info, INT_PCM *pcm_data, size_t len) {
    int channels = info.numChannels;
    for (size_t i=0; i<len; i+=channels){
        for (int j=0;j<channels;j++){
            Serial.print(pcm_data[i+j]);
            Serial.print(" ");
        }
        Serial.println();
    }
}

AACDecoderFDK aac(dataCallback);

void setup() {
    Serial.begin(115200);
    LOGLEVEL_FDK = FDKDebug;
    if (!aac.begin()){
        Serial.println("aac.begin() failed");
        while(1);
    }
}

void loop() {
    Serial.println("writing...");
    aac.write(BabyElephantWalk60_aac, BabyElephantWalk60_aac_len);    

    // restart from the beginning
    delay(2000);
    aac.begin();
}
