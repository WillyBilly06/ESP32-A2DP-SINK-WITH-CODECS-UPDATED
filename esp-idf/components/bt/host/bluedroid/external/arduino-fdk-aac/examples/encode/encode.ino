
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
#include "AACEncoderFDK.h"
#include <stdlib.h>     /* srand, rand */

using namespace aac_fdk;

void dataCallback(uint8_t *aac_data, size_t len) {
    Serial.print("AAC generated with ");
    Serial.print(len);
    Serial.println(" bytes");
}

AACEncoderFDK aac(dataCallback);
AudioInfo info;
int16_t buffer[512];

void setup() {
    Serial.begin(115200);
    LOGLEVEL_FDK = FDKInfo;

    info.channels = 1;
    info.sample_rate = 44100;
    aac.begin(info);

    Serial.println("writing...");

}

void loop() {
    Serial.println("writing 512 samples of random data");
    for (int j=0;j<512;j++){
        buffer[j] = (rand() % 100) - 50;         
    }
    aac.write((uint8_t*)buffer, 512);
}
