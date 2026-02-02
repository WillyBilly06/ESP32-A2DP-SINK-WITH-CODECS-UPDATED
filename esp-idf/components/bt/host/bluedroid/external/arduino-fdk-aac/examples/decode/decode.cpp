#ifdef EMULATOR
#include "Arduino.h"
#include "decode.ino"

int main(){
    setup();
    while(true){
        loop();
    }
    return -1;
}

#endif
