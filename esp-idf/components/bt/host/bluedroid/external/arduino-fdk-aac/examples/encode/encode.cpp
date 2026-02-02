#ifdef EMULATOR
#include "Arduino.h"
#include "encode.ino"

int main(){
    setup();
    while(true){
        loop();
    }
    return -1;
}

#endif
