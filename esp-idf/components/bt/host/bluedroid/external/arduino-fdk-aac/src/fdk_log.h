#pragma once

// User Settings: Activate/Deactivate logging
#ifndef FDK_LOGGING_ACTIVE
#  define FDK_LOGGING_ACTIVE true
#endif

#ifndef FDK_LOG_LEVEL
#  define FDK_LOG_LEVEL FDKWarning
#endif

// Logging Implementation
#if FDK_LOGGING_ACTIVE == true

#include <string.h>
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdio.h>
#endif

static const int log_buffer_size = 160;
static char log_buffer[log_buffer_size];
enum LogLevelFDK {FDKDebug, FDKInfo, FDKWarning, FDKError};
extern LogLevelFDK LOGLEVEL_FDK;

static const char* levelName(LogLevelFDK level) {
    switch(level){
        case FDKDebug:
            return "D";
        case FDKInfo:
            return "I";
        case FDKWarning:
            return "W";
        case FDKError:
            return "E";
    }
    return "";
}

static void printLogFDK(const char*msg){
#ifdef ARDUINO
    Serial.print(msg);
#else
    printf("%s",msg);
#endif
}

static void printLogFDK(const char* file, int line, LogLevelFDK current_level) {
    const char* file_name = strrchr(file, '/') ? strrchr(file, '/') + 1 : file;
    const char* level_code = levelName(current_level);
    printLogFDK("[");
    printLogFDK(level_code);
    printLogFDK("] ");
    printLogFDK(file_name);
    printLogFDK(" : ");
    char line_str[20];
    snprintf(line_str,20,"%d",line);
    printLogFDK(line_str);
    printLogFDK(" - ");
    printLogFDK(log_buffer);
    printLogFDK("\n");
}

// We print the log based on the log level
#define LOG_FDK(level,...) { if(level>=LOGLEVEL_FDK) {  snprintf(log_buffer,log_buffer_size, __VA_ARGS__);  printLogFDK(__FILE__,__LINE__, level); } }
#else
// Remove all log statments from the code
#define LOG_FDK(FDKDebug, ...) 
#endif
