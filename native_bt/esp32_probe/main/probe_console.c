#include "probe_console.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include <stddef.h>
#include <stdio.h>
#if !CONFIG_ESP_CONSOLE_UART || CONFIG_ESP_CONSOLE_UART_NUM != 0
#error This owned ESP32 console requires the existing UART0 console
#endif
esp_err_t probe_console_irq_init(bool *created){
    if(created)*created=false;
    const uart_port_t port=CONFIG_ESP_CONSOLE_UART_NUM;
    if(!uart_is_driver_installed(port)){
        esp_err_t result=uart_driver_install(port,256,0,0,NULL,0);
        if(result!=ESP_OK)return result;
        if(created)*created=true;
    }
    uart_vfs_dev_use_driver(port);
    return ESP_OK;
}

bool probe_console_write_record(char *record,size_t length,size_t capacity){
    if(!length)return true;
    if(!record || length>capacity || length>1023u || record[length-1u]!='\n')return false;
    for(size_t i=0;i+1u<length;++i)if(record[i]=='\n' || record[i]=='\r')return false;
#if CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF
    if(length==capacity)return false;
    record[length-1u]='\r';record[length++]='\n';
#elif CONFIG_LIBC_STDOUT_LINE_ENDING_CR
    record[length-1u]='\r';
#endif
    /* Match the existing record-level stdio serialization. The UART driver
     * mutex also protects the complete write; other stdout/stderr writers
     * cannot insert characters into this record. Background context only. */
    flockfile(stdout);flockfile(stderr);
    bool complete=uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM,record,length)==(int)length;
    funlockfile(stderr);funlockfile(stdout);
    return complete;
}
