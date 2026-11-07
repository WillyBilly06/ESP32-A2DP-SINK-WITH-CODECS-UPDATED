#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
typedef int uart_port_t;
bool uart_is_driver_installed(uart_port_t);
esp_err_t uart_driver_install(uart_port_t,int,int,int,void *,int);
int uart_write_bytes(uart_port_t,const void *,size_t);
