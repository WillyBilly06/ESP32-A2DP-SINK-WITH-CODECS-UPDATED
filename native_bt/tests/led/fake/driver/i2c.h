#ifndef FAKE_I2C
#define FAKE_I2C
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
typedef int i2c_port_t;
#define I2C_NUM_MAX 2
#define I2C_NUM_1 1
#define I2C_MODE_MASTER 1
#ifndef ESP_OK
#define ESP_OK 0
#endif
#define ESP_FAIL -1
typedef struct {int mode,sda_io_num,scl_io_num,sda_pullup_en,scl_pullup_en;struct {int clk_speed;} master;} i2c_config_t;
int i2c_param_config(i2c_port_t,const i2c_config_t*);int i2c_driver_install(i2c_port_t,int,int,int,int);int i2c_driver_delete(i2c_port_t);
int i2c_master_write_to_device(i2c_port_t,uint8_t,const uint8_t*,size_t,unsigned);
int i2c_master_read_from_device(i2c_port_t,uint8_t,uint8_t*,size_t,unsigned);
#endif
