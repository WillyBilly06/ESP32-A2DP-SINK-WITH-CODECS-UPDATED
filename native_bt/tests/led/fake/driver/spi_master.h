#ifndef FAKE_SPI
#define FAKE_SPI
#include <stddef.h>
typedef int spi_host_device_t;typedef void *spi_device_handle_t;
typedef struct { int mosi_io_num,miso_io_num,sclk_io_num,quadwp_io_num,quadhd_io_num,max_transfer_sz; } spi_bus_config_t;
typedef struct { int clock_speed_hz,spics_io_num,flags,queue_size; } spi_device_interface_config_t;
typedef struct { size_t length; const void *tx_buffer; } spi_transaction_t;
#define SPI_DMA_CH_AUTO 3
#define SPI2_HOST 1
#define SPI_DEVICE_TXBIT_LSBFIRST 1
#define ESP_OK 0
int spi_device_transmit(spi_device_handle_t,spi_transaction_t*);int spi_bus_remove_device(spi_device_handle_t);int spi_bus_free(spi_host_device_t);int spi_bus_initialize(spi_host_device_t,const spi_bus_config_t*,int);int spi_bus_add_device(spi_host_device_t,const spi_device_interface_config_t*,spi_device_handle_t*);
#endif
