#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
typedef int esp_err_t;
typedef void *i2s_chan_handle_t;
typedef struct {void *dma_buf;size_t size;} i2s_event_data_t;
typedef struct {int dma_desc_num,dma_frame_num;bool auto_clear_after_cb,auto_clear_before_cb;int intr_priority;} i2s_chan_config_t;
typedef struct {int clk_src;} i2s_clk_config_t;
typedef struct {int dummy;} i2s_slot_config_t;
typedef struct {int mclk,bclk,ws,dout,din;} i2s_gpio_config_t;
typedef struct {i2s_clk_config_t clk_cfg;i2s_slot_config_t slot_cfg;i2s_gpio_config_t gpio_cfg;} i2s_std_config_t;
typedef struct {bool (*on_sent)(i2s_chan_handle_t,i2s_event_data_t *,void *);} i2s_event_callbacks_t;
#define ESP_OK 0
#define I2S_NUM_1 1
#define I2S_ROLE_MASTER 1
#define I2S_GPIO_UNUSED -1
#define I2S_DATA_BIT_WIDTH_32BIT 32
#define I2S_SLOT_MODE_STEREO 2
#define I2S_CLK_SRC_APLL 1
#define I2S_CHANNEL_DEFAULT_CONFIG(n,r) ((i2s_chan_config_t){0})
#define I2S_STD_CLK_DEFAULT_CONFIG(r) ((i2s_clk_config_t){0})
#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(w,m) ((i2s_slot_config_t){0})
esp_err_t i2s_new_channel(const i2s_chan_config_t *,i2s_chan_handle_t *,void *);
esp_err_t i2s_del_channel(i2s_chan_handle_t);
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t,const i2s_std_config_t *);
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t,const i2s_event_callbacks_t *,void *);
esp_err_t i2s_channel_disable(i2s_chan_handle_t);
esp_err_t i2s_channel_enable(i2s_chan_handle_t);
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t,const void *,size_t,size_t *);
