#ifndef DBT_INPUT_ESP32_H
#define DBT_INPUT_ESP32_H
#include "dbt_input.h"
#include "driver/i2c.h"
typedef struct {
  dbt_input_t input;
  i2c_port_t port;
  uint8_t address;
  int gpio[3];
  bool installed, pending, retry_fresh;
  dbt_input_sample_t retry;
  atomic_uint read_errors, deferred_samples, feedback_errors, invalid_channels;
  uint8_t feedback, feedback_applied, previous_buttons;
} dbt_input_esp32_t;
typedef enum {
  DBT_INPUT_START_READY,
  /* Initial identity-register write was NACKed and driver delete succeeded.
   * This does not distinguish a missing device from wiring/power problems. */
  DBT_INPUT_START_NO_ACK,
  DBT_INPUT_START_FAILED,
  DBT_INPUT_START_CLEANUP_FAILED
} dbt_input_start_result_t;
/* Call only from an input task, never the host. Usual pins SDA23/SCL22,
 * button GPIO18/21/19, I2C port 1, address 0x07; board variants may differ.
 * All GPIOs must be distinct and valid. There is no reset or device scan. */
bool dbt_input_esp32_start(dbt_input_esp32_t *p, i2c_port_t port, int sda,
                           int scl, uint8_t address, int button1, int button2,
                           int effect);
dbt_input_start_result_t dbt_input_esp32_start_checked(dbt_input_esp32_t *p,i2c_port_t port,
    int sda,int scl,uint8_t address,int button1,int button2,int effect);
/* At ~25ms cadence in an input task. Failed queue admission retains the exact
 * sample for retry before further I2C reads. Owner drains dbt_input_next. */
bool dbt_input_esp32_poll(dbt_input_esp32_t *p);
typedef enum {DBT_INPUT_DRAIN_DONE,DBT_INPUT_DRAIN_BUSY,DBT_INPUT_DRAIN_FAULT} dbt_input_drain_result_t;
/* Producer only. Retries an already-copied sample without any new sensor
 * reads. BUSY preserves the sample; FAULT is not a successful drain. */
dbt_input_drain_result_t dbt_input_esp32_drain_pending(dbt_input_esp32_t *p);
void dbt_input_esp32_stop(dbt_input_esp32_t *p);
/* False leaves ownership recorded and bars another start on that port. */
bool dbt_input_esp32_stop_checked(dbt_input_esp32_t *p);
#endif
