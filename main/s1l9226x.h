/**
 * S1L9226X RF AMP & Servo Signal Processor Driver
 * Minimal implementation for power-on self-test
 */

#ifndef S1L9226X_H
#define S1L9226X_H

#include "gpio_define.h"
#include "driver/gpio.h"

/* GPIO Pin Definitions */
#define S1L9226X_PIN_MCK        J3_CLK1
#define S1L9226X_PIN_MDATA      J3_DATA1
#define S1L9226X_PIN_MLT        J3_XMLT
#define S1L9226X_PIN_RESET      J3_CDPRES
#define S1L9226X_PIN_ISTAT      J3_SENSE

/* Timing Constants */
#define S1L9226X_MCK_PERIOD_US  10
#define S1L9226X_RESET_HOLD_MS  10
#define S1L9226X_AUTO_FOCUS_TIMEOUT_MS 2000

/* Key Commands for Self-Test */
#define CMD_AUTO_SEQ_CANCEL     0x40
#define CMD_AUTO_FOCUS          0x47
#define CMD_FOCUS_ON            0x08
#define CMD_TRACK_SERVO_OFF     0x21
#define CMD_SLED_SERVO_OFF      0x25

/* Error Codes */
typedef enum {
    ERR_NONE = 0,
    ERR_GPIO_INIT_FAILED,
    ERR_FEBIAS_TIMEOUT,
    ERR_FOCUS_OFFSET_TIMEOUT,
    ERR_TRACKING_OFFSET_TIMEOUT,
    ERR_AUTO_FOCUS_TIMEOUT,
    ERR_NO_DISC
} error_code_t;

/* Driver Context */
typedef struct {
    gpio_num_t mck_pin;
    gpio_num_t mdata_pin;
    gpio_num_t mlt_pin;
    gpio_num_t reset_pin;
    gpio_num_t istat_pin;
    bool initialized;
    error_code_t last_error;
} s1l9226x_ctx_t;

/**
 * Initialize driver and execute full power-on self-test
 * Returns: 0 on success, error_code on failure
 */
int s1l9226x_power_on_self_test(s1l9226x_ctx_t *ctx);

#endif /* S1L9226X_H */