/**
 * S1L9226X RF AMP & Servo Signal Processor Driver
 *
 * MICOM interface: 3-wire serial (MCK, MDATA, MLT)
 * Frame format: 16-bit MSB first, D15..D8 = address, D7..D0 = data
 * MLT falling edge latches data
 *
 * Reference: S1L9226X Preliminary Spec
 */

#ifndef S1L9226X_H
#define S1L9226X_H

#include "gpio_define.h"
#include "driver/gpio.h"

/* ---- GPIO Pin Assignment ---- */
#define S1L9226X_PIN_MCK        J3_CLK1      /* GPIO5  - Micom Clock */
#define S1L9226X_PIN_MDATA      J3_DATA1     /* GPIO18 - Micom Data  */
#define S1L9226X_PIN_MLT        J3_XMLT      /* GPIO19 - Micom Latch */
#define S1L9226X_PIN_RESET      J3_CDPRES    /* GPIO17 - Chip Reset  */
#define S1L9226X_PIN_ISTAT      J3_SENSE     /* GPIO16 - Status Output */

/* ---- Timing Constants ---- */
/* MCK half-period: data sheet specifies MCK high/low width >= 1us typical */
#define S1L9226X_MCK_HALF_US        5       /* half clock period (us) */
#define S1L9226X_MLT_SETUP_US       1       /* MDATA setup before MCK rising (us) */
#define S1L9226X_MLT_HOLD_US        1       /* MDATA hold after MCK rising (us) */

/* Reset timing */
#define S1L9226X_RESET_HOLD_MS      10      /* RESET pulse width (ms) */
#define S1L9226X_RESET_SETTLE_MS    100     /* post-reset settling (ms) */

/* Timeout constants (per p.43 flowchart + datasheet specs) */
#define S1L9226X_FEBIAS_TIMEOUT_MS          200   /* Febias: spec 128ms, margin 200ms */
#define S1L9226X_FOCUS_OFFSET_TIMEOUT_MS    200   /* Focus offset: spec 128ms, margin 200ms */
#define S1L9226X_TRACKING_STEP_MS           2     /* Tracking offset: per-step settle time (ms)
                                                   * 32 steps max = 64ms total sweep time */
#define S1L9226X_AUTO_FOCUS_TIMEOUT_MS       2000  /* Auto-focus: 2s per flowchart */
#define S1L9226X_SERVO_ADJUST_TIMEOUT_MS     300   /* Tracking adjust: 300ms per flowchart */

/* ---- 16-bit Command Helper ---- */
/* Build 16-bit command: upper 8 bits = address, lower 8 bits = data */
#define S1L9226X_CMD(addr, data)  (((uint16_t)(addr) << 8) | ((uint16_t)(data) & 0xFF))

/* ---- Register Addresses ---- */
/* $0X: Focus control (D7-D4 = 0000) */
/* $1X: Tracking control (D7-D4 = 0001) */
/* $2X: Servo mode (D7-D4 = 0010) */
/* $3X: Focus/Sled search level, SSTOP */
/* $4X: Auto-sequence */
/* $5X: Blind/Brake timing */
/* $6X: Kick/Fast/PWM timing */
/* $7X: 2N/M track count, Fast search T */
/* $8X: Automatic control */
/* $9X: CLV control */
/* $CX: Brake point P */
/* $FX: Speed setting */

/* ---- Key Register Map ---- */

/* $50XX - Blind A/B/C timing, Fast K timing */
/* $51XX - Control register: PS3X, PSTZC, ATS, FZCOFF, TRSTS, TZCIC, MCC1, EQR */
/* $52XX - FJTS, PEAKC, FEB5..FEB0 */
/* $80XX - Tracking balance (D4-D0 = B4..B0) */
/* $81XX - Tracking gain (D4-D0 = G4..G0) */
/* $82XX - Photo-diode I/V AMP gain & TE gain */
/* $83XX - ISTAT monitor select, RFO offset, GSEL */
/* $84XX - STGW, STBW, F.S.O.C, F.E.O.C, LDON */
/* $85XX - APC PSUB (laser power) */
/* $86XX - RSTS, EQOC, DFCT1, DFCT2 */
/* $87XX - DIRC, RSTF, AGCL, EQB */
/* $8EXX - Focus/Tracking servo filter control */
/* $8FXX - Tracking servo offset control */
/* $99XX - CLV on/off */
/* $F0XX - Speed setting */

/* ---- Error Codes ---- */
typedef enum {
    S1L9226X_OK = 0,
    S1L9226X_ERR_GPIO,
    S1L9226X_ERR_FEBIAS_TIMEOUT,
    S1L9226X_ERR_FOCUS_OFFSET_TIMEOUT,
    S1L9226X_ERR_TRACKING_OFFSET_TIMEOUT,
    S1L9226X_ERR_AUTO_FOCUS_TIMEOUT,
    S1L9226X_ERR_NO_DISC,
} s1l9226x_err_t;

/* ---- Driver Context ---- */
typedef struct {
    gpio_num_t mck_pin;
    gpio_num_t mdata_pin;
    gpio_num_t mlt_pin;
    gpio_num_t reset_pin;
    gpio_num_t istat_pin;
    bool initialized;
    s1l9226x_err_t last_error;
} s1l9226x_ctx_t;

/* ---- Public API ---- */

/**
 * Full power-on self-test sequence per datasheet p.43 flow chart:
 *   Reset -> Febias control ($878+$87F+$841)
 *         -> Focus offset ($08+$867+200ms+$86F+$842)
 *         -> Tracking offset ($8F1F->$8F00)
 *         -> Laser ON ($8560)
 *         -> Auto-focus ($47, 3 retries)
 *         -> Spindle servo + tracking adjust ($20)
 *
 * Returns S1L9226X_OK on success (disc detected, spindle spinning).
 */
s1l9226x_err_t s1l9226x_power_on_self_test(s1l9226x_ctx_t *ctx);

/**
 * Send a raw 16-bit command to S1L9226X.
 * Format: MSB first, D15..D8 = address, D7..D0 = data.
 */
void s1l9226x_send_cmd(s1l9226x_ctx_t *ctx, uint16_t cmd);

/**
 * Hardware reset the chip and load default register values.
 */
void s1l9226x_reset(s1l9226x_ctx_t *ctx);

#endif /* S1L9226X_H */
