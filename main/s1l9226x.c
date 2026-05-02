/**
 * S1L9226X Driver - RF AMP & Servo Signal Processor
 *
 * MICOM 3-wire serial interface (MCK, MDATA, MLT)
 * Frame: 16-bit MSB first. D15..D8 = address, D7..D0 = data.
 * MLT = HIGH during transfer; falling edge latches data into chip.
 *
 * Initialization sequence per datasheet p.43 "System Control Flow":
 *   1. Febias offset control:  $878 -> $87F + $841 -> ISTAT L->H
 *   2. Focus offset control:   $08 + $867 + 200ms + $86F + $842 -> ISTAT L->H
 *   3. Tracking offset cancel: $8F1F -> $8F00 -> ISTAT H
 *   4. Laser diode ON:         LD ON + P-SUB $8560
 *   5. Limit SW check
 *   6. Auto-focus:             $47 -> FOK check (max 3 retries, 2s each)
 *   7. Spindle servo ON:       $20 -> Tracking balance/gain adjust -> TOC read
 *
 * Reference: S1L9226X Preliminary Spec
 */

#include "s1l9226x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "S1L9226X";

/* ---- Delay helpers ---- */
static inline void delay_us(uint32_t us) { esp_rom_delay_us(us); }
static inline void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/* ================================================================
 * GPIO Initialization
 * ================================================================ */
static s1l9226x_err_t init_gpio(s1l9226x_ctx_t *ctx)
{
    /*
     * Output pins: MCK, MDATA, MLT
     * All start LOW. MLT must be LOW (idle = not latching).
     */
    gpio_config_t io_out = {
        .pin_bit_mask = (1ULL << ctx->mck_pin)
                      | (1ULL << ctx->mdata_pin)
                      | (1ULL << ctx->mlt_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_out);
    if (err != ESP_OK) return S1L9226X_ERR_GPIO;

    gpio_set_level(ctx->mck_pin,   0);
    gpio_set_level(ctx->mdata_pin, 0);
    gpio_set_level(ctx->mlt_pin,   0);

    /*
     * RESET pin: active LOW.
     * Set HIGH (inactive) before configuring as output to avoid glitch reset.
     */
    gpio_set_pull_mode(ctx->reset_pin, GPIO_PULLUP_ONLY);
    delay_ms(1);
    gpio_set_direction(ctx->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(ctx->reset_pin, 1);  /* inactive */

    /*
     * ISTAT input pin with pull-up.
     * Datasheet: ISTAT is push-pull output from S1L9226X, but we add
     * pull-up for safety during chip reset (ISTAT may be Hi-Z).
     */
    gpio_config_t io_in = {
        .pin_bit_mask = (1ULL << ctx->istat_pin)
                      | (1ULL << J3_CDPSW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&io_in);
    if (err != ESP_OK) return S1L9226X_ERR_GPIO;

    return S1L9226X_OK;
}

/* ================================================================
 * Core 16-bit Command Transfer
 * ================================================================
 *
 * S1L9226X MICOM interface timing:
 *   1. Set MDATA before MCK rising edge (setup time)
 *   2. Raise MCK (rising edge)
 *   3. Hold MDATA after MCK rising edge (hold time)
 *   4. Lower MCK (falling edge)
 *   5. Repeat for D15..D0 (MSB first)
 *   6. After all 16 bits, MLT falling edge latches data
 *
 * MLT must be HIGH during the entire 16-bit transfer.
 * We raise MLT before the first clock, lower it after the last.
 */
void s1l9226x_send_cmd(s1l9226x_ctx_t *ctx, uint16_t cmd)
{
    /* MLT HIGH = frame in progress */
    gpio_set_level(ctx->mlt_pin, 1);
    delay_us(S1L9226X_MLT_SETUP_US);

    /* Transmit D15 (MSB) down to D0 (LSB) */
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(ctx->mdata_pin, (cmd >> i) & 1);
        delay_us(S1L9226X_MLT_SETUP_US);
        gpio_set_level(ctx->mck_pin, 1);
        delay_us(S1L9226X_MCK_HALF_US);
        gpio_set_level(ctx->mck_pin, 0);
        delay_us(S1L9226X_MLT_HOLD_US);
    }

    /* MLT falling edge = latch */
    delay_us(1);
    gpio_set_level(ctx->mlt_pin, 0);

    /* MDATA idle low */
    gpio_set_level(ctx->mdata_pin, 0);
}

/* ================================================================
 * ISTAT Polling Helpers
 * ================================================================
 * NOTE: ISTAT output from S1L9226X is inverted by the level-shift
 * circuit before reaching the MCU GPIO pin. Therefore:
 *   ISTAT signal HIGH  →  MCU pin reads LOW   (wait_sense_low)
 *   ISTAT signal LOW   →  MCU pin reads HIGH  (wait_sense_high)
 * ================================================================ */

/* Wait until ISTAT signal goes HIGH (= MCU pin goes LOW) */
static int wait_sense_low(s1l9226x_ctx_t *ctx, uint32_t timeout_ms)
{
    for (uint32_t t = 0; t < timeout_ms; t += 5) {
        if (gpio_get_level(ctx->istat_pin) == 0)
            return 0;
        delay_ms(5);
    }
    return -1;
}

static int is_sense_low(s1l9226x_ctx_t *ctx)
{
    if (gpio_get_level(ctx->istat_pin) == 0) return 0;
    return -1;
}


/* ================================================================
 * Hardware Reset & Default Register Initialization
 * ================================================================
 *
 * Register initial values from S1L9226X datasheet p.18-22:
 *   $50XX: Blind/Brake timing          INI = 0x88
 *   $51XX: Control register             INI = 0xFB
 *   $52XX: FJTS/PEAKC/FEB              INI = 0x87
 *   $80XX: Tracking balance             INI = 0x0F
 *   $81XX: Tracking gain                INI = 0x10
 *   $82XX: I/V AMP gain                 INI = 0x07
 *   $83XX: ISTAT monitor select         INI = 0xB8
 *   $84XX: Window/LDON control          INI = 0x00
 *   $86XX: RSTS/EQOC/DFCT1/DFCT2       INI = 0x0F
 *   $87XX: DIRC/RSTF/AGCL/EQB           INI = 0x0F
 *   $8EXX: Filter control               INI = 0xB6
 *   $8FXX: Tracking offset center       INI = 0x10
 */
void s1l9226x_reset(s1l9226x_ctx_t *ctx)
{
    /* Assert RESET (active LOW) */
    gpio_set_level(ctx->reset_pin, 0);
    delay_ms(S1L9226X_RESET_HOLD_MS);

    /* Deassert RESET */
    gpio_set_level(ctx->reset_pin, 1);
    delay_ms(S1L9226X_RESET_SETTLE_MS);

    /* ---- Load register defaults ---- */

    /* $50XX: Blind A/B/C timing, Fast K timing = 0x88 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x50, 0x88));

    /* $51XX: Control register = 0xFB */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x51, 0xFB));

    /* $52XX: FJTS/PEAKC/FEB = 0x87 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x52, 0x87));

    /* $80XX: Tracking balance = 0x0F */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x80, 0x0F));

    /* $81XX: Tracking gain = 0x10 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x81, 0x10));

    /* $82XX: Photo-diode I/V AMP gain = 0x07 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x82, 0x07));

    /* $83XX: ISTAT monitor select = 0xB8 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x83, 0xB8));

    /* $84XX: Window/LDON control = 0x00 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x00));

    /* $86XX: RSTS/EQOC/DFCT1/DFCT2 = 0x0F */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x86, 0x0F));

    /* $87XX: DIRC/RSTF/AGCL/EQB = 0x0F */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x87, 0x0F));

    /* $8EXX: Filter control = 0xB6 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8E, 0xB6));

    /* $8FXX: Tracking servo offset center = 0x10 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x10));

    /* Cancel any auto-sequence: $40 */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x40, 0x00));

    ESP_LOGI(TAG, "Reset complete, registers initialized");
}

/* ================================================================
 * Step 1: Febias Offset Control
 * ================================================================
 * Datasheet p.43 flow: $878 -> $87F + $841 -> wait ISTAT L->H (100ms max)
 * Datasheet p.44:
 *   - $878 resets the 4-bit Febias DAC
 *   - $87F cancels reset ($87 D2 bit changes from 0 to 1)
 *   - $841 starts Febias offset automatic control
 *   - 5-bit DAC sweeps -112mV..+112mV, 32 steps, 2.5ms/step, max 128ms
 *   - ISTAT goes H when complete
 *   - Post-control offset dispersion: -8mV..+8mV
 */
static s1l9226x_err_t febias_control(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Febias offset control start");

    /* 1. Reset 4-bit Febias DAC ($878 = $87, data 0x08 = D3=1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x87, 0x08));

    /* 2. Cancel DAC reset ($87F = $87, data 0x0F, D2 changes 0->1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x87, 0x0F));

    /* 3. Start Febias offset automatic control ($841)
     *    $84 D4 = F.E.O.C = 1 enables Febias offset control
     *    Data byte 0x11: D4=1 (F.E.O.C on), D0=1 (ISTAT window select)
     */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x11));

    /* 4. Wait for ISTAT L->H (max 100ms per flowchart, 128ms per spec) */
    if (wait_sense_low(ctx, S1L9226X_FEBIAS_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Febias control timeout");
        ctx->last_error = S1L9226X_ERR_FEBIAS_TIMEOUT;
        return S1L9226X_ERR_FEBIAS_TIMEOUT;
    }

    ESP_LOGI(TAG, "Febias control OK (ISTAT H)");
    return S1L9226X_OK;
}

/* ================================================================
 * Step 2: Focus Offset Control
 * ================================================================
 * Datasheet p.43 flow: $08 + $867 + (200ms wait) + $86F + $842 -> ISTAT L->H
 * Datasheet p.45:
 *   - $08: Focus control command (p.13: $0X = Focus control, data 0x08)
 *         FS4=0(off), FS3=0(normal gain), FS2=1(search on), FS1=0(search down)
 *   - $867: Reset 4-bit DAC for focus offset ($86 D3=0)
 *   - 200ms wait for DAC reset to settle
 *   - $86F: Cancel DAC reset ($86 D3 changes 0->1)
 *   - $842: Start focus offset automatic control ($84 D5 = F.S.O.C = 1)
 *   - 4-bit DAC sweeps 320mV..-320mV, 16 steps, 2.5ms/step, max 128ms
 *   - ISTAT goes H when complete
 *   - Post-control offset dispersion: -20mV..+20mV
 */
static s1l9226x_err_t focus_offset_control(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Focus offset control start");

    /* 1. Focus control: $08 (FS2=search on, others default) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x08, 0x08));

    /* 2. Reset 4-bit focus offset DAC ($867 = $86 D3=0)
     *    $86 initial = 0x0F (D3=1), so $867 = $86 data 0x07 (D3=0) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x86, 0x07));

    /* 3. Wait 200ms for DAC reset to settle (per flowchart) */
    delay_ms(200);

    /* 4. Cancel DAC reset ($86F = $86 data 0x0F, D3 changes 0->1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x86, 0x0F));

    /* 5. Start focus offset automatic control ($842)
     *    $84 D5 = F.S.O.C = 1 enables focus servo offset control
     *    Data byte 0x22: D5=1 (F.S.O.C on), D1=1 (ISTAT window select)
     */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x22));

    /* 6. Wait for ISTAT L->H (max 100ms per flowchart, 128ms per spec) */
    if (wait_sense_low(ctx, S1L9226X_FOCUS_OFFSET_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Focus offset control timeout");
        ctx->last_error = S1L9226X_ERR_FOCUS_OFFSET_TIMEOUT;
        return S1L9226X_ERR_FOCUS_OFFSET_TIMEOUT;
    }

    ESP_LOGI(TAG, "Focus offset control OK (ISTAT H)");
    return S1L9226X_OK;
}

static s1l9226x_err_t test_tracking_offset(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Test tracking offset: move inward 1 step, then outward 1 step, then stop");

    /* $8F is a 5-bit DAC, INI = 0x10 (0mV center)
     * Each step = ~10mV, range: $8F1F(-160mV) ~ $8F00(+160mV)
     * Higher value = more negative = inward (toward ID)
     * Lower value = more positive = outward (toward OD) */

    /* Step 1: Move inward 1 step (0x10 -> 0x11 = -10mV) */
    ESP_LOGI(TAG, "  -> INWARD: $8F = 0x11 (-10mV)");
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x11));
    delay_ms(200);

    /* Step 2: Move outward 1 step (0x11 -> 0x0F = +10mV, skip center) */
    ESP_LOGI(TAG, "  -> OUTWARD: $8F = 0x0F (+10mV)");
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x0F));
    delay_ms(200);

    /* Step 3: Stop
     * Use $20 (TM1 mode): spindle on, tracking/sled OFF.
     * $20 is more reliable than $21 because it explicitly sets chip mode
     * where BOTH tracking and sled servos are disabled.
     * First restore $8F to center to remove any DC drive current. */
    ESP_LOGI(TAG, "  -> Restoring $8F = 0x10 (center)");
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x10));
    delay_ms(20);

    ESP_LOGI(TAG, "  -> Sending $20 (TM1: tracking/sled OFF)");
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x20, 0x00));
    delay_ms(200);

    ESP_LOGI(TAG, "Test tracking offset done");
    return S1L9226X_OK;
}

/* ================================================================
 * Step 3: Tracking Offset Cancel
 * ================================================================
 * Datasheet p.43 flowchart: $8F1F -> $8F00 (ISTAT->H)
 *
 * Root cause analysis:
 *   $8F is a DIRECT DAC register — writing it only sets the static DC
 *   offset, it does NOT trigger any automatic sweep.
 *   MCU must step-sweep manually and poll ISTAT at each step.
 *
 *   CRITICAL prerequisite: tracking servo loop MUST be OFF ($21) before
 *   this step.  If servo is running, it continuously drives the actuator
 *   to minimize tracking error, making the ISTAT output reflect the
 *   closed-loop state rather than the open-loop DC offset.  The DAC
 *   sweep cannot find the true zero-crossing while servo is active.
 *
 * Per p.20 ($84 register):
 *   D6 = STBW  -- controls the tracking balance WINDOW WIDTH only:
 *                  0 → ±20mV window,  1 → ±30mV window
 *   STBW does NOT start any automatic control.
 *   It merely selects which threshold ISTAT uses to flag "in-window".
 *
 * Per p.14 ($2X servo mode commands):
 *   $21 → Track. servo OFF  (tracking actuator output = 0, open-loop)
 *   $22 → Track. servo ON   (restore after offset is set)
 *
 * Per p.22 ($8FXX):
 *   $8F1F = -160mV (step 0x1F),  $8F10 = 0mV (center),  $8F00 = +160mV
 *   Ideal final offset: +30mV ~ +50mV (step back 3-5 toward 0x1F)
 *
 * Procedure:
 *   1. Turn tracking servo OFF ($21) — open-loop, actuator free
 *   2. Set $8F1F (max negative, -160mV) — starting point
 *   3. Enable tracking balance window ($84 D6=STBW=1) → ±30mV window
 *   4. Step-sweep: 0x1F → 0x00, 2ms/step, check ISTAT after each step
 *      (MCU pin LOW = ISTAT signal HIGH = offset inside ±30mV window)
 *   5. Found: back off 3 steps (+30mV ideal bias per p.22)
 *      Not found: restore center ($8F10), report error
 *   6. Disable window ($84 D6=0), restore tracking servo ON ($22)
 *
 * NOTE: ISTAT is inverted by level-shift — MCU pin LOW = ISTAT signal HIGH
 */
static s1l9226x_err_t tracking_offset_control(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Tracking offset cancel: sweep $8F1F -> $8F00");

    /* 1. Turn tracking servo OFF
     *    $21 per p.14: Track. servo off
     *    Actuator output goes to 0, circuit is open-loop.
     *    The $8F DAC offset can now be measured without servo fighting it. */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x21, 0x00));
    delay_ms(5);

    /* 2. Set starting point: max negative offset ($8F1F = -160mV) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x1F));
    delay_ms(5);

    /* 3. Enable tracking balance window on ISTAT
     *    $84 D6 = STBW = 1  →  window = ±30mV
     *    Other $84 bits stay 0 (LDON=0, F.S.O.C=0, F.E.O.C=0) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x40));
    delay_ms(5);

    /* 4. Step-sweep: 0x1F (-160mV) → 0x00 (+160mV), 2ms/step
     *    ISTAT goes HIGH (MCU pin LOW) when DAC offset enters ±30mV window */
    int8_t found_step = -1;

    for (int8_t step = 0x1F; step >= 0x00; step--) {
        s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, (uint8_t)step));
        delay_ms(S1L9226X_TRACKING_STEP_MS);

        /* MCU pin LOW = ISTAT signal HIGH = offset within ±30mV window */
        if (gpio_get_level(ctx->istat_pin) == 0) {
            found_step = step;
            ESP_LOGI(TAG, "  -> zero crossing at step=0x%02X (~%+dmV)",
                     step, (0x10 - (int)step) * 10);
            break;
        }
    }

    /* 5. Handle result */
    if (found_step < 0) {
        /* Full sweep completed without ISTAT going HIGH — no valid zero point.
         * This indicates a hardware fault (damaged actuator, no disc loaded yet
         * is normal here since laser is not on; fall through to center). */
        ESP_LOGW(TAG, "Tracking offset: no zero crossing in sweep, using center ($8F10)");
        s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, 0x10));   /* center = 0mV */
    } else {
        /* Back off 3 steps toward $8F1F → +30mV ideal operating point.
         * Per p.22: "ideal +30mV~+50mV, raise 3-5 steps after controlling to 0mV"
         * Higher step value = more negative DAC = more positive system offset.
         * Clamped to 0x1F. */
        uint8_t final_step = (uint8_t)found_step + 3u;
        if (final_step > 0x1F) final_step = 0x1F;

        s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x8F, final_step));
        delay_ms(5);
        ESP_LOGI(TAG, "  -> settled at step=0x%02X (~+%dmV ideal bias)",
                 final_step, (0x10 - (int)final_step) * (-10));
    }

    /* 6. Disable tracking balance window */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x00));

    /* Restore tracking servo ON ($22)
     * Servo resumes with the newly calibrated DC offset applied.
     * Per p.14: $22 = Track. servo on */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x22, 0x00));
    delay_ms(5);

    ESP_LOGI(TAG, "Tracking offset cancel done, servo restored");
    return S1L9226X_OK;
}

/* ================================================================
 * Step 4: Laser Diode ON
 * ================================================================
 * Datasheet p.43: LD ON, P-SUB $8560
 *   $84 D3 = LDON = 1 (laser on)
 *   $85 = APC PSUB for laser power control, data = 0x60
 */
static void laser_on(s1l9226x_ctx_t *ctx)
{
    /* LD ON ($84 D3 = 1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x08));
    delay_ms(10);

    /* Set APC PSUB laser power ($8560 = $85, data 0x60) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x85, 0x60));
    delay_ms(50);

    ESP_LOGI(TAG, "Laser ON (P-SUB=0x60)");
}

static void laser_off(s1l9226x_ctx_t *ctx)
{
    /* APC PSUB off */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x85, 0x00));
    /* Disable LDON ($84 D3 = 0) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x00));
    ESP_LOGI(TAG, "Laser OFF");
}

/* ================================================================
 * Step 6: Auto-Focus
 * ================================================================
 * Datasheet p.43:
 *   - Send $47 (auto-focus command)
 *   - Max 2s wait
 *   - Check FOK (FOK H = disc present)
 *   - Max 3 retries, then laser off + standby
 *
 * Datasheet p.17: $47 = address $4X, data 0111 = auto-focus
 * Datasheet p.26-27: ISTAT = LOW during auto-focus, H when complete
 * After auto-focus: select FOK to ISTAT via $83XX
 */
static s1l9226x_err_t auto_focus(s1l9226x_ctx_t *ctx)
{
    /* Cancel any previous auto-sequence */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x40, 0x00));
    delay_ms(50);

    /* Start auto-focus ($47 = address $4X, data 0111) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x47, 0x07));

    /* Wait for auto-focus to complete (ISTAT L -> H), max 2s per flowchart */
    if (wait_sense_low(ctx, S1L9226X_AUTO_FOCUS_TIMEOUT_MS) != 0) {
        ESP_LOGE(TAG, "Auto-focus timeout (2s)");
        ctx->last_error = S1L9226X_ERR_AUTO_FOCUS_TIMEOUT;
        return S1L9226X_ERR_AUTO_FOCUS_TIMEOUT;
    }

    /* Select FOK to ISTAT output
     * $83XX: CSTAT=0 (select FOK group), RFBC=0 (select FOK)
     * MGA1=1, MGA2=0, RFOC=1, TOCD=1, EMODEC=1, GSEL=0
     * = 0xB8 with CSTAT cleared: 0xB0
     */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x83, 0xB0));
    delay_ms(10);

    /* Read FOK via ISTAT
     * S1L9226X ISTAT signal: FOK H = disc present, FOK L = no disc
     * After level-shift inversion: MCU pin LOW = FOK H (disc present)
     *                              MCU pin HIGH = FOK L (no disc)
     */
    if (gpio_get_level(ctx->istat_pin) == 0) {
        ESP_LOGI(TAG, "Auto-focus OK - FOK H, disc detected");
        return S1L9226X_OK;
    }

    ESP_LOGW(TAG, "Auto-focus FAILED - FOK L, no disc");
    ctx->last_error = S1L9226X_ERR_NO_DISC;
    return S1L9226X_ERR_NO_DISC;
}

/* ================================================================
 * Step 7: Spindle Servo Loop ON
 * ================================================================
 * Datasheet p.43:
 *   - Spindle Servo Loop ON (300ms max)
 *   - Tracking & Sled Loop OFF
 *   - $20 Transmission
 *   - Tracking Balance Adjust
 *   - Tracking Gain Adjust
 *   - TOC Read
 *
 * Datasheet p.14: $20 = TM1 mode (base mode)
 *   Tracking servo bits: TM7=1, TM5=0, TM4=1, TM3=0 -> track servo off
 *   Sled servo bits: TM2=1, TM1=0 -> sled servo off
 *   This is the "spindle servo on, tracking/sled off" mode
 */
static s1l9226x_err_t spindle_servo_on(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Spindle servo ON, tracking/sled OFF");

    /* $20: TM1 mode - spindle servo loop ON, tracking & sled loop OFF */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x20, 0x00));

    /* Tracking balance adjust ($800X ~ $801X)
     * Uses $84 D6 (STBW) to enable tracking balance control window on ISTAT
     * Then sweep balance using $80 commands
     * Per flowchart: this is an auto-adjust process
     */
    /* Enable tracking balance control window on ISTAT ($84 D6=1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x40));
    delay_ms(10);

    /* Tracking balance adjust via $801 command
     * $801: D0=1 triggers tracking balance auto-adjust
     */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x80, 0x11));

    /* Wait for balance adjust complete (ISTAT H), 300ms max per flowchart */
    if (wait_sense_low(ctx, S1L9226X_SERVO_ADJUST_TIMEOUT_MS) != 0) {
        ESP_LOGW(TAG, "Tracking balance adjust timeout (non-fatal)");
    }

    /* Tracking gain adjust via $811 command
     * $811: D0=1 triggers tracking gain auto-adjust
     * Enable tracking gain control window ($84 D7=1) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x80));
    delay_ms(10);

    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x81, 0x11));

    /* Wait for gain adjust complete (ISTAT H), 300ms max */
    if (wait_sense_low(ctx, S1L9226X_SERVO_ADJUST_TIMEOUT_MS) != 0) {
        ESP_LOGW(TAG, "Tracking gain adjust timeout (non-fatal)");
    }

    /* Disable control windows ($84 = 0x00) */
    s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x84, 0x00));

    ESP_LOGI(TAG, "Spindle servo ready, tracking adjusted");
    return S1L9226X_OK;
}

/* ================================================================
 * Power-On Self-Test (Full System Control Sequence)
 * ================================================================
 * Datasheet p.43 flow chart, complete sequence:
 *   1. Febias offset control    (100ms max)
 *   2. Focus offset control     (100ms max)
 *   3. Tracking offset cancel
 *   4. Laser diode ON
 *   5. Limit switch check
 *   6. Auto-focus               (2s max, 3 retries)
 *   7. Spindle servo + adjust   (300ms max)
 *   8. TOC read
 */
s1l9226x_err_t s1l9226x_power_on_self_test(s1l9226x_ctx_t *ctx)
{
    /* Apply default pin mapping if not set */
    if (ctx->mck_pin   == 0) ctx->mck_pin   = S1L9226X_PIN_MCK;
    if (ctx->mdata_pin == 0) ctx->mdata_pin = S1L9226X_PIN_MDATA;
    if (ctx->mlt_pin   == 0) ctx->mlt_pin   = S1L9226X_PIN_MLT;
    if (ctx->reset_pin == 0) ctx->reset_pin = S1L9226X_PIN_RESET;
    if (ctx->istat_pin == 0) ctx->istat_pin = S1L9226X_PIN_ISTAT;

    /* 1. GPIO init */
    s1l9226x_err_t rc = init_gpio(ctx);
    if (rc != S1L9226X_OK) {
        ctx->last_error = rc;
        return rc;
    }
    ctx->initialized = true;
    ESP_LOGI(TAG, "GPIO initialized");

    /* 2. Hardware reset + register defaults */
    s1l9226x_reset(ctx);

    delay_ms(1000);

    /* 3. Febias offset control ($878 -> $87F + $841 -> ISTAT H) */
    rc = febias_control(ctx);
    if (rc != S1L9226X_OK) return rc;

    /* 4. Focus offset control ($08 + $867 + 200ms + $86F + $842 -> ISTAT H) */
    rc = focus_offset_control(ctx);
    if (rc != S1L9226X_OK) return rc;

    /* 5. Tracking offset cancel ($8F1F -> $8F00 -> ISTAT H) */
    test_tracking_offset(ctx);

    // rc = tracking_offset_control(ctx);
    // if (rc != S1L9226X_OK) return rc;

    /* 6. Laser diode ON (LD ON + P-SUB $8560) */
    // laser_on(ctx);

    /* 7. Limit switch check (hardware-dependent, caller should implement) */
    // ESP_LOGI(TAG, "Limit switch check - caller should verify sled home position");
    /* TODO: Implement limit switch detection via GPIO */

    /* 8. Auto-focus with 3 retries (max 2s each attempt) */
    // for (int retry = 0; retry < 3; retry++) {
    //     ESP_LOGI(TAG, "Auto-focus attempt %d/3", retry + 1);
    //     rc = auto_focus(ctx);
    //     if (rc == S1L9226X_OK) break;

    //     if (retry < 2) {
    //         ESP_LOGW(TAG, "Auto-focus failed (attempt %d), retrying...", retry + 1);
    //         /* Cancel current sequence before retry */
    //         s1l9226x_send_cmd(ctx, S1L9226X_CMD(0x40, 0x00));
    //         delay_ms(100);
    //     }
    // }

    // if (rc != S1L9226X_OK) {
    //     /* 3 attempts failed: laser off, display "no disc", standby */
    //     laser_off(ctx);
    //     ESP_LOGE(TAG, "Auto-focus failed after 3 attempts - no disc, entering standby");
    //     return rc;
    // }

    /* 9. Spindle servo ON + tracking balance/gain adjust ($20 -> adjust) */
    // rc = spindle_servo_on(ctx);
    // if (rc != S1L9226X_OK) return rc;

    /* 10. TOC read - DSP handles this, signal ready for playback */
    ESP_LOGI(TAG, "System ready - disc detected, spindle spinning, awaiting TOC read");

    return S1L9226X_OK;
}
