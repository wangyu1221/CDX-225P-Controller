/**
 * S1L9226X Driver - Minimal implementation for power-on self-test
 */

#include "s1l9226x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "S1L9226X";

static inline void delay_us(uint32_t us) { esp_rom_delay_us(us); }
static inline void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static int init_gpio(s1l9226x_ctx_t *ctx)
{
    /* Configure output pins: MCK, MDATA, MLT */
    gpio_config_t io_out = {
        .pin_bit_mask = (1ULL << ctx->mck_pin) | (1ULL << ctx->mdata_pin) | (1ULL << ctx->mlt_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_out) != ESP_OK) return -1;

    gpio_set_level(ctx->mck_pin, 0);
    gpio_set_level(ctx->mdata_pin, 0);
    gpio_set_level(ctx->mlt_pin, 0);

    /* Configure RESET pin - cautious method to avoid triggering chip reset */
    gpio_set_pull_mode(ctx->reset_pin, GPIO_PULLUP_ONLY);
    delay_ms(1);
    gpio_set_level(ctx->reset_pin, 1);
    if (gpio_set_direction(ctx->reset_pin, GPIO_MODE_OUTPUT) != ESP_OK) return -1;
    gpio_set_level(ctx->reset_pin, 1);

    /* Configure ISTAT input pin */
    gpio_config_t io_in = {
        .pin_bit_mask = (1ULL << ctx->istat_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&io_in) != ESP_OK) return -1;

    return 0;
}

static void send_command(s1l9226x_ctx_t *ctx, uint16_t cmd)
{
    gpio_set_level(ctx->mlt_pin, 0);
    delay_us(5);
    for (int i = 15; i >= 0; i--) {
        gpio_set_level(ctx->mdata_pin, (cmd >> i) & 1);
        delay_us(5);
        gpio_set_level(ctx->mck_pin, 1);
        delay_us(5);
        gpio_set_level(ctx->mck_pin, 0);
        delay_us(5);
    }
    gpio_set_level(ctx->mlt_pin, 1);
    delay_us(5);
    gpio_set_level(ctx->mlt_pin, 0);
    gpio_set_level(ctx->mdata_pin, 0);
}

/* Wait for ISTAT to go low first, then high */
static int wait_istat_low_to_high(s1l9226x_ctx_t *ctx, uint32_t timeout_ms)
{
    /* First wait for low (with timeout) */
    uint32_t t = 0;
    while (gpio_get_level(ctx->istat_pin) == 1 && t < timeout_ms) {
        delay_ms(10);
        t += 10;
    }
    if (gpio_get_level(ctx->istat_pin) == 1) {
        return -1;  /* Never went low */
    }

    /* Now wait for high */
    while (gpio_get_level(ctx->istat_pin) == 0 && t < timeout_ms) {
        delay_ms(10);
        t += 10;
    }
    if (gpio_get_level(ctx->istat_pin) == 0) {
        return -1;  /* Never went high */
    }
    return 0;
}

static void reset_chip(s1l9226x_ctx_t *ctx)
{
    gpio_set_level(ctx->reset_pin, 0);
    delay_ms(S1L9226X_RESET_HOLD_MS);
    gpio_set_level(ctx->reset_pin, 1);
    delay_ms(100);

    send_command(ctx, 0x5088);
    send_command(ctx, 0x51FB);
    send_command(ctx, 0x52C7);
    send_command(ctx, 0x86FF);
    send_command(ctx, 0x87FF);
    send_command(ctx, 0x83BC);
    send_command(ctx, 0x800F);
    send_command(ctx, 0x8100);
    send_command(ctx, CMD_AUTO_SEQ_CANCEL << 8);
}

static int febias_control(s1l9226x_ctx_t *ctx)
{
    send_command(ctx, 0x8780);
    delay_ms(10);
    send_command(ctx, 0x87F0);
    delay_ms(10);
    send_command(ctx, 0x8410);
    if (wait_istat_low_to_high(ctx, 100) != 0) {
        ctx->last_error = ERR_FEBIAS_TIMEOUT;
        return -1;
    }
    return 0;
}

/* Focus Offset Cancel Automatic Control Start
 * $08 + $867 + (200ms wait) + $86F + $842 transfer
 * 100ms wait, ISTAT L -> H = success
 */
static int focus_offset_control(s1l9226x_ctx_t *ctx)
{
    send_command(ctx, 0x0800);  /* $08 */
    send_command(ctx, 0x8670);  /* $867 */
    delay_ms(200);
    send_command(ctx, 0x86F0);  /* $86F */
    send_command(ctx, 0x8420);  /* $842 transfer */

    if (wait_istat_low_to_high(ctx, 100) != 0) {
        ctx->last_error = ERR_FOCUS_OFFSET_TIMEOUT;
        return -1;
    }
    return 0;
}

/* Laser Diode ON - LD ON, P-SUB $8560 Transmission */
static void laser_on(s1l9226x_ctx_t *ctx) { send_command(ctx, 0x8560); delay_ms(50); }

/* Laser OFF - $850 Transmission */
static void laser_off(s1l9226x_ctx_t *ctx) { send_command(ctx, 0x8500); }

/* Focusing Auto-Focusing
 * $47 Transmission (2s maximum)
 * ISTAT L -> H indicates auto-focus sequence complete
 * Then check FOK status: FOK H = disc present, FOK L = no disc
 */
static int auto_focus(s1l9226x_ctx_t *ctx)
{
    /* Cancel any previous auto sequence first */
    send_command(ctx, CMD_AUTO_SEQ_CANCEL << 8);
    delay_ms(50);

    /* Start auto-focus */
    send_command(ctx, CMD_AUTO_FOCUS << 8);  /* $47 */

    /* Wait for ISTAT L->H (auto-focus sequence complete) */
    if (wait_istat_low_to_high(ctx, 2000) != 0) {
        ctx->last_error = ERR_AUTO_FOCUS_TIMEOUT;
        return -1;
    }

    /* Select FOK to ISTAT output - $830 */
    send_command(ctx, 0x8300);
    delay_ms(10);

    /* Check FOK status: FOK H = focus OK = disc present */
    if (gpio_get_level(ctx->istat_pin) == 1) {
        /* FOK is H - Focus OK, disc detected */
        return 0;
    }

    /* FOK is L - Focus failed, no disc */
    ctx->last_error = ERR_NO_DISC;
    return -1;
}

static void spindle_start(s1l9226x_ctx_t *ctx)
{
    send_command(ctx, 0xF000);
    send_command(ctx, 0x9901);
    delay_ms(300);
}

static void spindle_stop(s1l9226x_ctx_t *ctx) { send_command(ctx, 0x9900); }

/* Stop all servos - use hardware reset to force stop */
static void stop_all_servos(s1l9226x_ctx_t *ctx)
{
    ESP_LOGI(TAG, "Stopping all servos via hardware reset...");

    /* Hardware reset forces all servos to stop */
    gpio_set_level(ctx->reset_pin, 0);
    delay_ms(50);
    gpio_set_level(ctx->reset_pin, 1);
    delay_ms(100);

    /* Send stop commands after reset */
    send_command(ctx, CMD_AUTO_SEQ_CANCEL << 8);
    send_command(ctx, 0x8600);  /* Focus servo stop */
    send_command(ctx, 0x2100);  /* Track servo stop */
    send_command(ctx, 0x2500);  /* Sled servo stop */
    send_command(ctx, 0x9900);  /* Spindle stop */

    ESP_LOGI(TAG, "All servos stopped");
}

/* Tracking Offset Cancel Start
 * $8F1F -> $8F00 (ISTAT->H)
 */
static int tracking_offset_control(s1l9226x_ctx_t *ctx)
{
    send_command(ctx, 0x8F1A);
    delay_ms(10);
    if (wait_istat_low_to_high(ctx, 100) != 0) {
        ctx->last_error = ERR_TRACKING_OFFSET_TIMEOUT;
        return -1;
    }
    return 0;
}

int s1l9226x_power_on_self_test(s1l9226x_ctx_t *ctx)
{
    if (ctx->mck_pin == 0) ctx->mck_pin = S1L9226X_PIN_MCK;
    if (ctx->mdata_pin == 0) ctx->mdata_pin = S1L9226X_PIN_MDATA;
    if (ctx->mlt_pin == 0) ctx->mlt_pin = S1L9226X_PIN_MLT;
    if (ctx->reset_pin == 0) ctx->reset_pin = S1L9226X_PIN_RESET;
    if (ctx->istat_pin == 0) ctx->istat_pin = S1L9226X_PIN_ISTAT;

    if (init_gpio(ctx) != 0) {
        ctx->last_error = ERR_GPIO_INIT_FAILED;
        return ERR_GPIO_INIT_FAILED;
    }
    ctx->initialized = true;

    reset_chip(ctx);

    int febias_control_result = febias_control(ctx);
    ESP_LOGI(TAG, "ffebias_control_result: %d", febias_control_result);
    if (febias_control_result == -1){
        ESP_LOGE(TAG, "febias_control FAILED: %d", ctx->last_error);
    }
    
    int focus_offset_control_result = focus_offset_control(ctx);
    ESP_LOGI(TAG, "focus_offset_control_result: %d", focus_offset_control_result);
    if (focus_offset_control_result == -1){
        ESP_LOGE(TAG, "focus_offset_control FAILED: %d", ctx->last_error);
    }

    int tracking_offset_control_result = tracking_offset_control(ctx);
    ESP_LOGI(TAG, "tracking_offset_control_result: %d", tracking_offset_control_result);
    if (tracking_offset_control_result == -1){
        ESP_LOGE(TAG, "tracking_offset_control FAILED: %d", ctx->last_error);
    }

    /* Laser Diode ON */
    laser_on(ctx);
    ESP_LOGI(TAG, "Laser ON");

    /* Auto-Focus with 3 retries */
    int auto_focus_result;
    for (int retry = 0; retry < 3; retry++) {
        ESP_LOGI(TAG, "Auto-focus attempt %d/3", retry + 1);
        auto_focus_result = auto_focus(ctx);
        if (auto_focus_result == 0) {
            ESP_LOGI(TAG, "Auto-focus OK - Disc detected");
            break;
        }
        ESP_LOGW(TAG, "Auto-focus FAILED (attempt %d)", retry + 1);
    }

    if (auto_focus_result != 0) {
        ESP_LOGE(TAG, "Auto-focus FAILED after 3 attempts - No disc");
        laser_off(ctx);
        return ctx->last_error;
    }

    return ctx->last_error;
}