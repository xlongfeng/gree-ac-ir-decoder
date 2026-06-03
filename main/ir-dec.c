/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * GREE AC IR Protocol (YAPOF20 / YAPOF3 family)
 *
 * Each button press transmits TWO consecutive frames with identical wire format.
 * Both frames use 70 RMT symbols (bits sent LSB-first):
 *
 *   [HEADER]   9000 µs mark  + 4500 µs space
 *   [BLOCK 1]  32 bits (bytes 0-3)
 *   [FOOTER]    3 bits = 0b010 (bit0=0, bit1=1, bit2=0)
 *   [GAP]       620 µs mark  + ~20 ms space
 *   [BLOCK 2]  32 bits (bytes 4-7)
 *   [EOF]       620 µs mark  + ~40 ms space  (end-of-frame, triggers RMT timeout)
 *
 *   Bit 1: 620 µs mark + 1600 µs space
 *   Bit 0: 620 µs mark +  540 µs space
 *
 * Frame 1 byte map (byte[3] upper nibble = 0x5):
 *   [0]  Mode[2:0], Power[3], BasicFan[5:4], SwingAuto[6], Sleep[7]
 *        BasicFan: 0=AUTO, 1=Quiet/Lvl1, 2=Lvl2, 3=Turbo/Lvl3
 *        When Fan (frame 2) > Lvl3, BasicFan must be set to Lvl3.
 *   [1]  Temp[3:0], TimerHalfHr[4], TimerTensHr[6:5], TimerEnabled[7]
 *   [2]  TimerHours[3:0], Turbo[4], Light[5], ModelA[6], XFan[7]
 *   [3]  reserved[1:0], TempExtraF[2], UseFahrenheit[3], fixed=0x5[7:4]
 *   [4]  SwingV[3:0], SwingH[6:4], reserved[7]
 *   [5]  DisplayTemp[1:0], IFeel[2], fixed=0x4[5:3], WiFi[6], reserved[7]
 *   [6]  reserved (0x00)
 *   [7]  reserved[1:0], Econo[2], reserved[3], Checksum1[7:4]
 *
 * Frame 2 byte map (byte[3] upper nibble = 0x7):
 *   [0]    Repeat of frame-1 byte 0 (Mode, Power, BasicFan, SwingAuto, Sleep)
 *   [1]    Repeat of frame-1 byte 1 (Temp, Timer)
 *   [2]    Repeat of frame-1 byte 2 (TimerHours, Turbo, Light, ModelA, XFan)
 *   [3]    reserved[3:0], fixed=0b11[5:4], fixed=0b01[7:6]
 *   [4]    Sleep2[0], Sleep3[6:1], Quiet[7]  — when set, Fan must be Lvl1
 *   [5]    Sleep3 related (0x00)
 *   [6]    Sleep3[3:0], Fan[6:4], reserved[7]
 *          Fan: 0=AUTO, 1=Quiet/Lvl1, 2=Lvl2, 3=Lvl3, 4=Lvl4, 5=Turbo/Lvl5
 *   [7]    reserved[2:0], CoolingSensation[3], Checksum2[7:4]
 *
 * Field constraints:
 *   Quiet=1  → Fan = Lvl1 (both BasicFan in F1 and Fan in F2)
 *   Turbo=1  → Fan = Lvl5 (both BasicFan=3 in F1 and Fan=5 in F2)
 *
 * Checksum (both frames, top nibble of byte 7):
 *   sum = 10 + low_nibble(b0..b3) + high_nibble(b4..b6), then & 0x0F
 */

#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define IR_RX_GPIO CONFIG_IR_RX_GPIO
#define IR_RESOLUTION_HZ 1000000 // 1 MHz → 1 tick = 1 µs

/* GREE timing (µs) */
#define GREE_HDR_MARK_US 9000
#define GREE_HDR_SPACE_US 4500
#define GREE_BIT_MARK_US 620
#define GREE_ONE_SPACE_US 1600
#define GREE_ZERO_SPACE_US 540
#define GREE_MSG_SPACE_US 19980
#define GREE_TOLERANCE_US 250

/* Frame structure constants */
#define GREE_STATE_LEN 8       // bytes
#define GREE_BLOCK1_BITS 32    // bits in first block
#define GREE_FOOTER_BITS 3     // inter-block separator bits
#define GREE_BLOCK2_BITS 32    // bits in second block
#define GREE_BLOCK_FOOTER 0x02 // 0b010
#define GREE_RMT_SYMBOLS 70    // total RMT symbols per frame

/* Symbol index boundaries */
#define SYM_HEADER 0
#define SYM_BLOCK1_START 1  // 1..32
#define SYM_FOOTER_START 33 // 33..35
#define SYM_GAP 36
#define SYM_BLOCK2_START 37 // 37..68
#define SYM_EOF 69

/* Byte 0 */
#define GREE_MODE_AUTO 0
#define GREE_MODE_COOL 1
#define GREE_MODE_DRY 2
#define GREE_MODE_FAN 3
#define GREE_MODE_HEAT 4
#define GREE_MODE_ECONO 5

#define GREE_FAN_AUTO 0
#define GREE_FAN_LVL1 1
#define GREE_FAN_LVL2 2
#define GREE_FAN_LVL3 3
#define GREE_FAN_LVL4 4
#define GREE_FAN_LVL5 5

/* Byte 4 – vertical swing */
#define GREE_SWING_V_LAST 0b0000
#define GREE_SWING_V_AUTO 0b0001
#define GREE_SWING_V_UP 0b0010
#define GREE_SWING_V_MID_UP 0b0011
#define GREE_SWING_V_MID 0b0100
#define GREE_SWING_V_MID_DOWN 0b0101
#define GREE_SWING_V_DOWN 0b0110
#define GREE_SWING_V_DOWN_AUTO 0b0111
#define GREE_SWING_V_MID_AUTO 0b1001
#define GREE_SWING_V_UP_AUTO 0b1011

/* Byte 4 – horizontal swing */
#define GREE_SWING_H_OFF 0b000
#define GREE_SWING_H_AUTO 0b001
#define GREE_SWING_H_MAX_LEFT 0b010
#define GREE_SWING_H_LEFT 0b011
#define GREE_SWING_H_MID 0b100
#define GREE_SWING_H_RIGHT 0b101
#define GREE_SWING_H_MAX_RIGHT 0b110

static const char *TAG = "ir_gree";

static QueueHandle_t s_rx_queue;
/* 80 symbols gives comfortable headroom above the required 70 */
static rmt_symbol_word_t s_raw_symbols[80];

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static bool rmt_rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)user_data, edata, &high_task_woken);
    return high_task_woken == pdTRUE;
}

static inline bool in_range(uint32_t value, uint32_t target)
{
    return (value >= target - GREE_TOLERANCE_US) && (value <= target + GREE_TOLERANCE_US);
}

/* ── Checksum ─────────────────────────────────────────────────────────────── */

static uint8_t gree_calc_checksum(const uint8_t state[GREE_STATE_LEN])
{
    uint8_t sum = 10; // Kelvinator checksum start value
    for (int i = 0; i < 4; i++)
        sum += (state[i] & 0x0F); // low nibbles of bytes 0-3
    for (int i = 4; i < 7; i++)
        sum += (state[i] >> 4); // high nibbles of bytes 4-6
    return sum & 0x0F;
}

static bool gree_valid_checksum(const uint8_t state[GREE_STATE_LEN])
{
    return (state[7] >> 4) == gree_calc_checksum(state);
}

/* ── Decoder ──────────────────────────────────────────────────────────────── */

/**
 * @brief Decode a GREE AC IR frame from RMT symbols.
 *
 * @param[in]  symbols     RMT symbols from the receiver callback
 * @param[in]  num_symbols Number of received symbols
 * @param[out] state       Decoded 8-byte state array (caller-zeroed)
 * @return true  on success, false on any timing or structural mismatch
 */
static bool gree_decode(const rmt_symbol_word_t *symbols, size_t num_symbols, uint8_t state[GREE_STATE_LEN])
{
    if (num_symbols < GREE_RMT_SYMBOLS) {
        return false;
    }

    /* Header */
    if (!in_range(symbols[SYM_HEADER].duration0, GREE_HDR_MARK_US) ||
        !in_range(symbols[SYM_HEADER].duration1, GREE_HDR_SPACE_US)) {
        return false;
    }

    /* Block 1: bytes 0-3, 32 bits, LSB first */
    for (int i = 0; i < GREE_BLOCK1_BITS; i++) {
        const rmt_symbol_word_t *s = &symbols[SYM_BLOCK1_START + i];
        if (!in_range(s->duration0, GREE_BIT_MARK_US)) {
            return false;
        }
        if (in_range(s->duration1, GREE_ONE_SPACE_US)) {
            state[i / 8] |= (uint8_t)(1u << (i % 8));
        } else if (!in_range(s->duration1, GREE_ZERO_SPACE_US)) {
            return false;
        }
    }

    /* Footer: 3 bits encoding 0b010, LSB first */
    uint8_t footer = 0;
    for (int i = 0; i < GREE_FOOTER_BITS; i++) {
        const rmt_symbol_word_t *s = &symbols[SYM_FOOTER_START + i];
        if (!in_range(s->duration0, GREE_BIT_MARK_US)) {
            return false;
        }
        if (in_range(s->duration1, GREE_ONE_SPACE_US)) {
            footer |= (uint8_t)(1u << i);
        } else if (!in_range(s->duration1, GREE_ZERO_SPACE_US)) {
            return false;
        }
    }
    if (footer != GREE_BLOCK_FOOTER) {
        ESP_LOGW(TAG, "footer mismatch: got 0x%X, expected 0x%X", footer, GREE_BLOCK_FOOTER);
        return false;
    }

    /* Inter-block gap: 620 µs mark + ~20 ms space */
    if (!in_range(symbols[SYM_GAP].duration0, GREE_BIT_MARK_US) || symbols[SYM_GAP].duration1 < 15000) {
        return false;
    }

    /* Block 2: bytes 4-7, 32 bits, LSB first */
    for (int i = 0; i < GREE_BLOCK2_BITS; i++) {
        const rmt_symbol_word_t *s = &symbols[SYM_BLOCK2_START + i];
        if (!in_range(s->duration0, GREE_BIT_MARK_US)) {
            return false;
        }
        if (in_range(s->duration1, GREE_ONE_SPACE_US)) {
            state[4 + i / 8] |= (uint8_t)(1u << (i % 8));
        } else if (!in_range(s->duration1, GREE_ZERO_SPACE_US)) {
            return false;
        }
    }

    /* End-of-frame mark (duration1 will be 0 due to RMT timeout) */
    if (!in_range(symbols[SYM_EOF].duration0, GREE_BIT_MARK_US)) {
        return false;
    }

    return true;
}

/* ── Pretty-print helpers ─────────────────────────────────────────────────── */

static const char *mode_str(uint8_t m)
{
    switch (m) {
    case GREE_MODE_AUTO:
        return "AUTO";
    case GREE_MODE_COOL:
        return "COOL";
    case GREE_MODE_DRY:
        return "DRY";
    case GREE_MODE_FAN:
        return "FAN";
    case GREE_MODE_HEAT:
        return "HEAT";
    case GREE_MODE_ECONO:
        return "ECONO";
    default:
        return "?";
    }
}

static const char *fan_str(uint8_t f)
{
    switch (f) {
    case GREE_FAN_AUTO:
        return "AUTO";
    case GREE_FAN_LVL1:
        return "Level 1";
    case GREE_FAN_LVL2:
        return "Level 2";
    case GREE_FAN_LVL3:
        return "Level 3";
    case GREE_FAN_LVL4:
        return "Level 4";
    case GREE_FAN_LVL5:
        return "Level 5";
    default:
        return "?";
    }
}

static const char *swing_v_str(uint8_t v)
{
    switch (v) {
    case GREE_SWING_V_LAST:
        return "LAST";
    case GREE_SWING_V_AUTO:
        return "AUTO";
    case GREE_SWING_V_UP:
        return "UP";
    case GREE_SWING_V_MID_UP:
        return "MID-UP";
    case GREE_SWING_V_MID:
        return "MID";
    case GREE_SWING_V_MID_DOWN:
        return "MID-DOWN";
    case GREE_SWING_V_DOWN:
        return "DOWN";
    case GREE_SWING_V_DOWN_AUTO:
        return "DOWN-AUTO";
    case GREE_SWING_V_MID_AUTO:
        return "MID-AUTO";
    case GREE_SWING_V_UP_AUTO:
        return "UP-AUTO";
    default:
        return "?";
    }
}

static const char *swing_h_str(uint8_t h)
{
    switch (h) {
    case GREE_SWING_H_OFF:
        return "OFF";
    case GREE_SWING_H_AUTO:
        return "AUTO";
    case GREE_SWING_H_MAX_LEFT:
        return "MAX-LEFT";
    case GREE_SWING_H_LEFT:
        return "LEFT";
    case GREE_SWING_H_MID:
        return "MID";
    case GREE_SWING_H_RIGHT:
        return "RIGHT";
    case GREE_SWING_H_MAX_RIGHT:
        return "MAX-RIGHT";
    default:
        return "?";
    }
}

static const char *disp_temp_str(uint8_t d)
{
    switch (d) {
    case 0:
        return "OFF";
    case 1:
        return "SET-POINT";
    case 2:
        return "INDOOR";
    case 3:
        return "OUTDOOR";
    default:
        return "?";
    }
}

static void print_both_frames(const uint8_t f1[GREE_STATE_LEN], const uint8_t f2[GREE_STATE_LEN])
{
    /* ── Frame 1 fields ── */
    uint8_t mode = (f1[0] >> 0) & 0x07;
    uint8_t power = (f1[0] >> 3) & 0x01;
    uint8_t fan = (f1[0] >> 4) & 0x03;
    uint8_t swing_auto = (f1[0] >> 6) & 0x01;
    uint8_t sleep_mode = (f1[0] >> 7) & 0x01;

    uint8_t temp_raw = (f1[1] >> 0) & 0x0F;
    uint8_t timer_half = (f1[1] >> 4) & 0x01;
    uint8_t timer_tens = (f1[1] >> 5) & 0x03;
    uint8_t timer_en = (f1[1] >> 7) & 0x01;

    uint8_t timer_hrs = (f1[2] >> 0) & 0x0F;
    uint8_t turbo = (f1[2] >> 4) & 0x01;
    uint8_t light = (f1[2] >> 5) & 0x01;
    uint8_t xfan = (f1[2] >> 7) & 0x01;

    uint8_t use_f = (f1[3] >> 3) & 0x01;
    uint8_t temp_xtra_f = (f1[3] >> 2) & 0x01;

    uint8_t swing_v = (f1[4] >> 0) & 0x0F;
    uint8_t swing_h = (f1[4] >> 4) & 0x07;

    uint8_t disp_temp = (f1[5] >> 0) & 0x03;
    uint8_t ifeel = (f1[5] >> 2) & 0x01;
    uint8_t wifi = (f1[5] >> 6) & 0x01;

    uint8_t econo = (f1[7] >> 2) & 0x01;

    uint8_t temp_c = temp_raw + 16;
    uint8_t temp_f_val = use_f ? (uint8_t)((temp_c * 9 / 5) + 32 + temp_xtra_f) : 0;

    uint16_t timer_min = 0;
    if (timer_en) {
        timer_min = (uint16_t)((timer_tens * 10 + timer_hrs) * 60 + (timer_half ? 30 : 0));
    }

    /* ── Frame 2 fields ── */
    uint8_t fan2 = (f2[6] >> 4) & 0x07;
    uint8_t quiet = (f2[4] >> 7) & 0x01;
    uint8_t cooling_sensation = (f2[7] >> 3) & 0x01;

    bool csum1_ok = gree_valid_checksum(f1);
    bool csum2_ok = gree_valid_checksum(f2);

    printf("─── GREE AC ───────────────────────────────────\n");
    printf("  Raw[1]:   %02X %02X %02X %02X  %02X %02X %02X %02X  (%s)\n", f1[0], f1[1], f1[2], f1[3], f1[4], f1[5],
           f1[6], f1[7], csum1_ok ? "CRC OK" : "CRC FAIL");
    printf("  Raw[2]:   %02X %02X %02X %02X  %02X %02X %02X %02X  (%s)\n", f2[0], f2[1], f2[2], f2[3], f2[4], f2[5],
           f2[6], f2[7], csum2_ok ? "CRC OK" : "CRC FAIL");
    printf("  Power:    %s\n", power ? "ON" : "OFF");
    printf("  Mode:     %s\n", mode_str(mode));
    if (use_f) {
        printf("  Temp:     %u°F\n", temp_f_val);
    } else {
        printf("  Temp:     %u°C\n", temp_c);
    }
    printf("  BasicFan: %s\n", fan_str(fan));
    printf("  Fan:      %s\n", fan_str(fan2));
    printf("  SwingV:   %s%s\n", swing_v_str(swing_v), swing_auto ? " (AUTO)" : "");
    printf("  SwingH:   %s\n", swing_h_str(swing_h));
    printf("  Sleep:    %s\n", sleep_mode ? "ON" : "OFF");
    printf("  Turbo:    %s\n", turbo ? "ON" : "OFF");
    printf("  Econo:    %s\n", econo ? "ON" : "OFF");
    printf("  Light:    %s\n", light ? "ON" : "OFF");
    printf("  XFan:     %s\n", xfan ? "ON" : "OFF");
    printf("  IFeel:    %s\n", ifeel ? "ON" : "OFF");
    printf("  WiFi:     %s\n", wifi ? "ON" : "OFF");
    printf("  DispTemp: %s\n", disp_temp_str(disp_temp));
    printf("  Quiet:    %s\n", quiet ? "ON" : "OFF");
    printf("  CoolingSensation: %s\n", cooling_sensation ? "ON" : "OFF");
    if (timer_en) {
        printf("  Timer:    %uh%s\n", timer_min / 60, timer_half ? "30m" : "00m");
    } else {
        printf("  Timer:    OFF\n");
    }
    printf("───────────────────────────────────────────────\n");
}

static bool gree_is_second_frame(const uint8_t state[GREE_STATE_LEN])
{
    /* Second frame has fixed bits[7:4] of byte[3] = 0b0111 (0x7) */
    return ((state[3] >> 4) & 0x0F) == 0x7;
}

/* ── Main ─────────────────────────────────────────────────────────────────── */

void app_main(void)
{
    printf("GREE AC IR Decoder (YAPOF20) — GPIO %d\n", IR_RX_GPIO);

    s_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));

    rmt_rx_channel_config_t rx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = IR_RESOLUTION_HZ,
        .mem_block_symbols = 80,
        .gpio_num = IR_RX_GPIO,
    };
    rmt_channel_handle_t rx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));

    rmt_rx_event_callbacks_t cbs = {.on_recv_done = rmt_rx_done_callback};
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, s_rx_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_channel));

    /*
     * signal_range_max_ns must exceed the ~20 ms inter-block gap so the RMT
     * does not terminate mid-frame, yet still fires after the final ~20 ms
     * end-of-frame silence.  22 ms comfortably satisfies both constraints.
     */
    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns = 1250,     // filter glitches < 1.25 µs
        .signal_range_max_ns = 22000000, // 22 ms — triggers after final silence
    };
    ESP_ERROR_CHECK(rmt_receive(rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &recv_cfg));

    /*
     * Frame sequencing: the remote sends Frame 1 then Frame 2 ~40 ms later.
     * We buffer Frame 1 and only print after Frame 2 arrives, so no output
     * occurs between the two frames and neither is missed.
     */
    bool has_frame1 = false;
    uint8_t frame1[GREE_STATE_LEN];

    rmt_rx_done_event_data_t rx_data;
    while (1) {
        if (xQueueReceive(s_rx_queue, &rx_data, portMAX_DELAY) == pdTRUE) {
            uint8_t state[GREE_STATE_LEN] = {0};

            if (gree_decode(rx_data.received_symbols, rx_data.num_symbols, state)) {
                if (gree_is_second_frame(state)) {
                    if (has_frame1) {
                        print_both_frames(frame1, state);
                        has_frame1 = false;
                    } else {
                        /* Frame 2 without a preceding Frame 1 — log raw bytes */
                        printf("Frame 2 (no Frame 1): %02X %02X %02X %02X  %02X %02X %02X %02X\n", state[0], state[1],
                               state[2], state[3], state[4], state[5], state[6], state[7]);
                    }
                } else {
                    /* Buffer Frame 1; discard any stale previous Frame 1 */
                    memcpy(frame1, state, GREE_STATE_LEN);
                    has_frame1 = true;
                }
            } else if (rx_data.num_symbols >= GREE_RMT_SYMBOLS) {
                /* Full-length frame that failed structural checks — worth logging */
                printf("Unknown IR signal (%zu symbols)\n", rx_data.num_symbols);
                has_frame1 = false;
            }
            /* Short frames (< 70 symbols) are silently discarded */

            ESP_ERROR_CHECK(rmt_receive(rx_channel, s_raw_symbols, sizeof(s_raw_symbols), &recv_cfg));
        }
    }
}
