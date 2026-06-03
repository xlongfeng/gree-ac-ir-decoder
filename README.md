# GREE AC IR Decoder / Transmitter

ESP32 project for receiving and decoding GREE YAPOF20 / YAPOF3 infrared remote
control signals. This document also covers everything a developer needs to
**transmit** GREE IR commands from an ESP32.

---

## Hardware

| Item | Details |
|------|---------|
| MCU | ESP32-C3 (any ESP32 variant works) |
| IR receiver | VS1838B or equivalent 38 kHz demodulator |
| IR LED (TX) | 940 nm, series resistor ~33 Ω for 3.3 V |
| RX GPIO | Default **GPIO 4** — change via `idf.py menuconfig` |
| TX GPIO | Any free GPIO — set in your transmit code |

Wiring the receiver:

```
VS1838B OUT ──► GPIO 4
VS1838B VCC ──► 3.3 V
VS1838B GND ──► GND
```

---

## Building and flashing

```bash
. $IDF_PATH/export.sh
idf.py menuconfig          # optional: change IR_RX_GPIO under "IR Decoder"
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Protocol overview

Each button press on the GREE YAPOF20 / YAPOF3 remote sends **two consecutive
frames**. Both frames share the same wire format (70 RMT symbols, bits LSB-first).

### Wire format (one frame = 70 symbols)

```
[HEADER]   9000 µs mark  + 4500 µs space
[BLOCK 1]  32 bits  → bytes [0..3]
[FOOTER]    3 bits  = 0b010
[GAP]        620 µs mark  + ~20 ms space
[BLOCK 2]  32 bits  → bytes [4..7]
[EOF]        620 µs mark  + ~40 ms silence
```

Bit encoding (NRZ, active-low demodulated signal):

| Bit | Mark    | Space   |
|-----|---------|---------|
| `1` | 620 µs  | 1600 µs |
| `0` | 620 µs  |  540 µs |

---

## Byte map

### Frame 1  (`state[3] >> 4 == 0x5`)

| Byte | Bits   | Field            | Values / notes |
|------|--------|-----------------|----------------|
| [0]  | [2:0]  | Mode            | 0=AUTO 1=COOL 2=DRY 3=FAN 4=HEAT 5=ECONO |
| [0]  | [3]    | Power           | 0=OFF 1=ON |
| [0]  | [5:4]  | BasicFan        | 0=AUTO 1=Lvl1(Quiet) 2=Lvl2 3=Lvl3(Turbo) — when Fan>Lvl3, set BasicFan=Lvl3 |
| [0]  | [6]    | SwingAuto       | 1=auto swing |
| [0]  | [7]    | Sleep           | 1=ON |
| [1]  | [3:0]  | Temp            | offset from 16 °C  (0=16 °C … 14=30 °C) |
| [1]  | [4]    | TimerHalfHr     | adds 30 min to timer |
| [1]  | [6:5]  | TimerTensHr     | tens digit of timer hours |
| [1]  | [7]    | TimerEnabled    | 1=timer on |
| [2]  | [3:0]  | TimerHours      | units digit of timer hours |
| [2]  | [4]    | Turbo           | 1=ON — forces Fan=Lvl5 |
| [2]  | [5]    | Light           | 1=ON |
| [2]  | [6]    | ModelA          | model flag |
| [2]  | [7]    | XFan            | 1=ON (keep fan running after stop) |
| [3]  | [1:0]  | reserved        | 0 |
| [3]  | [2]    | TempExtraF      | 0.5 °F extra (Fahrenheit mode) |
| [3]  | [3]    | UseFahrenheit   | 1=°F display |
| [3]  | [7:4]  | **fixed = 0x5** | frame-1 identifier |
| [4]  | [3:0]  | SwingV          | see swing constants below |
| [4]  | [6:4]  | SwingH          | see swing constants below |
| [5]  | [1:0]  | DisplayTemp     | 0=OFF 1=Set-point 2=Indoor 3=Outdoor |
| [5]  | [2]    | IFeel           | 1=ON |
| [5]  | [5:3]  | **fixed = 0x4** | always 0b100 |
| [5]  | [6]    | WiFi            | 1=ON |
| [6]  | —      | reserved        | 0x00 |
| [7]  | [1:0]  | reserved        | 0 |
| [7]  | [2]    | Econo           | 1=ON |
| [7]  | [3]    | reserved        | 0 |
| [7]  | [7:4]  | **Checksum**    | see below |

### Frame 2  (`state[3] >> 4 == 0x7`)

| Byte | Bits   | Field                | Values / notes |
|------|--------|---------------------|----------------|
| [0]  | —      | repeat of F1 byte 0 | Mode, Power, BasicFan, SwingAuto, Sleep |
| [1]  | —      | repeat of F1 byte 1 | Temp, Timer fields |
| [2]  | —      | repeat of F1 byte 2 | TimerHours, Turbo, Light, ModelA, XFan |
| [3]  | [3:0]  | reserved            | 0 |
| [3]  | [5:4]  | **fixed = 0b11**    | |
| [3]  | [7:6]  | **fixed = 0b01**    | combined [7:4] = 0x7, frame-2 identifier |
| [4]  | [0]    | Sleep2              | |
| [4]  | [6:1]  | Sleep3              | |
| [4]  | [7]    | Quiet               | 1=ON — forces Fan=Lvl1 |
| [5]  | —      | Sleep3 related      | 0x00 |
| [6]  | [3:0]  | Sleep3              | |
| [6]  | [6:4]  | Fan                 | 0=AUTO 1=Lvl1(Quiet) 2=Lvl2 3=Lvl3 4=Lvl4 5=Lvl5(Turbo) — coexists with BasicFan |
| [7]  | [2:0]  | reserved            | 0 |
| [7]  | [3]    | CoolingSensation    | 1=ON |
| [7]  | [7:4]  | **Checksum**        | see below |

### Swing constants

**Vertical (SwingV, bits[3:0] of byte[4]):**

| Value  | Meaning    |
|--------|-----------|
| 0b0000 | Last       |
| 0b0001 | Auto       |
| 0b0010 | Up         |
| 0b0011 | Mid-Up     |
| 0b0100 | Mid        |
| 0b0101 | Mid-Down   |
| 0b0110 | Down       |
| 0b0111 | Down-Auto  |
| 0b1001 | Mid-Auto   |
| 0b1011 | Up-Auto    |

**Horizontal (SwingH, bits[6:4] of byte[4]):**

| Value | Meaning   |
|-------|----------|
| 0b000 | OFF       |
| 0b001 | Auto      |
| 0b010 | Max-Left  |
| 0b011 | Left      |
| 0b100 | Mid       |
| 0b101 | Right     |
| 0b110 | Max-Right |

---

## Checksum

Same algorithm for both frames. Result stored in **bits[7:4] of byte[7]**:

```c
uint8_t gree_checksum(const uint8_t state[8])
{
    uint8_t sum = 10;
    for (int i = 0; i < 4; i++) sum += (state[i] & 0x0F);  // low nibbles  b0-b3
    for (int i = 4; i < 7; i++) sum += (state[i] >> 4);    // high nibbles b4-b6
    return sum & 0x0F;
}

// Write into byte[7]:
state[7] = (state[7] & 0x0F) | (gree_checksum(state) << 4);
```

---

## Sending GREE IR — step by step

### 1. Build the 8-byte state arrays

```c
uint8_t f1[8] = {0};
uint8_t f2[8] = {0};

// ── Frame 1 ──────────────────────────────────────────────
f1[0]  = GREE_MODE_COOL;            // bits[2:0]  mode
f1[0] |= (1 << 3);                  // bit[3]     power ON
// BasicFan (frame 1, 2-bit) and Fan (frame 2, 3-bit) coexist independently.
// Both use 0=AUTO, 1=Lvl1/Quiet, 2=Lvl2, 3=Lvl3/Turbo(BasicFan max).
// Fan extends to 4=Lvl4, 5=Lvl5/Turbo.
// Rule: when Fan > Lvl3, BasicFan must be Lvl3.
uint8_t fan_level = 2;  // e.g. Level 2
f1[0] |= ((fan_level <= 3 ? fan_level : 3) << 4);  // BasicFan: capped at Lvl3
f2[6] |= (fan_level << 4);                          // Fan (full range)

f1[1]  = (26 - 16) & 0x0F;          // bits[3:0]  temp = 26 °C

f1[3]  = 0x50;                      // bits[7:4]  fixed = 0x5 (frame-1 id)

f1[4]  = GREE_SWING_V_AUTO;         // bits[3:0]  SwingV
// f1[4] |= (GREE_SWING_H_AUTO << 4); // bits[6:4]  SwingH (optional)

f1[5]  = (0x4 << 3);                // bits[5:3]  fixed = 0x4

// Apply checksum
f1[7]  = (gree_checksum(f1) << 4);

// ── Frame 2 ──────────────────────────────────────────────
f2[0] = f1[0];  f2[1] = f1[1];  f2[2] = f1[2];  // repeat bytes 0-2

f2[3]  = 0x70;                      // bits[7:4] fixed = 0x7 (frame-2 id)

// f2[4] |= (1 << 7);               // bit[7]     Quiet ON

// Apply checksum
f2[7]  = (gree_checksum(f2) << 4);
```

### 2. Encode a frame into RMT symbols

Each 8-byte frame encodes to exactly **69 active symbols + 1 EOF mark** (70 total).

```c
#define GREE_HDR_MARK_US   9000
#define GREE_HDR_SPACE_US  4500
#define GREE_BIT_MARK_US    620
#define GREE_ONE_SPACE_US  1600
#define GREE_ZERO_SPACE_US  540
#define GREE_MSG_SPACE_US 19980

// Returns number of symbols written (always 70).
static int gree_encode(const uint8_t state[8], rmt_symbol_word_t *sym)
{
    int n = 0;

    // Header
    sym[n].level0    = 1; sym[n].duration0 = GREE_HDR_MARK_US;
    sym[n].level1    = 0; sym[n++].duration1 = GREE_HDR_SPACE_US;

    // Block 1: bytes 0-3, LSB first
    for (int i = 0; i < 32; i++) {
        uint8_t bit = (state[i / 8] >> (i % 8)) & 1;
        sym[n].level0    = 1; sym[n].duration0 = GREE_BIT_MARK_US;
        sym[n].level1    = 0;
        sym[n++].duration1 = bit ? GREE_ONE_SPACE_US : GREE_ZERO_SPACE_US;
    }

    // Footer: 3 bits = 0b010 (LSB first: 0, 1, 0)
    uint8_t footer = 0x02; // 0b010
    for (int i = 0; i < 3; i++) {
        uint8_t bit = (footer >> i) & 1;
        sym[n].level0    = 1; sym[n].duration0 = GREE_BIT_MARK_US;
        sym[n].level1    = 0;
        sym[n++].duration1 = bit ? GREE_ONE_SPACE_US : GREE_ZERO_SPACE_US;
    }

    // Inter-block gap
    sym[n].level0    = 1; sym[n].duration0 = GREE_BIT_MARK_US;
    sym[n].level1    = 0; sym[n++].duration1 = GREE_MSG_SPACE_US;

    // Block 2: bytes 4-7, LSB first
    for (int i = 0; i < 32; i++) {
        uint8_t bit = (state[4 + i / 8] >> (i % 8)) & 1;
        sym[n].level0    = 1; sym[n].duration0 = GREE_BIT_MARK_US;
        sym[n].level1    = 0;
        sym[n++].duration1 = bit ? GREE_ONE_SPACE_US : GREE_ZERO_SPACE_US;
    }

    // EOF mark (space duration = 0 → RMT inserts idle level)
    sym[n].level0    = 1; sym[n].duration0 = GREE_BIT_MARK_US;
    sym[n].level1    = 0; sym[n++].duration1 = 0;

    return n; // 70
}
```

### 3. Transmit via ESP-IDF RMT TX

```c
#include "driver/rmt_tx.h"

#define IR_TX_GPIO   5       // change as needed
#define IR_TX_CARRIER_HZ 38000

rmt_channel_handle_t tx_channel = NULL;
rmt_tx_channel_config_t tx_cfg = {
    .clk_src           = RMT_CLK_SRC_DEFAULT,
    .resolution_hz     = 1000000,   // 1 µs per tick
    .mem_block_symbols = 128,
    .gpio_num          = IR_TX_GPIO,
    .trans_queue_depth = 4,
};
ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_channel));

rmt_carrier_config_t carrier = {
    .frequency_hz = IR_TX_CARRIER_HZ,
    .duty_cycle   = 0.33f,
};
ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier));
ESP_ERROR_CHECK(rmt_enable(tx_channel));

rmt_transmit_config_t tx_xmit = { .loop_count = 0 };
rmt_copy_encoder_handle_t encoder = NULL;
rmt_new_copy_encoder(&(rmt_copy_encoder_config_t){}, &encoder);

// Encode and send Frame 1
rmt_symbol_word_t symbols[70];
gree_encode(f1, symbols);
ESP_ERROR_CHECK(rmt_transmit(tx_channel, encoder, symbols,
                             sizeof(symbols), &tx_xmit));
rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);

// ~40 ms gap between frames (GREE spec)
vTaskDelay(pdMS_TO_TICKS(40));

// Encode and send Frame 2
gree_encode(f2, symbols);
ESP_ERROR_CHECK(rmt_transmit(tx_channel, encoder, symbols,
                             sizeof(symbols), &tx_xmit));
rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);
```

---

## Project structure

```
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt       adds esp_driver_rmt dependency
│   ├── ir-dec.c             RMT RX decoder + pretty-printer
│   └── Kconfig.projbuild    IR_RX_GPIO menu item (default GPIO 4)
├── .clang-format            code style
└── README.md                this file
```

---

## References

- [GREE YAPOF3 IR format](https://snowstar.org/2022/02/21/gree-yapof3-ir-format/)
- [IRremoteESP8266 — ir_Gree.cpp](https://github.com/crankyoldgit/IRremoteESP8266/blob/master/src/ir_Gree.cpp)
- [IRremoteESP8266 — ir_Kelvinator.cpp](https://github.com/crankyoldgit/IRremoteESP8266/blob/master/src/ir_Kelvinator.cpp)
- [ESP-IDF RMT driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)

