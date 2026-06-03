# Copilot Instructions

## Build commands

ESP-IDF must be sourced before any `idf.py` invocation:

```bash
source /home/xlongfeng/Projects/esp/esp-idf/export.sh
```

| Task | Command |
|------|---------|
| Build | `idf.py build` |
| Clean build | `idf.py fullclean && idf.py build` |
| Flash + monitor | `idf.py -p /dev/ttyUSB0 flash monitor` |
| Configure GPIO | `idf.py menuconfig` → "IR Decoder Configuration" |
| Format source | `clang-format -i main/ir-dec.c` |

There is no automated test suite for this project.

## Architecture

All logic lives in a single file: `main/ir-dec.c`.

The decoder runs in a FreeRTOS loop using the ESP-IDF RMT RX peripheral (`esp_driver_rmt`). On each button press the GREE remote sends **two consecutive frames** (~40 ms apart). The loop:

1. Receives Frame 1 → decodes it → stores in `frame1[]`, sets `has_frame1 = true` — **no output yet**
2. Receives Frame 2 → decodes it → calls `print_both_frames(frame1, state)` — single combined printout

This buffering is intentional: printing during Frame 1 would delay re-arming the RMT receiver and risk missing Frame 2.

Frame type is identified by `state[3] >> 4`: `0x5` = Frame 1, `0x7` = Frame 2.

The RMT `signal_range_max_ns = 22 ms` is carefully chosen: it must exceed the ~20 ms inter-block gap (so the receiver doesn't trigger mid-frame) but fire during the ~40 ms inter-frame silence (so it re-arms before Frame 2 arrives).

## Key conventions

### Code style
- LLVM-based clang-format, 4-space indent, 120-column limit (see `.clang-format`)
- Run `clang-format -i main/ir-dec.c` after every edit

### Protocol constants
All GREE timing and frame-structure constants are `#define`d at the top of `ir-dec.c`. Do not hardcode µs values inline.

### Fan speed
`BasicFan` (Frame 1, 2-bit) and `Fan` (Frame 2, 3-bit) are **independent fields** — both are decoded and printed separately. When transmitting:
- `Fan` (frame 2) holds the full speed: 0=AUTO, 1–5=Level 1–5
- `BasicFan` (frame 1) mirrors it, capped at 3: `min(fan_level, 3)`
- Quiet ON → both fields = 1; Turbo ON → `BasicFan` = 3, `Fan` = 5

### Checksum
Top nibble of `state[7]`, same algorithm for both frames:
```c
uint8_t sum = 10;
for (int i = 0; i < 4; i++) sum += (state[i] & 0x0F);
for (int i = 4; i < 7; i++) sum += (state[i] >> 4);
state[7] = (state[7] & 0x0F) | ((sum & 0x0F) << 4);
```

### Adding new decoded fields
- Add extraction near the other frame fields in `print_both_frames()`
- Update the byte-map comment block at the top of `ir-dec.c`
- Update the byte-map tables in `README.md`
