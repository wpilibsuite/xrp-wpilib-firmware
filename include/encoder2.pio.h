// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// -------- //
// encoder2 //
// -------- //

#define encoder2_wrap_target 16
#define encoder2_wrap 31

#define encoder2_VERSION 102
#define encoder2_CYCLES_PER_COUNT 16

static const uint16_t encoder2_program_instructions[] = {
    0x0b10, //  0: jmp    16                     [11]
    0x0315, //  1: jmp    21                     [3]
    0x0318, //  2: jmp    24                     [3]
    0x0b11, //  3: jmp    17                     [11]
    0x0318, //  4: jmp    24                     [3]
    0x0b10, //  5: jmp    16                     [11]
    0x0b11, //  6: jmp    17                     [11]
    0x0315, //  7: jmp    21                     [3]
    0x0315, //  8: jmp    21                     [3]
    0x0b11, //  9: jmp    17                     [11]
    0x0b10, // 10: jmp    16                     [11]
    0x0318, // 11: jmp    24                     [3]
    0x0b11, // 12: jmp    17                     [11]
    0x0318, // 13: jmp    24                     [3]
    0x0315, // 14: jmp    21                     [3]
    0x0b10, // 15: jmp    16                     [11]
            //     .wrap_target
    0x0052, // 16: jmp    x--, 18
    0xe020, // 17: set    x, 0
    0x4002, // 18: in     pins, 2
    0xa0e6, // 19: mov    osr, isr
    0x60a4, // 20: out    pc, 4
    0x0079, // 21: jmp    !y, 25
    0xa04a, // 22: mov    y, !y
    0x0019, // 23: jmp    25
    0x0076, // 24: jmp    !y, 22
    0xa0e6, // 25: mov    osr, isr
    0xa0c9, // 26: mov    isr, !x
    0x4041, // 27: in     y, 1
    0x8000, // 28: push   noblock
    0x60c2, // 29: out    isr, 2
    0xa0eb, // 30: mov    osr, !null
    0x603f, // 31: out    x, 31
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program encoder2_program = {
    .instructions = encoder2_program_instructions,
    .length = 32,
    .origin = 0,
};

static inline pio_sm_config encoder2_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + encoder2_wrap_target, offset + encoder2_wrap);
    return c;
}

static inline void encoder2_program_init(PIO pio, uint sm, uint offset, uint base_pin) {
    pio_gpio_init(pio, base_pin);
    pio_gpio_init(pio, base_pin+1);
    pio_sm_set_consecutive_pindirs(pio, sm, base_pin, 2, false);
    pio_sm_config c = encoder2_program_get_default_config(offset);
    sm_config_set_in_pins(&c, base_pin);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif
