#pragma once

#include <stdint.h>

// Level bit layout used throughout: bit 0 = encoder A, bit 1 = encoder B, bit 2 = button.
constexpr uint8_t LEVEL_A = 1 << 0;
constexpr uint8_t LEVEL_B = 1 << 1;
constexpr uint8_t LEVEL_BUTTON = 1 << 2;

struct WakeMasks {
    uint64_t onLow;   // GPIO bit mask of pins that should wake on LOW
    uint64_t onHigh;  // GPIO bit mask of pins that should wake on HIGH
};

// Deep-sleep GPIO wake is level triggered, so each pin wakes on the opposite of its current
// level. A pin that is already LOW at sleep entry then wakes on change instead of looping.
WakeMasks computeWakeMasks(uint8_t levels, const uint8_t pins[3]);

// True when the button woke the chip by being pressed (it was HIGH at sleep entry).
// A wake on button release is not a press.
bool wakeIsPress(uint8_t levelsAtSleep, uint8_t wakePins);
