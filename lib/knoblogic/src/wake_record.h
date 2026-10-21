#pragma once

#include <stddef.h>
#include <stdint.h>

// One boot of the knob, kept in RTC memory. Pin fields use the LEVEL_* bit layout.
struct WakeRecord {
    uint32_t boot;
    uint8_t resetReason;
    uint8_t wakeCause;
    uint8_t wakePins;       // pins that triggered the wake
    uint8_t levelsAtBoot;   // 1 = HIGH
    uint8_t levelsAtSleep;  // 1 = HIGH
    bool slept;             // false = boot ended without deep sleep (crash, reset, still awake)
    uint32_t awakeMs;
    uint32_t encoderEdges;
    uint16_t wifiMs;      // ms since boot until Wi-Fi was up, 0 = n/a
    uint16_t firstAckMs;  // ms since boot until the first command was acknowledged, 0 = n/a
};

// "#12 reset=DEEPSLEEP wake=GPIO by=A B | boot: A=1 B=1 BTN=1 | sleep: ... | awake=30.1s ..."
size_t formatWakeRecord(const WakeRecord &r, const char *resetName, const char *wakeName, char *out, size_t cap);
