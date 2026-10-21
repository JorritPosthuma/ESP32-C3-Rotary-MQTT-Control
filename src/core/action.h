#pragma once

#include <stdint.h>

// What a knob variant does with input. The core owns sleep, Wi-Fi and input.
class Action {
   public:
    virtual ~Action() = default;
    virtual void begin(uint32_t nowMs) = 0;   // Wi-Fi is up
    virtual void onClicks(int clicks) = 0;    // signed detents, + = clockwise
    virtual void onPress() = 0;
    virtual void loop(uint32_t nowMs) = 0;    // pump I/O, flush coalesced commands
    virtual uint32_t firstAckMs() const = 0;  // millis() of the first acknowledged command, 0 if none
    virtual void end() = 0;                   // before deep sleep
};

// One definition per variant (selected by build_src_filter)
Action &makeAction();
