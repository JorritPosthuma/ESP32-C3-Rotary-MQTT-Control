#pragma once

#include <stdint.h>

// Debounces a push button that is polled with a timestamp.
class Debouncer {
   public:
    Debouncer(uint32_t debounceMs, bool initiallyPressed);

    // Returns +1 on a stable press, -1 on a stable release, else 0.
    int8_t update(bool pressed, uint32_t nowMs);

   private:
    uint32_t debounceMs_;
    bool stable_;
    bool raw_;
    uint32_t rawSinceMs_ = 0;
};
