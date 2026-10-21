#include "debouncer.h"

Debouncer::Debouncer(uint32_t debounceMs, bool initiallyPressed)
    : debounceMs_(debounceMs), stable_(initiallyPressed), raw_(initiallyPressed) {}

int8_t Debouncer::update(bool pressed, uint32_t nowMs) {
    if (pressed != raw_) {
        raw_ = pressed;
        rawSinceMs_ = nowMs;
    }
    if (raw_ == stable_ || nowMs - rawSinceMs_ < debounceMs_) return 0;

    stable_ = raw_;
    return stable_ ? 1 : -1;
}
