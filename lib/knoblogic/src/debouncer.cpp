#include "debouncer.h"

Debouncer::Debouncer(uint32_t debounceMs, bool initiallyPressed)
    : debounceMs_(debounceMs), stable_(initiallyPressed), raw_(initiallyPressed) {}

int8_t Debouncer::update(bool pressed, uint32_t nowMs) {
    // Settle the previous level before taking the new one: edges replayed from the ring after
    // a blocked loop arrive in one burst, and a whole tap must still count. Signed time, since
    // a replayed edge can be older than the last poll.
    int8_t change = 0;
    if (raw_ != stable_ && (int32_t)(nowMs - rawSinceMs_) >= (int32_t)debounceMs_) {
        stable_ = raw_;
        change = stable_ ? 1 : -1;
    }
    if (pressed != raw_) {
        raw_ = pressed;
        rawSinceMs_ = nowMs;
    }
    return change;
}
