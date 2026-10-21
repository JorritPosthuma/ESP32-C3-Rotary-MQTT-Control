#include "rate_coalescer.h"

int RateCoalescer::take(uint32_t nowMs) {
    if (pending_ == 0) return 0;
    if (sentAny_ && nowMs - lastAt_ < intervalMs_) return 0;

    int clicks = pending_;
    pending_ = 0;
    lastAt_ = nowMs;
    sentAny_ = true;
    return clicks;
}
