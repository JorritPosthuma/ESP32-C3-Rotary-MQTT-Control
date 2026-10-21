#pragma once

#include <stdint.h>

// Accumulates detents and releases them at most once per interval; the first release is immediate.
class RateCoalescer {
   public:
    explicit RateCoalescer(uint32_t intervalMs) : intervalMs_(intervalMs) {}

    void add(int clicks) { pending_ += clicks; }
    int take(uint32_t nowMs);

   private:
    uint32_t intervalMs_;
    int pending_ = 0;
    bool sentAny_ = false;
    uint32_t lastAt_ = 0;
};
