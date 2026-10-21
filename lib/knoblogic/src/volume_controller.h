#pragma once

#include <stdint.h>

// Turns detents into absolute hub volume SETs (0..100): coalesced, rate limited, and kept in
// sync with what the hub reports.
class VolumeController {
   public:
    VolumeController(int step, uint32_t minIntervalMs = 100, uint32_t echoWindowMs = 500);

    void setMax(int max) { max_ = max; }  // upper clamp, default 100
    void addClicks(int clicks);
    void onHubVolume(int volume, uint32_t nowMs);  // GET reply or MB 64 notification
    int takeSet(uint32_t nowMs);                   // value to SET now, or -1
    bool known() const { return known_; }
    int target() const { return target_; }

   private:
    static constexpr int RECENT = 4;

    int step_;
    uint32_t minIntervalMs_;
    uint32_t echoWindowMs_;
    int max_ = 100;
    bool known_ = false;
    int pendingClicks_ = 0;
    int target_ = 0;
    int lastSent_ = -1;
    bool sentAny_ = false;
    uint32_t lastSentAt_ = 0;
    int recentValue_[RECENT] = {-1, -1, -1, -1};
    uint32_t recentAt_[RECENT] = {};
    int recentNext_ = 0;

    bool isOwnEcho(int volume, uint32_t nowMs) const;
    int clamp(int volume) const;
};
