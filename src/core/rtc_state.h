#pragma once

#include <stddef.h>
#include <stdint.h>

#include "wake_record.h"

struct WifiCache {
    uint8_t bssid[6];
    uint8_t channel;
    bool valid;
};

// State in RTC memory: survives deep sleep and soft resets, not power loss.
void rtcBegin();  // validates RTC memory and starts a new wake record
WakeRecord &rtcCurrentWake();
const WakeRecord *rtcPreviousWake();  // nullptr on the first boot
void rtcDumpToSerial();
uint32_t rtcNextPlayPauseSeq();
WifiCache &rtcWifiCache();

// formatWakeRecord with reset reason and wake cause names filled in
size_t rtcFormat(const WakeRecord &r, char *out, size_t cap);
