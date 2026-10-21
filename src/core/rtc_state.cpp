#include "rtc_state.h"

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_sleep.h>
#include <esp_system.h>

constexpr uint32_t RTC_MAGIC = 0x4B4E0001;  // bump when RtcState changes layout
constexpr int WAKE_LOG_SIZE = 20;

struct RtcState {
    uint32_t magic;
    uint32_t bootCount;
    WakeRecord log[WAKE_LOG_SIZE];
    WifiCache wifi;
    uint32_t playPauseSeq;
};

RTC_NOINIT_ATTR static RtcState rtc;

void rtcBegin() {
    if (rtc.magic != RTC_MAGIC) {
        memset(&rtc, 0, sizeof(rtc));
        rtc.magic = RTC_MAGIC;
    }
    rtc.bootCount++;
    WakeRecord &r = rtcCurrentWake();
    memset(&r, 0, sizeof(r));
    r.boot = rtc.bootCount;
}

WakeRecord &rtcCurrentWake() {
    return rtc.log[rtc.bootCount % WAKE_LOG_SIZE];
}

const WakeRecord *rtcPreviousWake() {
    return rtc.bootCount > 1 ? &rtc.log[(rtc.bootCount - 1) % WAKE_LOG_SIZE] : nullptr;
}

uint32_t rtcNextPlayPauseSeq() {
    return ++rtc.playPauseSeq;
}

WifiCache &rtcWifiCache() {
    return rtc.wifi;
}

static const char *resetReasonName(uint8_t reason) {
    switch (reason) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_SW: return "SW";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_USB: return "USB";
        case ESP_RST_JTAG: return "JTAG";
        default: return "OTHER";
    }
}

static const char *wakeCauseName(uint8_t cause) {
    switch (cause) {
        case ESP_SLEEP_WAKEUP_UNDEFINED: return "NONE";
        case ESP_SLEEP_WAKEUP_TIMER: return "TIMER";
        case ESP_SLEEP_WAKEUP_GPIO: return "GPIO";
        default: return "OTHER";
    }
}

size_t rtcFormat(const WakeRecord &r, char *out, size_t cap) {
    return formatWakeRecord(r, resetReasonName(r.resetReason), wakeCauseName(r.wakeCause), out, cap);
}

void rtcDumpToSerial() {
    char line[200];
    Serial.println("--- Wake log (oldest first) ---");
    uint32_t first = rtc.bootCount > WAKE_LOG_SIZE ? rtc.bootCount - WAKE_LOG_SIZE + 1 : 1;
    for (uint32_t n = first; n <= rtc.bootCount; n++) {
        rtcFormat(rtc.log[n % WAKE_LOG_SIZE], line, sizeof(line));
        Serial.println(line);
    }
    Serial.println("-------------------------------");
}
