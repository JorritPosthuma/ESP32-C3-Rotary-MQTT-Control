#include <Arduino.h>
#include <WiFi.h>
#include <esp_sleep.h>

#include "config.h"
#include "core/action.h"
#include "core/input.h"
#include "core/net.h"
#include "core/pins.h"
#include "core/rtc_state.h"
#include "core/sleep.h"
#include "wake_masks.h"

constexpr uint32_t WIFI_BUDGET_MS = 5000;

static Action &action = makeAction();
static bool actionStarted = false;

static uint16_t clampMs(uint32_t ms) {
    return ms > 0xFFFF ? 0xFFFF : (uint16_t)ms;
}

// Record how and why we woke up, before anything else touches the pins
static void captureWake() {
    for (uint8_t pin : {PIN_A, PIN_B, PIN_BUTTON}) pinMode(pin, INPUT);

    WakeRecord &r = rtcCurrentWake();
    r.resetReason = esp_reset_reason();
    r.wakeCause = esp_sleep_get_wakeup_cause();
    r.levelsAtBoot = inputReadLevels();
    if (r.wakeCause == ESP_SLEEP_WAKEUP_GPIO) {
        uint64_t status = esp_sleep_get_gpio_wakeup_status();
        r.wakePins = ((status >> PIN_A) & 1 ? LEVEL_A : 0) | ((status >> PIN_B) & 1 ? LEVEL_B : 0) |
                     ((status >> PIN_BUTTON) & 1 ? LEVEL_BUTTON : 0);
    }
}

[[noreturn]] static void finishAndSleep() {
    WakeRecord &r = rtcCurrentWake();
    r.levelsAtSleep = inputReadLevels();
    r.awakeMs = millis();
    r.encoderEdges = inputEdgeCount();
    if (actionStarted) r.firstAckMs = clampMs(action.firstAckMs());
    r.slept = true;

    char line[200];
    rtcFormat(r, line, sizeof(line));
    netLog("%s", line);

    if (actionStarted) action.end();
    delay(20);  // let the last UDP packets leave before the radio powers down
    sleepNow();
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // never block when no USB host is listening

    rtcBegin();
    captureWake();
    const WakeRecord *previous = rtcPreviousWake();
    bool wokeByPress = previous && wakeIsPress(previous->levelsAtSleep, rtcCurrentWake().wakePins);
    inputBegin(wokeByPress);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);  // LED on while awake
    rtcDumpToSerial();

    // A weak battery browns out on the Wi-Fi current spike; retrying would loop until it dies
    if (rtcCurrentWake().resetReason == ESP_RST_BROWNOUT) {
        Serial.println("brownout: sleeping without Wi-Fi until touched");
        finishAndSleep();
    }

    if (!netConnect(WIFI_BUDGET_MS)) {
        Serial.println("wifi failed");
        finishAndSleep();
    }
    rtcCurrentWake().wifiMs = clampMs(millis());

    if (previous) {
        char line[200];
        rtcFormat(*previous, line, sizeof(line));
        netLog("%s", line);
    }

    action.begin(millis());
    actionStarted = true;
    if (wokeByPress) action.onPress();
}

void loop() {
    int clicks = inputTakeClicks();
    if (clicks) action.onClicks(clicks);
    if (inputTakePress()) action.onPress();
    action.loop(millis());

    if (millis() - inputLastActivityMs() > AWAKE_MS) finishAndSleep();
    delay(2);
}
