#include "sleep.h"

#include <Arduino.h>
#include <esp_sleep.h>

#include "input.h"
#include "pins.h"
#include "wake_masks.h"

void sleepNow() {
    static const uint8_t pins[3] = {PIN_A, PIN_B, PIN_BUTTON};
    WakeMasks masks = computeWakeMasks(inputReadLevels(), pins);

    digitalWrite(PIN_LED, HIGH);  // LED off
    if (masks.onLow) esp_deep_sleep_enable_gpio_wakeup(masks.onLow, ESP_GPIO_WAKEUP_GPIO_LOW);
    if (masks.onHigh) esp_deep_sleep_enable_gpio_wakeup(masks.onHigh, ESP_GPIO_WAKEUP_GPIO_HIGH);
    Serial.flush();
    esp_deep_sleep_start();
}
