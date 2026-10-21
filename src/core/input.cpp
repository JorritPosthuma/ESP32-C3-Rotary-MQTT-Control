#include "input.h"

#include <Arduino.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>

#include "config.h"
#include "debouncer.h"
#include "detent_decoder.h"
#include "pins.h"
#include "wake_masks.h"

// Pin level snapshots from the ISR, drained in the main loop
struct Edge {
    uint32_t ms;
    uint8_t levels;
};

constexpr int RING_SIZE = 128;
static Edge ring[RING_SIZE];
static volatile uint8_t ringHead = 0;  // written by the ISR
static uint8_t ringTail = 0;
static volatile uint32_t edges = 0;
static uint8_t lastLevels = 0;

static DetentDecoder decoder;
static Debouncer button(30, false);
static int pendingClicks = 0;
static bool pendingPress = false;
static uint32_t lastActivityMs = 0;

static uint8_t IRAM_ATTR levelsFromRegister() {
    uint32_t in = REG_READ(GPIO_IN_REG);
    return ((in >> PIN_A) & 1) | (((in >> PIN_B) & 1) << 1) | (((in >> PIN_BUTTON) & 1) << 2);
}

static void IRAM_ATTR onEdge() {
    edges = edges + 1;
    uint8_t next = (ringHead + 1) % RING_SIZE;
    if (next == ringTail) return;  // full: drop rather than overwrite unread edges
    ring[ringHead] = {(uint32_t)(esp_timer_get_time() / 1000), levelsFromRegister()};
    ringHead = next;
}

static void feedButton(bool pressed, uint32_t ms) {
    int8_t change = button.update(pressed, ms);
    if (change == 0) return;
    lastActivityMs = ms;
    if (change > 0) pendingPress = true;
}

// Drains captured edges into the decoder and debouncer
static void drain() {
    while (ringTail != ringHead) {
        Edge e = ring[ringTail];
        ringTail = (ringTail + 1) % RING_SIZE;

        if ((e.levels ^ lastLevels) & (LEVEL_A | LEVEL_B)) {
            int8_t step = decoder.update(e.levels & LEVEL_A ? 1 : 0, e.levels & LEVEL_B ? 1 : 0);
            if (step) {
                pendingClicks += ENCODER_REVERSE ? -step : step;
                lastActivityMs = e.ms;
            }
        }
        feedButton(!(e.levels & LEVEL_BUTTON), e.ms);
        lastLevels = e.levels;
    }
    // Let a press that stopped bouncing become stable
    feedButton(digitalRead(PIN_BUTTON) == LOW, millis());
}

void inputBegin(bool wokeByPress) {
    for (uint8_t pin : {PIN_A, PIN_B, PIN_BUTTON}) {
        gpio_hold_dis((gpio_num_t)pin);  // release the deep-sleep wake configuration
        pinMode(pin, INPUT);
    }
    button = Debouncer(30, wokeByPress);
    lastLevels = levelsFromRegister();
    attachInterrupt(PIN_A, onEdge, CHANGE);
    attachInterrupt(PIN_B, onEdge, CHANGE);
    attachInterrupt(PIN_BUTTON, onEdge, CHANGE);
}

int inputTakeClicks() {
    drain();
    int clicks = pendingClicks;
    pendingClicks = 0;
    return clicks;
}

bool inputTakePress() {
    drain();
    bool press = pendingPress;
    pendingPress = false;
    return press;
}

uint32_t inputLastActivityMs() {
    return lastActivityMs;
}

uint32_t inputEdgeCount() {
    return edges;
}

uint8_t inputReadLevels() {
    return digitalRead(PIN_A) | (digitalRead(PIN_B) << 1) | (digitalRead(PIN_BUTTON) << 2);
}
