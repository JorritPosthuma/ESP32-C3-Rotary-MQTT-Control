#pragma once

#include <stdint.h>

// Encoder and button input. Edges are captured by interrupts from the start of setup, so
// input made while Wi-Fi connects is not lost.
void inputBegin(bool wokeByPress);
int inputTakeClicks();  // signed detents since the last call, ENCODER_REVERSE applied
bool inputTakePress();  // true once per debounced press
uint32_t inputLastActivityMs();
uint32_t inputEdgeCount();
uint8_t inputReadLevels();  // LEVEL_* bits, 1 = HIGH
