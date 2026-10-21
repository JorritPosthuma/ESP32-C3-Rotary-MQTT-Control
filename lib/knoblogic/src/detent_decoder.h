#pragma once

#include <stdint.h>

// Decodes a rotary encoder whose detents are one full quadrature cycle and rest at A=1 B=1.
class DetentDecoder {
   public:
    // Feed the current levels (1 = HIGH). Returns +1 or -1 when a detent completes, else 0.
    // +1 is the sequence (A,B) 11 -> 01 -> 00 -> 10 -> 11.
    int8_t update(uint8_t a, uint8_t b);

   private:
    uint8_t state_ = 3;
    int8_t steps_ = 0;
};
