#include "detent_decoder.h"

// Quadrature step for (previous << 2 | current), state = A << 1 | B.
// Forward is 3 -> 1 -> 0 -> 2 -> 3; double-bit jumps are invalid and count as 0.
static const int8_t STEP[16] = {
    0, -1, +1, 0,   // from 0: to 1 is back, to 2 is forward
    +1, 0, 0, -1,   // from 1: to 0 is forward, to 3 is back
    -1, 0, 0, +1,   // from 2: to 0 is back, to 3 is forward
    0, +1, -1, 0,   // from 3: to 1 is forward, to 2 is back
};

int8_t DetentDecoder::update(uint8_t a, uint8_t b) {
    uint8_t state = (a ? 2 : 0) | (b ? 1 : 0);
    if (state == state_) return 0;

    steps_ += STEP[(state_ << 2) | state];
    state_ = state;
    if (state != 3) return 0;

    // Back at rest: a full detent is 4 steps; accept 2+ so one missed edge still counts,
    // while bounces and half turns cancel out to 0
    int8_t result = steps_ >= 2 ? 1 : (steps_ <= -2 ? -1 : 0);
    steps_ = 0;
    return result;
}
