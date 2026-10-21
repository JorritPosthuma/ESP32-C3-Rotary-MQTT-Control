#include "wake_masks.h"

WakeMasks computeWakeMasks(uint8_t levels, const uint8_t pins[3]) {
    WakeMasks masks = {0, 0};
    for (int i = 0; i < 3; i++) {
        uint64_t bit = 1ULL << pins[i];
        if (levels & (1 << i)) {
            masks.onLow |= bit;
        } else {
            masks.onHigh |= bit;
        }
    }
    return masks;
}

bool wakeIsPress(uint8_t levelsAtSleep, uint8_t wakePins) {
    return (wakePins & LEVEL_BUTTON) && (levelsAtSleep & LEVEL_BUTTON);
}
