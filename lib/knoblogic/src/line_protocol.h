#pragma once

#include <stddef.h>
#include <stdint.h>

// One-line ASCII messages broadcast on UDP 47777 (agreed with BuchardtBar).
// Both return the length written, or 0 with out[0] = 0 if the line does not fit.

// "VOLKNOB/1 PLAYPAUSE <seq>"
size_t formatPlayPause(char *out, size_t cap, uint32_t seq);

// "VOLKNOB/1 LOG <knobName> <text>"
size_t formatLog(char *out, size_t cap, const char *knobName, const char *text);

// Shared helper: snprintf that returns 0 and an empty string when the result is truncated
size_t formatFit(char *out, size_t cap, const char *fmt, ...);
