#include "line_protocol.h"

#include <stdarg.h>
#include <stdio.h>

size_t formatFit(char *out, size_t cap, const char *fmt, ...) {
    if (cap == 0) return 0;
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(out, cap, fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= cap) {
        out[0] = 0;
        return 0;
    }
    return (size_t)n;
}

size_t formatPlayPause(char *out, size_t cap, uint32_t seq) {
    return formatFit(out, cap, "VOLKNOB/1 PLAYPAUSE %lu", (unsigned long)seq);
}

size_t formatLog(char *out, size_t cap, const char *knobName, const char *text) {
    return formatFit(out, cap, "VOLKNOB/1 LOG %s %s", knobName, text);
}
