#include "wake_record.h"

#include <stdio.h>
#include <string.h>

#include "line_protocol.h"
#include "wake_masks.h"

static void pinsToString(uint8_t pins, char *out, size_t cap) {
    snprintf(out, cap, "%s%s%s", (pins & LEVEL_A) ? "A " : "", (pins & LEVEL_B) ? "B " : "",
             (pins & LEVEL_BUTTON) ? "BTN " : "");
    size_t len = strlen(out);
    if (len == 0) {
        snprintf(out, cap, "-");
    } else {
        out[len - 1] = 0;  // drop the trailing space
    }
}

static void levelsToString(uint8_t levels, char *out, size_t cap) {
    snprintf(out, cap, "A=%d B=%d BTN=%d", (levels & LEVEL_A) ? 1 : 0, (levels & LEVEL_B) ? 1 : 0,
             (levels & LEVEL_BUTTON) ? 1 : 0);
}

static void msToString(uint16_t ms, char *out, size_t cap) {
    if (ms == 0) {
        snprintf(out, cap, "-");
    } else {
        snprintf(out, cap, "%ums", (unsigned)ms);
    }
}

size_t formatWakeRecord(const WakeRecord &r, const char *resetName, const char *wakeName, char *out, size_t cap) {
    char by[16], boot[24];
    pinsToString(r.wakePins, by, sizeof(by));
    levelsToString(r.levelsAtBoot, boot, sizeof(boot));

    if (!r.slept) {
        return formatFit(out, cap, "#%lu reset=%s wake=%s by=%s | boot: %s | did not reach deep sleep",
                         (unsigned long)r.boot, resetName, wakeName, by, boot);
    }

    char sleep[24], wifi[12], ack[12];
    levelsToString(r.levelsAtSleep, sleep, sizeof(sleep));
    msToString(r.wifiMs, wifi, sizeof(wifi));
    msToString(r.firstAckMs, ack, sizeof(ack));
    return formatFit(out, cap, "#%lu reset=%s wake=%s by=%s | boot: %s | sleep: %s | awake=%.1fs edges=%lu wifi=%s ack=%s",
                     (unsigned long)r.boot, resetName, wakeName, by, boot, sleep, r.awakeMs / 1000.0,
                     (unsigned long)r.encoderEdges, wifi, ack);
}
