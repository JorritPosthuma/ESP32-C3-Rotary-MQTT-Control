#pragma once

#include <stdint.h>

// Connects with a static IP and the cached BSSID/channel when available. False if not
// connected within budgetMs.
bool netConnect(uint32_t budgetMs);

// UDP broadcast to the subnet broadcast address, port 47777 (no-op without Wi-Fi)
void netBroadcastLine(const char *line);

// printf-style log line to Serial, also broadcast as "VOLKNOB/1 LOG <KNOB_NAME> <text>"
void netLog(const char *fmt, ...);
