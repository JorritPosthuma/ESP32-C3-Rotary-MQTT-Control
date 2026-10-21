#include "hue.h"

#include <string.h>

#include "line_protocol.h"

namespace hue {

size_t groupPath(char *out, size_t cap, const char *key, int group) {
    return formatFit(out, cap, "/api/%s/groups/%d", key, group);
}

size_t groupActionPath(char *out, size_t cap, const char *key, int group) {
    return formatFit(out, cap, "/api/%s/groups/%d/action", key, group);
}

size_t dimBody(char *out, size_t cap, int briInc, int transitionDs) {
    if (briInc > 254) briInc = 254;
    if (briInc < -254) briInc = -254;
    return formatFit(out, cap, "{%s\"bri_inc\":%d,\"transitiontime\":%d}", briInc > 0 ? "\"on\":true," : "", briInc,
                     transitionDs);
}

size_t onBody(char *out, size_t cap, bool on) {
    return formatFit(out, cap, "{\"on\":%s}", on ? "true" : "false");
}

size_t buildRequest(char *out, size_t cap, const char *method, const char *host, const char *path, const char *body) {
    if (!body) {
        return formatFit(out, cap, "%s %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", method, path, host);
    }
    return formatFit(out, cap,
                     "%s %s HTTP/1.1\r\nHost: %s\r\nContent-Type: application/json\r\nContent-Length: %u\r\n"
                     "Connection: close\r\n\r\n%s",
                     method, path, host, (unsigned)strlen(body), body);
}

bool parseAnyOn(const char *json, bool *anyOn) {
    const char *p = strstr(json, "\"any_on\"");
    if (!p) return false;
    p += strlen("\"any_on\"");
    while (*p == ' ' || *p == ':') p++;
    if (strncmp(p, "true", 4) == 0) {
        *anyOn = true;
        return true;
    }
    if (strncmp(p, "false", 5) == 0) {
        *anyOn = false;
        return true;
    }
    return false;
}

bool isHueError(const char *body) {
    return strstr(body, "\"error\"") != nullptr;
}

}  // namespace hue
