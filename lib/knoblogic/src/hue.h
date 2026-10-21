#pragma once

#include <stddef.h>

// Philips Hue bridge, local v1 REST API over plain HTTP.
// Builders return the length written, or 0 with out[0] = 0 if it does not fit.
namespace hue {

size_t groupPath(char *out, size_t cap, const char *key, int group);        // /api/<key>/groups/<id>
size_t groupActionPath(char *out, size_t cap, const char *key, int group);  // .../action

// Relative brightness change, clamped to -254..254. Turning up also switches the group on.
size_t dimBody(char *out, size_t cap, int briInc, int transitionDs);
size_t onBody(char *out, size_t cap, bool on);

// HTTP/1.1 request that asks the bridge to close afterwards. body may be nullptr (no Content-Type/Content-Length).
size_t buildRequest(char *out, size_t cap, const char *method, const char *host, const char *path, const char *body);

// Reads "any_on" from a group GET reply. False if it is missing (for example an error reply).
bool parseAnyOn(const char *json, bool *anyOn);
bool isHueError(const char *body);

}  // namespace hue
