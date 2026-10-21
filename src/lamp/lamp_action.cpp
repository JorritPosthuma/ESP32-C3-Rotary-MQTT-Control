// Lamp knob: dims a Hue room through the bridge's local v1 API; a press toggles the room.
#include <Arduino.h>
#include <WiFi.h>

#include "../core/action.h"
#include "../core/net.h"
#include "config.h"
#include "hue.h"
#include "rate_coalescer.h"

constexpr uint16_t HUE_PORT = 80;
constexpr int32_t HUE_CONNECT_TIMEOUT_MS = 500;
constexpr uint32_t HUE_REPLY_TIMEOUT_MS = 1500;  // the bridge can be slow while it runs transitions

class LampAction : public Action {
   public:
    void begin(uint32_t) override {
        bridge_.fromString(HUE_BRIDGE_IP);
        hue::groupPath(groupPath_, sizeof(groupPath_), HUE_APP_KEY, HUE_GROUP_ID);
        hue::groupActionPath(actionPath_, sizeof(actionPath_), HUE_APP_KEY, HUE_GROUP_ID);
        if (request("GET", groupPath_, nullptr) != 200 || !hue::parseAnyOn(body_, &anyOn_)) {
            netLog("hue error: %.80s", body_);
        }
    }

    void onClicks(int clicks) override { dim_.add(clicks); }

    void onPress() override {
        anyOn_ = !anyOn_;
        char body[24];
        hue::onBody(body, sizeof(body), anyOn_);
        put(body);
    }

    void loop(uint32_t nowMs) override {
        int clicks = dim_.take(nowMs);
        if (clicks == 0) return;
        char body[64];
        hue::dimBody(body, sizeof(body), clicks * DIM_STEP, HUE_TRANSITION_DS);
        if (clicks > 0) anyOn_ = true;  // turning up switches the room on
        put(body);
    }

    uint32_t firstAckMs() const override { return firstAckMs_; }

    void end() override {}

   private:
    IPAddress bridge_;
    char groupPath_[96];
    char actionPath_[104];
    char response_[2048];
    const char *body_ = "";
    bool anyOn_ = false;
    uint32_t firstAckMs_ = 0;
    RateCoalescer dim_{HUE_MIN_INTERVAL_MS};

    void put(const char *body) {
        int status = request("PUT", actionPath_, body);
        if (status == 200 && !hue::isHueError(body_)) {
            if (firstAckMs_ == 0) firstAckMs_ = millis();
        } else {
            netLog("hue error %d: %.80s", status, body_);
        }
    }

    // One request per connection; the bridge closes after replying. Returns the HTTP status,
    // -2 if connecting failed or -1 without a valid reply, and points body_ at the response body.
    int request(const char *method, const char *path, const char *body) {
        body_ = "";
        char request[384];
        size_t length = hue::buildRequest(request, sizeof(request), method, HUE_BRIDGE_IP, path, body);
        if (length == 0) return -1;

        // A fresh client per request: right after the previous close, a reused client can
        // report "disconnected" before the new reply arrives
        WiFiClient client;
        if (!client.connect(bridge_, HUE_PORT, HUE_CONNECT_TIMEOUT_MS)) return -2;
        client.write((const uint8_t *)request, length);

        size_t received = 0;
        uint32_t start = millis();
        while (millis() - start < HUE_REPLY_TIMEOUT_MS && received < sizeof(response_) - 1) {
            int n = client.read((uint8_t *)response_ + received, sizeof(response_) - 1 - received);
            if (n > 0) {
                received += n;
            } else if (received > 0 && !client.connected()) {
                break;  // only trust "closed" once the reply has started
            } else {
                delay(1);
            }
        }
        client.stop();
        response_[received] = 0;

        int status = -1;
        if (sscanf(response_, "HTTP/1.%*d %d", &status) != 1) {
            netLog("hue no reply: %u bytes after %lums: %.40s", (unsigned)received, (unsigned long)(millis() - start),
                   response_);
            return -1;
        }
        const char *separator = strstr(response_, "\r\n\r\n");
        body_ = separator ? separator + 4 : "";
        return status;
    }
};

Action &makeAction() {
    static LampAction action;
    return action;
}
