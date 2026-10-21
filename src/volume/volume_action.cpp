// Volume knob: Buchardt hub volume over LUCI (TCP 7777); a press broadcasts play/pause to BuchardtBar.
#include <Arduino.h>
#include <WiFi.h>

#include "../core/action.h"
#include "../core/net.h"
#include "../core/rtc_state.h"
#include "config.h"
#include "line_protocol.h"
#include "luci.h"
#include "luci_session.h"
#include "volume_controller.h"

constexpr uint16_t HUB_PORT = 7777;
constexpr int32_t HUB_CONNECT_TIMEOUT_MS = 200;  // a LAN SYN-ACK takes < 10 ms; keeps a dead hub from stalling input
constexpr uint32_t HUB_RETRY_MS = 2000;
constexpr int PLAYPAUSE_REPEATS = 3;
constexpr uint32_t PLAYPAUSE_GAP_MS = 30;

class VolumeAction : public Action {
   public:
    void begin(uint32_t nowMs) override {
        hub_.fromString(HUB_IP);
        strlcpy(ownIp_, WiFi.localIP().toString().c_str(), sizeof(ownIp_));
        volume_.setMax(VOLUME_MAX);
        // Connect from loop() so a wake press goes out before any hub I/O
        lastAttemptMs_ = nowMs - HUB_RETRY_MS;
    }

    void onClicks(int clicks) override { volume_.addClicks(clicks); }

    void onPress() override {
        uint32_t seq = rtcNextPlayPauseSeq();
        char line[48];
        formatPlayPause(line, sizeof(line), seq);
        for (int i = 0; i < PLAYPAUSE_REPEATS; i++) {
            if (i) delay(PLAYPAUSE_GAP_MS);
            netBroadcastLine(line);
        }
        netLog("playpause %lu", (unsigned long)seq);
    }

    void loop(uint32_t nowMs) override {
        if (!client_.connected()) {
            if (nowMs - lastAttemptMs_ >= HUB_RETRY_MS) connect(nowMs);
            return;
        }

        uint8_t buf[128];
        while (client_.available() > 0) {
            int n = client_.read(buf, sizeof(buf));
            if (n <= 0) break;
            decoder_.push(buf, n);
        }
        luci::Message message;
        while (decoder_.next(message)) handle(message);

        switch (session_.poll(millis())) {
            case LuciSession::Send::Register: send(luci::SET, luci::MB_REGISTER, ownIp_); break;
            case LuciSession::Send::GetVolume: send(luci::GET, luci::MB_VOLUME, ""); break;
            case LuciSession::Send::None: break;
        }

        int set = volume_.takeSet(millis());
        if (set >= 0) {
            char value[8];
            snprintf(value, sizeof(value), "%d", set);
            send(luci::SET, luci::MB_VOLUME, value);
            if (!sentAny_) firstSetMs_ = millis();
            sentAny_ = true;
        }
    }

    uint32_t firstAckMs() const override { return firstAckMs_; }

    void end() override {
        if (!client_.connected()) return;
        send(luci::SET, luci::MB_DEREGISTER, ownIp_);
        client_.stop();  // lwIP sends the queued frame before the FIN
    }

   private:
    WiFiClient client_;
    IPAddress hub_;
    char ownIp_[16] = "";
    luci::Decoder decoder_;
    LuciSession session_;
    VolumeController volume_{VOLUME_STEP};
    uint32_t lastAttemptMs_ = 0;
    bool sentAny_ = false;
    uint32_t firstSetMs_ = 0;
    uint32_t firstAckMs_ = 0;

    void connect(uint32_t nowMs) {
        lastAttemptMs_ = nowMs;
        if (!client_.connect(hub_, HUB_PORT, HUB_CONNECT_TIMEOUT_MS)) {
            netLog("hub connect failed");
            return;
        }
        client_.setNoDelay(true);
        decoder_ = luci::Decoder();
        session_.onConnected();
    }

    void send(uint8_t type, uint16_t box, const char *payload) {
        uint8_t frame[32];
        size_t length = luci::encode(type, box, payload, frame, sizeof(frame));
        if (length) client_.write(frame, length);
    }

    void handle(const luci::Message &message) {
        session_.onMessage(message.box, message.status);
        int volume;
        if (luci::parseVolume(message, &volume)) {
            if (sentAny_ && firstAckMs_ == 0) {
                firstAckMs_ = millis();
                netLog("first set at %lums, acked after %lums", (unsigned long)firstSetMs_,
                       (unsigned long)(firstAckMs_ - firstSetMs_));
            }
            volume_.onHubVolume(volume, millis());
        }
    }
};

Action &makeAction() {
    static VolumeAction action;
    return action;
}
