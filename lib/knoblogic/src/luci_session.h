#pragma once

#include <stdint.h>

// LUCI handshake: REGISTER first, query the volume only after the hub acknowledged it, and
// retry either step without an answer. The hub drops a GET sent right behind a REGISTER,
// which is also why the Android app waits for the MB 3 reply.
class LuciSession {
   public:
    enum class Send { None, Register, GetVolume };

    explicit LuciSession(uint32_t retryMs = 500) : retryMs_(retryMs) {}

    void onConnected();                          // new TCP connection: start over
    void onMessage(uint16_t box, uint8_t status);  // every decoded reply or notification
    Send poll(uint32_t nowMs);                   // what to send now, if anything
    bool ready() const { return registered_ && volumeKnown_; }

   private:
    uint32_t retryMs_;
    bool registered_ = false;
    bool volumeKnown_ = false;
    Send waitingFor_ = Send::None;  // request in flight
    uint32_t sentAtMs_ = 0;
};
