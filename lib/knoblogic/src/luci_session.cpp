#include "luci_session.h"

#include "luci.h"

void LuciSession::onConnected() {
    registered_ = false;
    volumeKnown_ = false;
    waitingFor_ = Send::None;
}

void LuciSession::onMessage(uint16_t box, uint8_t status) {
    if (box == luci::MB_REGISTER && waitingFor_ == Send::Register) {
        registered_ = status == 1;
        if (registered_) waitingFor_ = Send::None;  // a failed REGISTER waits for its retry
    } else if (box == luci::MB_VOLUME) {
        volumeKnown_ = true;
        if (waitingFor_ == Send::GetVolume) waitingFor_ = Send::None;
    } else if (box == luci::MB_NOT_REGISTERED) {
        registered_ = false;
        waitingFor_ = Send::None;
    }
}

LuciSession::Send LuciSession::poll(uint32_t nowMs) {
    Send next = !registered_ ? Send::Register : (!volumeKnown_ ? Send::GetVolume : Send::None);
    if (next == Send::None) return Send::None;
    if (waitingFor_ == next && nowMs - sentAtMs_ < retryMs_) return Send::None;

    waitingFor_ = next;
    sentAtMs_ = nowMs;
    return next;
}
