#include "volume_controller.h"

static int clampVolume(int volume) {
    return volume < 0 ? 0 : (volume > 100 ? 100 : volume);
}

VolumeController::VolumeController(int step, uint32_t minIntervalMs, uint32_t echoWindowMs)
    : step_(step), minIntervalMs_(minIntervalMs), echoWindowMs_(echoWindowMs) {}

void VolumeController::addClicks(int clicks) {
    if (known_) {
        target_ = clampVolume(target_ + clicks * step_);
    } else {
        pendingClicks_ += clicks;
    }
}

bool VolumeController::isOwnEcho(int volume, uint32_t nowMs) const {
    for (int i = 0; i < RECENT; i++) {
        if (recentValue_[i] == volume && nowMs - recentAt_[i] < echoWindowMs_) return true;
    }
    return false;
}

void VolumeController::onHubVolume(int volume, uint32_t nowMs) {
    if (!known_) {
        known_ = true;
        lastSent_ = volume;  // the hub is already there, no need to send it
        target_ = clampVolume(volume + pendingClicks_ * step_);
        pendingClicks_ = 0;
        return;
    }
    if (isOwnEcho(volume, nowMs)) return;

    // Clamped by the hub or changed by another client: adopt it, keeping clicks still in flight
    target_ = clampVolume(volume + (target_ - lastSent_));
    lastSent_ = volume;
}

int VolumeController::takeSet(uint32_t nowMs) {
    if (!known_ || target_ == lastSent_) return -1;
    if (sentAny_ && nowMs - lastSentAt_ < minIntervalMs_) return -1;

    lastSent_ = target_;
    lastSentAt_ = nowMs;
    sentAny_ = true;
    recentValue_[recentNext_] = target_;
    recentAt_[recentNext_] = nowMs;
    recentNext_ = (recentNext_ + 1) % RECENT;
    return target_;
}
