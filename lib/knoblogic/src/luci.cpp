#include "luci.h"

#include <string.h>

namespace luci {

size_t encode(uint8_t type, uint16_t box, const char *payload, uint8_t *out, size_t cap) {
    size_t length = payload ? strlen(payload) : 0;
    if (HEADER_SIZE + length > cap || length > 0xFFFF) return 0;

    const uint8_t header[HEADER_SIZE] = {
        0, 0, type, (uint8_t)(box & 0xFF), (uint8_t)(box >> 8), 0, 0, 0, (uint8_t)(length & 0xFF), (uint8_t)(length >> 8),
    };
    memcpy(out, header, HEADER_SIZE);
    if (length) memcpy(out + HEADER_SIZE, payload, length);
    return HEADER_SIZE + length;
}

void Decoder::push(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (skip_) {
            skip_--;
            continue;
        }
        if (len_ == sizeof(buf_)) return;  // only when the caller stops draining; the rest is lost
        buf_[len_++] = data[i];

        size_t received = len_ - frameStart_;
        if (received < HEADER_SIZE) continue;
        size_t length = lengthAt(frameStart_);
        if (length > MAX_PAYLOAD) {
            len_ = frameStart_;  // drop the header and skip the payload as it arrives
            skip_ = length;
        } else if (received == HEADER_SIZE + length) {
            frameStart_ = len_;  // frame complete
        }
    }
}

bool Decoder::next(Message &out) {
    if (frameStart_ == 0) return false;

    size_t length = lengthAt(0);
    size_t frame = HEADER_SIZE + length;
    out.box = (uint16_t)(buf_[3] << 8 | buf_[4]);
    out.type = buf_[2];
    out.status = buf_[5];
    out.length = (uint16_t)length;
    memcpy(out.payload, buf_ + HEADER_SIZE, length);
    out.payload[length] = 0;

    memmove(buf_, buf_ + frame, len_ - frame);
    len_ -= frame;
    frameStart_ -= frame;
    return true;
}

bool parseVolume(const Message &m, int *volume) {
    if (m.box != MB_VOLUME || m.length == 0) return false;

    int value = 0;
    bool digits = false;
    for (const char *p = m.payload; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            value = value * 10 + (*p - '0');
            digits = true;
        } else if (*p != '\r' && *p != '\n' && *p != ' ') {
            return false;
        }
    }
    if (!digits) return false;
    *volume = value;
    return true;
}

}  // namespace luci
