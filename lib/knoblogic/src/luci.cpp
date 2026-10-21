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
    if (skip_) {
        size_t n = len < skip_ ? len : skip_;
        skip_ -= n;
        data += n;
        len -= n;
    }
    size_t room = sizeof(buf_) - len_;
    if (len > room) len = room;  // only when the caller stops draining; the excess is lost
    memcpy(buf_ + len_, data, len);
    len_ += len;
}

void Decoder::consume(size_t n) {
    memmove(buf_, buf_ + n, len_ - n);
    len_ -= n;
}

bool Decoder::next(Message &out) {
    while (len_ >= HEADER_SIZE) {
        size_t length = (size_t)buf_[8] << 8 | buf_[9];
        size_t frame = HEADER_SIZE + length;

        if (length > MAX_PAYLOAD) {
            size_t present = len_ < frame ? len_ : frame;
            skip_ = frame - present;
            consume(present);
            continue;
        }
        if (len_ < frame) return false;

        out.box = (uint16_t)(buf_[3] << 8 | buf_[4]);
        out.type = buf_[2];
        out.status = buf_[5];
        out.length = (uint16_t)length;
        memcpy(out.payload, buf_ + HEADER_SIZE, length);
        out.payload[length] = 0;
        consume(frame);
        return true;
    }
    return false;
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
