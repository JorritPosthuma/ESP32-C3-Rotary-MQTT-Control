#pragma once

#include <stddef.h>
#include <stdint.h>

// LUCI (Libre Unified Control Interface) over TCP 7777, as spoken by the Buchardt hub.
// Requests use little-endian message box and length; replies use big-endian.
// Reference: BuchardtBar docs/protocol.md §3 and §12.
namespace luci {

constexpr uint16_t MB_NOT_REGISTERED = 0;
constexpr uint16_t MB_REGISTER = 3;
constexpr uint16_t MB_DEREGISTER = 4;
constexpr uint16_t MB_VOLUME = 64;

constexpr uint8_t GET = 1;
constexpr uint8_t SET = 2;

constexpr size_t HEADER_SIZE = 10;
constexpr size_t MAX_PAYLOAD = 63;

// Builds a request frame. Returns its length, or 0 if it does not fit in cap.
size_t encode(uint8_t type, uint16_t box, const char *payload, uint8_t *out, size_t cap);

struct Message {
    uint16_t box;
    uint8_t type;
    uint8_t status;  // 1 = success
    uint16_t length;
    char payload[MAX_PAYLOAD + 1];  // NUL-terminated
};

// Reassembles reply frames from a TCP byte stream. Frames with a payload longer than
// MAX_PAYLOAD (now-playing JSON and the like) are dropped without losing sync.
class Decoder {
   public:
    void push(const uint8_t *data, size_t len);
    bool next(Message &out);

   private:
    uint8_t buf_[512];
    size_t len_ = 0;
    size_t skip_ = 0;  // bytes of a dropped frame still to discard from the stream

    void consume(size_t n);
};

// True for an MB 64 message with an ASCII integer payload.
bool parseVolume(const Message &m, int *volume);

}  // namespace luci
