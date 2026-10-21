#include <string.h>
#include <unity.h>

#include "luci.h"

void setUp() {}
void tearDown() {}

static const uint8_t VOLUME_REPLY_36[] = {0x00, 0x00, 0x01, 0x00, 0x40, 0x01, 0xef, 0xa1, 0x00, 0x02, '3', '6'};
static const uint8_t REGISTER_ACK[] = {0x00, 0x00, 0x02, 0x00, 0x03, 0x01, 0x27, 0xac, 0x00, 0x00};

void test_encode_volume_set_30() {
    uint8_t out[32];
    const uint8_t expected[] = {0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x33, 0x30};
    TEST_ASSERT_EQUAL(sizeof(expected), luci::encode(luci::SET, luci::MB_VOLUME, "30", out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_encode_volume_get() {
    uint8_t out[32];
    const uint8_t expected[] = {0x00, 0x00, 0x01, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    TEST_ASSERT_EQUAL(sizeof(expected), luci::encode(luci::GET, luci::MB_VOLUME, "", out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
}

void test_encode_register() {
    uint8_t out[32];
    const uint8_t header[] = {0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00};
    TEST_ASSERT_EQUAL(22, luci::encode(luci::SET, luci::MB_REGISTER, "192.168.1.50", out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(header, out, sizeof(header));
    TEST_ASSERT_EQUAL_MEMORY("192.168.1.50", out + 10, 12);
}

void test_encode_too_small_is_zero() {
    uint8_t out[11];
    TEST_ASSERT_EQUAL(0, luci::encode(luci::SET, luci::MB_VOLUME, "30", out, sizeof(out)));
}

void test_decode_volume_reply() {
    luci::Decoder d;
    luci::Message m;
    d.push(VOLUME_REPLY_36, sizeof(VOLUME_REPLY_36));
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(64, m.box);
    TEST_ASSERT_EQUAL(1, m.type);
    TEST_ASSERT_EQUAL(1, m.status);
    TEST_ASSERT_EQUAL_STRING("36", m.payload);
    int volume = -1;
    TEST_ASSERT_TRUE(luci::parseVolume(m, &volume));
    TEST_ASSERT_EQUAL(36, volume);
    TEST_ASSERT_FALSE(d.next(m));
}

void test_decode_register_ack() {
    luci::Decoder d;
    luci::Message m;
    d.push(REGISTER_ACK, sizeof(REGISTER_ACK));
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(3, m.box);
    TEST_ASSERT_EQUAL(1, m.status);
    TEST_ASSERT_EQUAL(0, m.length);
}

void test_decode_split_across_pushes() {
    luci::Decoder d;
    luci::Message m;
    d.push(VOLUME_REPLY_36, 3);
    TEST_ASSERT_FALSE(d.next(m));
    d.push(VOLUME_REPLY_36 + 3, sizeof(VOLUME_REPLY_36) - 3);
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL_STRING("36", m.payload);
}

void test_decode_two_frames_one_push() {
    uint8_t both[sizeof(REGISTER_ACK) + sizeof(VOLUME_REPLY_36)];
    memcpy(both, REGISTER_ACK, sizeof(REGISTER_ACK));
    memcpy(both + sizeof(REGISTER_ACK), VOLUME_REPLY_36, sizeof(VOLUME_REPLY_36));
    luci::Decoder d;
    luci::Message m;
    d.push(both, sizeof(both));
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(3, m.box);
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(64, m.box);
    TEST_ASSERT_FALSE(d.next(m));
}

// MB 42 now-playing JSON (357 bytes) followed by a volume reply
static size_t buildOversized(uint8_t *out) {
    const uint8_t header[] = {0x00, 0x00, 0x01, 0x00, 0x2a, 0x01, 0x15, 0x9b, 0x01, 0x65};
    memcpy(out, header, sizeof(header));
    memset(out + sizeof(header), 'x', 0x165);
    memcpy(out + sizeof(header) + 0x165, VOLUME_REPLY_36, sizeof(VOLUME_REPLY_36));
    return sizeof(header) + 0x165 + sizeof(VOLUME_REPLY_36);
}

void test_oversized_frame_skipped() {
    uint8_t stream[400];
    size_t len = buildOversized(stream);
    luci::Decoder d;
    luci::Message m;
    d.push(stream, len);
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(64, m.box);
    TEST_ASSERT_EQUAL_STRING("36", m.payload);
    TEST_ASSERT_FALSE(d.next(m));
}

void test_oversized_frame_split_skipped() {
    uint8_t stream[400];
    size_t len = buildOversized(stream);
    luci::Decoder d;
    luci::Message m;
    d.push(stream, 110);
    TEST_ASSERT_FALSE(d.next(m));
    d.push(stream + 110, len - 110);
    TEST_ASSERT_TRUE(d.next(m));
    TEST_ASSERT_EQUAL(64, m.box);
}

void test_parse_volume_rejects_other_box() {
    const uint8_t name[] = {0x00, 0x00, 0x01, 0x00, 0x5a, 0x01, 0x06, 0x09, 0x00, 0x0d,
                            'B', 'u', 'c', 'h', 'a', 'r', 'd', 't', ' ', 'A', '5', '0', '0'};
    luci::Decoder d;
    luci::Message m;
    d.push(name, sizeof(name));
    TEST_ASSERT_TRUE(d.next(m));
    int volume = -1;
    TEST_ASSERT_FALSE(luci::parseVolume(m, &volume));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_encode_volume_set_30);
    RUN_TEST(test_encode_volume_get);
    RUN_TEST(test_encode_register);
    RUN_TEST(test_encode_too_small_is_zero);
    RUN_TEST(test_decode_volume_reply);
    RUN_TEST(test_decode_register_ack);
    RUN_TEST(test_decode_split_across_pushes);
    RUN_TEST(test_decode_two_frames_one_push);
    RUN_TEST(test_oversized_frame_skipped);
    RUN_TEST(test_oversized_frame_split_skipped);
    RUN_TEST(test_parse_volume_rejects_other_box);
    return UNITY_END();
}
