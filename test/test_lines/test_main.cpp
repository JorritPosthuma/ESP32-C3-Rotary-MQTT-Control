#include <string.h>
#include <unity.h>

#include "line_protocol.h"
#include "wake_record.h"

void setUp() {}
void tearDown() {}

static char buf[160];

void test_playpause() {
    TEST_ASSERT_EQUAL(22, formatPlayPause(buf, 64, 42));
    TEST_ASSERT_EQUAL_STRING("VOLKNOB/1 PLAYPAUSE 42", buf);
}

void test_log() {
    formatLog(buf, 64, "volume", "boot");
    TEST_ASSERT_EQUAL_STRING("VOLKNOB/1 LOG volume boot", buf);
}

void test_too_small_is_empty() {
    TEST_ASSERT_EQUAL(0, formatPlayPause(buf, 10, 42));
    TEST_ASSERT_EQUAL_STRING("", buf);
}

void test_record_slept() {
    WakeRecord r = {};
    r.boot = 12;
    r.wakePins = 0b011;
    r.levelsAtBoot = 0b111;
    r.levelsAtSleep = 0b111;
    r.slept = true;
    r.awakeMs = 30100;
    r.encoderEdges = 4;
    r.wifiMs = 412;
    r.firstAckMs = 530;
    formatWakeRecord(r, "DEEPSLEEP", "GPIO", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING(
        "#12 reset=DEEPSLEEP wake=GPIO by=A B | boot: A=1 B=1 BTN=1 | sleep: A=1 B=1 BTN=1 | "
        "awake=30.1s edges=4 wifi=412ms ack=530ms",
        buf);
}

void test_record_not_slept() {
    WakeRecord r = {};
    r.boot = 3;
    r.wakePins = 0b100;
    r.levelsAtBoot = 0b011;
    formatWakeRecord(r, "PANIC", "NONE", buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("#3 reset=PANIC wake=NONE by=BTN | boot: A=1 B=1 BTN=0 | did not reach deep sleep",
                             buf);
}

void test_record_no_timings() {
    WakeRecord r = {};
    r.boot = 1;
    r.slept = true;
    formatWakeRecord(r, "USB", "NONE", buf, sizeof(buf));
    const char *suffix = "edges=0 wifi=- ack=-";
    size_t len = strlen(buf);
    TEST_ASSERT_TRUE(len >= strlen(suffix));
    TEST_ASSERT_EQUAL_STRING(suffix, buf + len - strlen(suffix));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_playpause);
    RUN_TEST(test_log);
    RUN_TEST(test_too_small_is_empty);
    RUN_TEST(test_record_slept);
    RUN_TEST(test_record_not_slept);
    RUN_TEST(test_record_no_timings);
    return UNITY_END();
}
