#include <unity.h>

#include "luci_session.h"

void setUp() {}
void tearDown() {}

using Send = LuciSession::Send;

void test_registers_first_and_retries() {
    LuciSession s(500);
    s.onConnected();
    TEST_ASSERT_EQUAL(Send::Register, s.poll(0));
    TEST_ASSERT_EQUAL(Send::None, s.poll(100));
    TEST_ASSERT_EQUAL(Send::Register, s.poll(500));
}

void test_queries_volume_only_after_register_ack() {
    LuciSession s(500);
    s.onConnected();
    s.poll(0);
    s.onMessage(3, 1);
    TEST_ASSERT_EQUAL(Send::GetVolume, s.poll(50));
    TEST_ASSERT_EQUAL(Send::None, s.poll(100));
    TEST_ASSERT_EQUAL(Send::GetVolume, s.poll(550));
}

void test_ready_after_volume() {
    LuciSession s(500);
    s.onConnected();
    s.poll(0);
    s.onMessage(3, 1);
    s.poll(50);
    TEST_ASSERT_FALSE(s.ready());
    s.onMessage(64, 1);
    TEST_ASSERT_TRUE(s.ready());
    TEST_ASSERT_EQUAL(Send::None, s.poll(2000));
}

void test_failed_register_is_retried() {
    LuciSession s(500);
    s.onConnected();
    s.poll(0);
    s.onMessage(3, 2);
    TEST_ASSERT_EQUAL(Send::None, s.poll(100));
    TEST_ASSERT_EQUAL(Send::Register, s.poll(500));
}

void test_not_registered_reregisters_without_query() {
    LuciSession s(500);
    s.onConnected();
    s.poll(0);
    s.onMessage(3, 1);
    s.poll(50);
    s.onMessage(64, 1);
    s.onMessage(0, 0);
    TEST_ASSERT_EQUAL(Send::Register, s.poll(3000));
    s.onMessage(3, 1);
    TEST_ASSERT_EQUAL(Send::None, s.poll(3050));
    TEST_ASSERT_TRUE(s.ready());
}

void test_reconnect_starts_over() {
    LuciSession s(500);
    s.onConnected();
    s.poll(0);
    s.onMessage(3, 1);
    s.poll(50);
    s.onMessage(64, 1);
    s.onConnected();
    TEST_ASSERT_FALSE(s.ready());
    TEST_ASSERT_EQUAL(Send::Register, s.poll(5000));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_registers_first_and_retries);
    RUN_TEST(test_queries_volume_only_after_register_ack);
    RUN_TEST(test_ready_after_volume);
    RUN_TEST(test_failed_register_is_retried);
    RUN_TEST(test_not_registered_reregisters_without_query);
    RUN_TEST(test_reconnect_starts_over);
    return UNITY_END();
}
