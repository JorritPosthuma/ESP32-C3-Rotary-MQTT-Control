#include <unity.h>

#include "volume_controller.h"

void setUp() {}
void tearDown() {}

void test_clicks_before_known_applied() {
    VolumeController v(1);
    v.addClicks(3);
    TEST_ASSERT_EQUAL(-1, v.takeSet(0));
    v.onHubVolume(36, 10);
    TEST_ASSERT_EQUAL(39, v.takeSet(10));
}

void test_step_multiplies() {
    VolumeController v(5);
    v.onHubVolume(20, 0);
    v.addClicks(2);
    TEST_ASSERT_EQUAL(30, v.takeSet(0));
}

void test_clamp_low_high() {
    VolumeController v(1);
    v.onHubVolume(2, 0);
    v.addClicks(-5);
    TEST_ASSERT_EQUAL(0, v.takeSet(0));
    v.onHubVolume(99, 1000);
    v.addClicks(5);
    TEST_ASSERT_EQUAL(100, v.takeSet(1000));
}

void test_rate_limit_coalesces() {
    VolumeController v(1);
    v.onHubVolume(30, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(31, v.takeSet(0));
    v.addClicks(1);
    TEST_ASSERT_EQUAL(-1, v.takeSet(50));
    v.addClicks(1);
    TEST_ASSERT_EQUAL(33, v.takeSet(100));
}

void test_never_resend_same() {
    VolumeController v(1);
    v.onHubVolume(30, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(31, v.takeSet(0));
    TEST_ASSERT_EQUAL(-1, v.takeSet(200));
}

void test_own_echo_ignored() {
    VolumeController v(1);
    v.onHubVolume(30, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(31, v.takeSet(0));
    v.addClicks(1);
    TEST_ASSERT_EQUAL(32, v.takeSet(100));
    v.onHubVolume(31, 130);
    TEST_ASSERT_EQUAL(32, v.target());
    TEST_ASSERT_EQUAL(-1, v.takeSet(200));
}

void test_hub_clamp_adopted() {
    VolumeController v(1);
    v.onHubVolume(50, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(51, v.takeSet(0));
    v.onHubVolume(50, 30);
    TEST_ASSERT_EQUAL(50, v.target());
    TEST_ASSERT_EQUAL(-1, v.takeSet(200));
    v.addClicks(-1);
    TEST_ASSERT_EQUAL(49, v.takeSet(300));
}

void test_clicks_in_flight_kept_on_adopt() {
    VolumeController v(1);
    v.onHubVolume(50, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(51, v.takeSet(0));
    v.addClicks(1);
    v.onHubVolume(50, 30);
    TEST_ASSERT_EQUAL(51, v.target());
    TEST_ASSERT_EQUAL(51, v.takeSet(100));
}

void test_external_change_resets_last_sent() {
    VolumeController v(1);
    v.onHubVolume(30, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(31, v.takeSet(0));
    v.onHubVolume(31, 20);
    v.onHubVolume(20, 1000);
    v.addClicks(11);
    TEST_ASSERT_EQUAL(31, v.takeSet(1100));
}

void test_fast_spin_coalesces() {
    VolumeController v(1);
    v.onHubVolume(10, 0);
    v.addClicks(1);
    TEST_ASSERT_EQUAL(11, v.takeSet(0));
    for (int i = 0; i < 19; i++) v.addClicks(1);
    TEST_ASSERT_EQUAL(-1, v.takeSet(50));
    TEST_ASSERT_EQUAL(30, v.takeSet(100));
    TEST_ASSERT_EQUAL(-1, v.takeSet(200));
}

void test_set_max_clamps() {
    VolumeController v(1);
    v.setMax(50);
    v.onHubVolume(49, 0);
    v.addClicks(3);
    TEST_ASSERT_EQUAL(50, v.takeSet(0));
    v.addClicks(5);
    TEST_ASSERT_EQUAL(-1, v.takeSet(200));
    v.addClicks(-1);
    TEST_ASSERT_EQUAL(49, v.takeSet(300));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_set_max_clamps);
    RUN_TEST(test_clicks_before_known_applied);
    RUN_TEST(test_step_multiplies);
    RUN_TEST(test_clamp_low_high);
    RUN_TEST(test_rate_limit_coalesces);
    RUN_TEST(test_never_resend_same);
    RUN_TEST(test_own_echo_ignored);
    RUN_TEST(test_hub_clamp_adopted);
    RUN_TEST(test_clicks_in_flight_kept_on_adopt);
    RUN_TEST(test_external_change_resets_last_sent);
    RUN_TEST(test_fast_spin_coalesces);
    return UNITY_END();
}
