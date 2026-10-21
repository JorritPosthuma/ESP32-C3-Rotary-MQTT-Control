#include <unity.h>

#include "debouncer.h"
#include "detent_decoder.h"
#include "wake_masks.h"

void setUp() {}
void tearDown() {}

// Feeds states written as two-digit AB values (11 = A high, B high) and sums the result
static int feed(const int *states, int count) {
    DetentDecoder decoder;
    int sum = 0;
    for (int i = 0; i < count; i++) {
        sum += decoder.update(states[i] / 10, states[i] % 10);
    }
    return sum;
}

#define FEED(...)                                     \
    ([] {                                             \
        const int s[] = {__VA_ARGS__};                \
        return feed(s, sizeof(s) / sizeof(s[0]));     \
    }())

void test_cw_detent_is_plus_one() { TEST_ASSERT_EQUAL(1, FEED(11, 1, 0, 10, 11)); }
void test_ccw_detent_is_minus_one() { TEST_ASSERT_EQUAL(-1, FEED(11, 10, 0, 1, 11)); }
void test_bounce_is_zero() { TEST_ASSERT_EQUAL(0, FEED(11, 1, 11, 1, 11)); }
void test_half_turn_back_is_zero() { TEST_ASSERT_EQUAL(0, FEED(11, 1, 0, 1, 11)); }
void test_missed_edge_still_counts() { TEST_ASSERT_EQUAL(1, FEED(11, 1, 10, 11)); }
void test_two_detents() { TEST_ASSERT_EQUAL(2, FEED(11, 1, 0, 10, 11, 1, 0, 10, 11)); }
void test_repeated_state_no_effect() { TEST_ASSERT_EQUAL(1, FEED(11, 11, 1, 1, 0, 10, 11)); }

void test_press_after_stable_30ms() {
    Debouncer d(30, false);
    TEST_ASSERT_EQUAL(0, d.update(true, 0));
    TEST_ASSERT_EQUAL(0, d.update(false, 5));
    TEST_ASSERT_EQUAL(0, d.update(true, 10));
    TEST_ASSERT_EQUAL(0, d.update(true, 39));
    TEST_ASSERT_EQUAL(1, d.update(true, 40));
    TEST_ASSERT_EQUAL(0, d.update(true, 100));
}

void test_release_reported() {
    Debouncer d(30, false);
    d.update(true, 0);
    TEST_ASSERT_EQUAL(1, d.update(true, 40));
    TEST_ASSERT_EQUAL(0, d.update(false, 200));
    TEST_ASSERT_EQUAL(-1, d.update(false, 230));
}

void test_initially_pressed_no_press() {
    Debouncer d(30, true);
    TEST_ASSERT_EQUAL(0, d.update(true, 0));
    TEST_ASSERT_EQUAL(0, d.update(true, 100));
    TEST_ASSERT_EQUAL(0, d.update(false, 150));
    TEST_ASSERT_EQUAL(-1, d.update(false, 180));
}

static const uint8_t PINS[3] = {0, 1, 2};

void test_all_high_wake_on_low() {
    WakeMasks m = computeWakeMasks(0b111, PINS);
    TEST_ASSERT_EQUAL_UINT64(0b111, m.onLow);
    TEST_ASSERT_EQUAL_UINT64(0, m.onHigh);
}

void test_button_low_wakes_high() {
    WakeMasks m = computeWakeMasks(0b011, PINS);
    TEST_ASSERT_EQUAL_UINT64(0b011, m.onLow);
    TEST_ASSERT_EQUAL_UINT64(0b100, m.onHigh);
}

void test_all_low_wake_on_high() {
    WakeMasks m = computeWakeMasks(0b000, PINS);
    TEST_ASSERT_EQUAL_UINT64(0, m.onLow);
    TEST_ASSERT_EQUAL_UINT64(0b111, m.onHigh);
}

void test_button_wake_from_high_is_press() { TEST_ASSERT_TRUE(wakeIsPress(0b111, 0b100)); }
void test_release_wake_is_not_press() { TEST_ASSERT_FALSE(wakeIsPress(0b011, 0b100)); }
void test_encoder_wake_is_not_press() { TEST_ASSERT_FALSE(wakeIsPress(0b111, 0b011)); }

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_cw_detent_is_plus_one);
    RUN_TEST(test_ccw_detent_is_minus_one);
    RUN_TEST(test_bounce_is_zero);
    RUN_TEST(test_half_turn_back_is_zero);
    RUN_TEST(test_missed_edge_still_counts);
    RUN_TEST(test_two_detents);
    RUN_TEST(test_repeated_state_no_effect);
    RUN_TEST(test_press_after_stable_30ms);
    RUN_TEST(test_release_reported);
    RUN_TEST(test_initially_pressed_no_press);
    RUN_TEST(test_all_high_wake_on_low);
    RUN_TEST(test_button_low_wakes_high);
    RUN_TEST(test_all_low_wake_on_high);
    RUN_TEST(test_button_wake_from_high_is_press);
    RUN_TEST(test_release_wake_is_not_press);
    RUN_TEST(test_encoder_wake_is_not_press);
    return UNITY_END();
}
