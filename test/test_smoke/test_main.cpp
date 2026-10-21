#include <unity.h>

void setUp() {}
void tearDown() {}

void test_harness_runs() {
    TEST_ASSERT_EQUAL(1, 1);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_harness_runs);
    return UNITY_END();
}
