#include <string.h>
#include <unity.h>

#include "hue.h"
#include "rate_coalescer.h"

void setUp() {}
void tearDown() {}

static char buf[256];

void test_group_paths() {
    hue::groupPath(buf, sizeof(buf), "abc", 3);
    TEST_ASSERT_EQUAL_STRING("/api/abc/groups/3", buf);
    hue::groupActionPath(buf, sizeof(buf), "abc", 3);
    TEST_ASSERT_EQUAL_STRING("/api/abc/groups/3/action", buf);
}

void test_dim_up_turns_on() {
    hue::dimBody(buf, sizeof(buf), 30, 4);
    TEST_ASSERT_EQUAL_STRING("{\"on\":true,\"bri_inc\":30,\"transitiontime\":4}", buf);
}

void test_dim_down() {
    hue::dimBody(buf, sizeof(buf), -30, 4);
    TEST_ASSERT_EQUAL_STRING("{\"bri_inc\":-30,\"transitiontime\":4}", buf);
}

void test_dim_clamps() {
    hue::dimBody(buf, sizeof(buf), 400, 4);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"bri_inc\":254"));
    hue::dimBody(buf, sizeof(buf), -400, 4);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"bri_inc\":-254"));
}

void test_on_body() {
    hue::onBody(buf, sizeof(buf), true);
    TEST_ASSERT_EQUAL_STRING("{\"on\":true}", buf);
    hue::onBody(buf, sizeof(buf), false);
    TEST_ASSERT_EQUAL_STRING("{\"on\":false}", buf);
}

void test_build_request() {
    hue::buildRequest(buf, sizeof(buf), "PUT", "192.168.1.220", "/api/k/groups/3/action", "{\"on\":true}");
    TEST_ASSERT_EQUAL_STRING(
        "PUT /api/k/groups/3/action HTTP/1.1\r\n"
        "Host: 192.168.1.220\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: 11\r\n"
        "Connection: keep-alive\r\n"
        "\r\n"
        "{\"on\":true}",
        buf);
}

void test_build_get_no_body() {
    hue::buildRequest(buf, sizeof(buf), "GET", "192.168.1.220", "/api/k/groups/3", nullptr);
    TEST_ASSERT_EQUAL_STRING(
        "GET /api/k/groups/3 HTTP/1.1\r\n"
        "Host: 192.168.1.220\r\n"
        "Connection: keep-alive\r\n"
        "\r\n",
        buf);
}

void test_parse_any_on() {
    bool anyOn = false;
    TEST_ASSERT_TRUE(hue::parseAnyOn("{\"state\":{\"all_on\":false,\"any_on\":true}}", &anyOn));
    TEST_ASSERT_TRUE(anyOn);
    TEST_ASSERT_TRUE(hue::parseAnyOn("{\"state\":{\"all_on\": false, \"any_on\": false}}", &anyOn));
    TEST_ASSERT_FALSE(anyOn);
}

void test_parse_any_on_missing() {
    bool anyOn = false;
    TEST_ASSERT_FALSE(hue::parseAnyOn("{\"name\":\"x\"}", &anyOn));
}

void test_hue_error() {
    const char *body = "[{\"error\":{\"type\":1,\"address\":\"/\",\"description\":\"unauthorized user\"}}]";
    bool anyOn = false;
    TEST_ASSERT_TRUE(hue::isHueError(body));
    TEST_ASSERT_FALSE(hue::parseAnyOn(body, &anyOn));
    TEST_ASSERT_FALSE(hue::isHueError("[{\"success\":{\"/groups/3/action/on\":true}}]"));
}

void test_first_take_immediate() {
    RateCoalescer r(1000);
    r.add(2);
    TEST_ASSERT_EQUAL(2, r.take(0));
}

void test_coalesces_within_interval() {
    RateCoalescer r(1000);
    r.add(2);
    TEST_ASSERT_EQUAL(2, r.take(0));
    r.add(1);
    TEST_ASSERT_EQUAL(0, r.take(500));
    r.add(3);
    TEST_ASSERT_EQUAL(4, r.take(1000));
    TEST_ASSERT_EQUAL(0, r.take(2500));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_group_paths);
    RUN_TEST(test_dim_up_turns_on);
    RUN_TEST(test_dim_down);
    RUN_TEST(test_dim_clamps);
    RUN_TEST(test_on_body);
    RUN_TEST(test_build_request);
    RUN_TEST(test_build_get_no_body);
    RUN_TEST(test_parse_any_on);
    RUN_TEST(test_parse_any_on_missing);
    RUN_TEST(test_hue_error);
    RUN_TEST(test_first_take_immediate);
    RUN_TEST(test_coalesces_within_interval);
    return UNITY_END();
}
