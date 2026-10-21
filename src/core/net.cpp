#include "net.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdarg.h>

#include "config.h"
#include "line_protocol.h"
#include "rtc_state.h"

constexpr uint16_t UDP_PORT = 47777;
constexpr uint32_t CACHED_CONNECT_MS = 2500;

static WiFiUDP udp;

static bool waitConnected(uint32_t startMs, uint32_t limitMs) {
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startMs >= limitMs) return false;
        delay(5);
    }
    return true;
}

bool netConnect(uint32_t budgetMs) {
    uint32_t start = millis();
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);

    if (strlen(KNOB_STATIC_IP) > 0) {
        IPAddress ip, gateway, subnet, dns;
        ip.fromString(KNOB_STATIC_IP);
        gateway.fromString(KNOB_GATEWAY);
        subnet.fromString(KNOB_SUBNET);
        dns.fromString(KNOB_DNS);
        WiFi.config(ip, gateway, subnet, dns);
    }

    WifiCache &cache = rtcWifiCache();
#ifdef KNOB_TEST_STALE_WIFI_CACHE
    // Test hook: poison the cache once after flashing to exercise the fallback
    if (esp_reset_reason() != ESP_RST_DEEPSLEEP) {
        const uint8_t bogus[6] = {0x02, 0, 0, 0, 0, 0x01};
        memcpy(cache.bssid, bogus, sizeof(bogus));
        cache.channel = 13;
        cache.valid = true;
    }
#endif

    if (cache.valid) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD, cache.channel, cache.bssid);
        if (waitConnected(start, min(budgetMs, CACHED_CONNECT_MS))) return true;
        WiFi.disconnect();
        cache.valid = false;
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    if (!waitConnected(start, budgetMs)) return false;

    memcpy(cache.bssid, WiFi.BSSID(), sizeof(cache.bssid));
    cache.channel = WiFi.channel();
    cache.valid = true;
    return true;
}

void netBroadcastLine(const char *line) {
    if (WiFi.status() != WL_CONNECTED) return;
    // Subnet broadcast rather than 255.255.255.255: on macOS a listener bound to the subnet
    // broadcast address can then share the port with BuchardtBar's exclusive wildcard socket
    udp.beginPacket(WiFi.broadcastIP(), UDP_PORT);
    udp.write((const uint8_t *)line, strlen(line));
    udp.endPacket();
}

void netLog(const char *fmt, ...) {
    char text[200];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);
    Serial.println(text);

    char line[240];
    if (formatLog(line, sizeof(line), KNOB_NAME, text)) netBroadcastLine(line);
}
