// Copy to include/config.h (gitignored) and fill in per knob.
#pragma once

// Wi-Fi
#define WIFI_SSID "your-ssid"
#define WIFI_PASSWORD "your-password"

// Static IP skips DHCP on every wake. Leave KNOB_STATIC_IP empty to use DHCP.
// Only use a static IP that the router reserves for this knob.
#define KNOB_STATIC_IP ""
#define KNOB_GATEWAY "192.168.1.1"
#define KNOB_SUBNET "255.255.255.0"
#define KNOB_DNS "192.168.1.1"

// Knob
#define KNOB_NAME "volume"   // used in UDP log lines
#define AWAKE_MS 30000       // stay awake this long after the last input
#define ENCODER_REVERSE 0    // set to 1 if clockwise turns the wrong way

// Variant "volume": Buchardt hub (LUCI on TCP 7777)
#define HUB_IP "192.168.1.47"
#define VOLUME_STEP 1        // hub volume steps (0..100) per detent
#define VOLUME_MAX 50        // the knob never sets more; match the hub's max volume setting

// Variant "lamp": Philips Hue bridge (v1 API over HTTP)
#define HUE_BRIDGE_IP "192.168.1.220"
#define HUE_APP_KEY ""       // create with tools/hue_pair.sh
#define HUE_GROUP_ID 0       // room id, listed by tools/hue_pair.sh
#define DIM_STEP 25          // brightness steps (1..254) per detent
#define HUE_MIN_INTERVAL_MS 1000
#define HUE_TRANSITION_DS 4  // transition time in 1/10 s
