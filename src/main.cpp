#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <esp_sleep.h>

// Encoder pins
constexpr int PIN_A = GPIO_NUM_0;       // Encoder channel A
constexpr int PIN_B = GPIO_NUM_1;       // Encoder channel B
constexpr int PIN_BUTTON = GPIO_NUM_2;  // Encoder button

// WiFi and MQTT configuration
constexpr char WIFI_SSID[] = "Aether";
constexpr char WIFI_PASSWORD[] = "cityoftheamstel";
constexpr char MQTT_SERVER[] = "192.168.1.174";  // MQTT broker IP
constexpr int MQTT_PORT = 1883;
constexpr char MQTT_USER[] = "mqtt";
constexpr char MQTT_PASSWORD[] = "r37zQSVw";
constexpr char MQTT_CONFIG_TOPIC[] = "homeassistant/sensor/encoder/config";
constexpr char MQTT_STATE_TOPIC[] = "homeassistant/sensor/encoder/state";

// Encoder and inactivity settings
constexpr unsigned long INACTIVITY_THRESHOLD_MS = 30 * 1000; // 30 seconds
constexpr int ENCODER_MIN = 0;                              // Min position
constexpr int ENCODER_MAX = 255;                            // Max position

// Globals
int encoderPosition = 0;
unsigned long lastActivityTime = 0;
RotaryEncoder encoder(PIN_A, PIN_B, RotaryEncoder::LatchMode::TWO03);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;

// Encoder ISR
void IRAM_ATTR updateEncoder() {
    encoder.tick();
    lastActivityTime = millis();
}

// Publish MQTT configuration for HA discovery
void publishConfig() {
    String configPayload =
        String("{") + "\"name\": \"Rotary Encoder\"," +
        "\"unique_id\": \"rotary_encoder\"," + "\"state_topic\": \"" +
        String(MQTT_STATE_TOPIC) + "\"," +
        "\"value_template\": \"{{ value_json.position }}\"" + "}";
    mqttClient.publish(MQTT_CONFIG_TOPIC, configPayload.c_str(), true);
}

// Reconnect to MQTT broker
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("Connected to MQTT broker");
            publishConfig();
        } else {
            Serial.print("Failed, rc=");
            Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

// Initialize WiFi
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        delay(500);
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());
}

// Enter deep sleep after inactivity
void enterDeepSleep() {
    Serial.println("Entering deep sleep due to inactivity...");

    // Save encoder position to NVS
    preferences.putInt("position", encoderPosition);
    preferences.end();  // Close preferences

    // Disable GPIO hold to reduce power consumption
    gpio_deep_sleep_hold_dis();

    // Configure GPIOs as wakeup sources (active LOW)
    esp_deep_sleep_enable_gpio_wakeup(
        (1ULL << PIN_A) | (1ULL << PIN_B) | (1ULL << PIN_BUTTON),
        ESP_GPIO_WAKEUP_GPIO_LOW);

    // Optionally isolate GPIOs to minimize power leakage
    esp_sleep_config_gpio_isolate();

    // Ensure GPIOs are set to input to avoid unnecessary power draw
    gpio_set_direction(static_cast<gpio_num_t>(PIN_A), GPIO_MODE_INPUT);
    gpio_set_direction(static_cast<gpio_num_t>(PIN_B), GPIO_MODE_INPUT);
    gpio_set_direction(static_cast<gpio_num_t>(PIN_BUTTON), GPIO_MODE_INPUT);

    Serial.println("GPIO wakeup sources configured. Going to deep sleep...");

    // Start deep sleep
    esp_deep_sleep_start();
}

// Setup
void setup() {
    Serial.begin(115200);

    // Initialize NVS
    preferences.begin("encoder", false);
    encoderPosition = preferences.getInt("position", 0);
    encoder.setPosition(encoderPosition);

    // Setup encoder and button pins
    pinMode(PIN_A, INPUT);  // Built-in pull-up resistors in the encoder
    pinMode(PIN_B, INPUT);  // Built-in pull-up resistors in the encoder
    pinMode(PIN_BUTTON,
            INPUT);  // Built-in pull-up resistors in the encoder button

    attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), updateEncoder, CHANGE);

    lastActivityTime = millis();

    // Connect to WiFi and MQTT
    connectWiFi();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    // Turn on the LED
    pinMode(8, OUTPUT);

    Serial.println("Setup complete");
}

// Main loop
void loop() {
    static int lastPosition = encoderPosition;

    encoder.tick();
    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    // Handle encoder position changes
    int newPosition = encoder.getPosition();
    newPosition = constrain(newPosition, ENCODER_MIN, ENCODER_MAX);
    if (newPosition != lastPosition) {
        encoderPosition = newPosition;
        String payload = "{\"position\":" + String(encoderPosition) + "}";
        mqttClient.publish(MQTT_STATE_TOPIC, payload.c_str());

        Serial.print("Encoder position: ");
        Serial.println(encoderPosition);

        lastActivityTime = millis();
        lastPosition = newPosition;
    }

    // Handle button press
    if (digitalRead(PIN_BUTTON) == LOW) {
        Serial.println("Button pressed");
        lastActivityTime = millis();
    }

    // Handle inactivity
    if (millis() - lastActivityTime > INACTIVITY_THRESHOLD_MS) {
        enterDeepSleep();
    }

    delay(250);  // Reduce loop frequency
}