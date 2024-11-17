#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <esp_sleep.h>

// Encoder pins
constexpr int PIN_A = GPIO_NUM_0;       // Encoder channel A
constexpr int PIN_B = GPIO_NUM_1;       // Encoder channel B
constexpr int PIN_BUTTON = GPIO_NUM_2;  // Encoder button

// WiFi configuration
constexpr char WIFI_SSID[] = "Aether";
constexpr char WIFI_PASSWORD[] = "cityoftheamstel";

// MQTT configuration
constexpr char MQTT_SERVER[] = "192.168.1.174";  // MQTT broker IP
constexpr int MQTT_PORT = 1883;
constexpr char MQTT_USER[] = "mqtt";
constexpr char MQTT_PASSWORD[] = "r37zQSVw";

// MQTT topics and unique IDs
constexpr char MQTT_ENCODER_CONFIG_TOPIC[] = "homeassistant/sensor/encoder_position/config";
constexpr char MQTT_ENCODER_STATE_TOPIC[] = "homeassistant/sensor/encoder_position/state";
constexpr char MQTT_BUTTON_CONFIG_TOPIC[] = "homeassistant/sensor/encoder_button/config";
constexpr char MQTT_BUTTON_STATE_TOPIC[] = "homeassistant/sensor/encoder_button/state";

constexpr char ENCODER_UNIQUE_ID[] = "rotary_encoder_position";
constexpr char BUTTON_UNIQUE_ID[] = "rotary_encoder_button";

// Encoder and inactivity settings
constexpr unsigned long INACTIVITY_THRESHOLD_MS = 30 * 1000;  // 30 seconds
constexpr int ENCODER_MIN = 0;                                // Min position
constexpr int ENCODER_MAX = 255;                              // Max position

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

// Publish MQTT configuration for Home Assistant discovery
void publishConfig() {
    // Configuration for the encoder position
    JsonDocument configDoc;
    configDoc["name"] = "Rotary Encoder Position";
    configDoc["unique_id"] = ENCODER_UNIQUE_ID;
    configDoc["state_topic"] = MQTT_ENCODER_STATE_TOPIC;
    configDoc["value_template"] = "{{ value_json.position }}";

    char configPayload[256];
    serializeJson(configDoc, configPayload);  // Serialize JSON to string
    mqttClient.publish(MQTT_ENCODER_CONFIG_TOPIC, configPayload, true);

    // Configuration for the button state
    configDoc.clear();
    configDoc["name"] = "Encoder Button State";
    configDoc["unique_id"] = BUTTON_UNIQUE_ID;
    configDoc["state_topic"] = MQTT_BUTTON_STATE_TOPIC;
    configDoc["value_template"] = "{{ value_json.button }}";

    serializeJson(configDoc, configPayload);
    mqttClient.publish(MQTT_BUTTON_CONFIG_TOPIC, configPayload, true);
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
        delay(100);
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
    pinMode(PIN_BUTTON, INPUT);  // Built-in pull-up resistors in the encoder button

    attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), updateEncoder, CHANGE);

    lastActivityTime = millis();

    // Connect to WiFi and MQTT
    connectWiFi();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    // Turn on the LED
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);

    Serial.println("Setup complete");
}

// Main loop
void loop() {
    static int lastPosition = encoderPosition;
    static bool lastButtonState = HIGH;

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

        JsonDocument stateDoc;
        stateDoc["position"] = encoderPosition;

        char statePayload[128];
        serializeJson(stateDoc, statePayload);  // Serialize JSON to string

        mqttClient.publish(MQTT_ENCODER_STATE_TOPIC, statePayload);

        Serial.print("Encoder position: ");
        Serial.println(encoderPosition);

        lastActivityTime = millis();
        lastPosition = newPosition;
    }

    // Handle button state
    bool buttonState = digitalRead(PIN_BUTTON) == LOW;  // LOW means pressed
    if (buttonState != lastButtonState) {
        JsonDocument buttonDoc;
        buttonDoc["button"] = buttonState;

        char buttonPayload[128];
        serializeJson(buttonDoc, buttonPayload);

        mqttClient.publish(MQTT_BUTTON_STATE_TOPIC, buttonPayload);

        Serial.print("Button state: ");
        Serial.println(buttonState ? "pressed" : "released");

        lastActivityTime = millis();
        lastButtonState = buttonState;
    }

    // Handle inactivity
    if (millis() - lastActivityTime > INACTIVITY_THRESHOLD_MS) {
        enterDeepSleep();
    }

    delay(250);  // Reduce loop frequency
}