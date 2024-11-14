#include <Arduino.h>
#include <PubSubClient.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <esp_sleep.h>

// Define the pins for the encoder
const int PIN_A = GPIO_NUM_1;       // S1
const int PIN_B = GPIO_NUM_2;       // S2
const int PIN_BUTTON = GPIO_NUM_0;  // Key

// WiFi and MQTT settings
const char* ssid = "Aether";
const char* password = "cityoftheamstel";
const char* mqttServer = "192.168.1.174";  // Replace with your MQTT broker IP
const int mqttPort = 1883;
const char* mqttConfigTopic = "homeassistant/sensor/encoder/config";
const char* mqttStateTopic = "homeassistant/sensor/encoder/state";
const char* mqttUser = "mqtt";
const char* mqttPassword = "r37zQSVw";

// Initialize encoder position and activity timer
int encoderPosition = 0;
unsigned long lastActivityTime = 0;
const unsigned long inactivityThreshold = 5000;  // 5 seconds threshold

// Initialize RotaryEncoder object
RotaryEncoder encoder(PIN_A, PIN_B, RotaryEncoder::LatchMode::TWO03);

// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Function to handle encoder updates
void IRAM_ATTR updateEncoder() {
    encoder.tick();
    lastActivityTime = millis();
}

// Publish MQTT configuration message
void publishConfig() {
    String configPayload = "{\"name\": \"Rotary Encoder\","
                           "\"unique_id\": \"rotary_encoder\","
                           "\"state_topic\": \"" + String(mqttStateTopic) + "\","
                           "\"value_template\": \"{{ value_json.position }}\""
                            "}";

    mqttClient.publish(mqttConfigTopic, configPayload.c_str(), true);  // Retain message for HA discovery
}

// MQTT reconnect function
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
            Serial.println("connected to MQTT broker");
            publishConfig();  // Publish configuration on connection
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(8, OUTPUT);

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Setup MQTT
    mqttClient.setServer(mqttServer, mqttPort);

    // Setup encoder pins
    pinMode(PIN_A, INPUT);
    pinMode(PIN_B, INPUT);
    pinMode(PIN_BUTTON, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), updateEncoder, CHANGE);

    lastActivityTime = millis();
}

void loop() {
    static int lastPosition = 0;
    encoder.tick();  // Manually update encoder in loop

    if (!mqttClient.connected()) {
        reconnectMQTT();
    }
    mqttClient.loop();

    int newPosition = encoder.getPosition();
    if (newPosition != lastPosition) {
        encoderPosition = newPosition;
        String direction = (encoder.getDirection() == RotaryEncoder::Direction::CLOCKWISE) ? "clockwise" : "counterclockwise";

        // State JSON payload for MQTT
        String payload = "{\"position\":" + String(encoderPosition) + "}";

        // Publish the state message
        mqttClient.publish(mqttStateTopic, payload.c_str());

        Serial.print("Encoder position: ");
        Serial.println(encoderPosition);
        Serial.println("Encoder direction: " + direction);

        lastPosition = newPosition;
        lastActivityTime = millis();
    }

    // Check button state
    if (digitalRead(PIN_BUTTON) == LOW) {
        Serial.println("Button pressed");
        lastActivityTime = millis();
    }

    // Check for inactivity and enter deep sleep if needed
    if (millis() - lastActivityTime > inactivityThreshold) {
        Serial.println("Entering deep sleep due to inactivity...");
        gpio_deep_sleep_hold_dis();
        esp_sleep_config_gpio_isolate();
        gpio_set_direction(gpio_num_t(0), GPIO_MODE_INPUT);
        gpio_set_direction(gpio_num_t(1), GPIO_MODE_INPUT);
        gpio_set_direction(gpio_num_t(2), GPIO_MODE_INPUT);
        esp_deep_sleep_enable_gpio_wakeup(1 << 0, ESP_GPIO_WAKEUP_GPIO_LOW);
        esp_deep_sleep_enable_gpio_wakeup(1 << 1, ESP_GPIO_WAKEUP_GPIO_LOW);
        esp_deep_sleep_enable_gpio_wakeup(1 << 2, ESP_GPIO_WAKEUP_GPIO_LOW);
        esp_deep_sleep_start();
    }

    delay(100);  // Debounce delay
}