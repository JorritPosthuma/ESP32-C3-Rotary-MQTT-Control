#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <RotaryEncoder.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <esp_sleep.h>
#include <nvs_flash.h>
#include "esp_task_wdt.h"

// Encoder pins
constexpr int PIN_A = GPIO_NUM_0;       // Encoder channel A
constexpr int PIN_B = GPIO_NUM_1;       // Encoder channel B
constexpr int PIN_BUTTON = GPIO_NUM_2;  // Encoder button

// MQTT topics and unique IDs (dynamic configuration later)
String mqttServer;
String mqttUsername;
String mqttPassword;
String entityPrefix;
String entityName;
constexpr int MQTT_PORT = 1883;

int encoderMin = -1;  // Configurable minimum position
int encoderMax = -1;  // Configurable maximum position

String mqttEncoderConfigTopic;
String mqttEncoderStateTopic;
String mqttButtonConfigTopic;
String mqttButtonStateTopic;

// Encoder and inactivity settings
constexpr unsigned long INACTIVITY_THRESHOLD_MS = 30 * 1000;  // 30 seconds
constexpr unsigned long BUTTON_HOLD_TIME_MS = 5000;           // 5 seconds

// Globals
int encoderPosition = 0;
unsigned long lastActivityTime = 0;
bool buttonState = false;

RotaryEncoder encoder(PIN_A, PIN_B, RotaryEncoder::LatchMode::TWO03);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences preferences;

// Function to reset NVS storage
void resetNVS() {
    esp_task_wdt_init(10, true);  // Set timeout to 10 seconds

    Serial.println("Resetting NVS storage...");
    preferences.end();  // Close preferences
    nvs_flash_erase();  // Erase all NVS storage
    nvs_flash_init();   // Reinitialize NVS
    ESP.restart();      // Restart the ESP
}

// Encoder interrupt handler
void IRAM_ATTR updateEncoder() {
    lastActivityTime = millis();

    encoder.tick();
}

// Publish MQTT configuration for Home Assistant discovery
void publishConfig() {
    // Configuration for the encoder position
    JsonDocument configDoc;
    configDoc["name"] = entityName + " Position";
    configDoc["unique_id"] = entityPrefix + "_position";
    configDoc["state_topic"] = mqttEncoderStateTopic.c_str();
    configDoc["value_template"] = "{{ value_json.position }}";

    char configPayload[256];
    serializeJson(configDoc, configPayload);
    mqttClient.publish(mqttEncoderConfigTopic.c_str(), configPayload, true);

    // Configuration for the button state
    configDoc.clear();
    configDoc["name"] = entityName + " Button";
    configDoc["unique_id"] = entityPrefix + "_button";
    configDoc["state_topic"] = mqttButtonStateTopic.c_str();
    configDoc["value_template"] = "{{ value_json.button }}";

    serializeJson(configDoc, configPayload);
    mqttClient.publish(mqttButtonConfigTopic.c_str(), configPayload, true);
}

// Reconnect to MQTT broker
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect("BigButtonClient", mqttUsername.c_str(), mqttPassword.c_str())) {
            Serial.println("Connected to MQTT broker");
            publishConfig();
        } else {
            Serial.print("Failed, rc=");
            Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

// Initialize WiFi and gather additional configuration
void configureWiFiAndMQTT() {
    WiFiManager wifiManager;

    // Custom parameters for MQTT configuration
    WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT Server IP", mqttServer.c_str(), 40);
    WiFiManagerParameter custom_mqtt_user("mqtt_user", "MQTT Username", mqttUsername.c_str(), 40);
    WiFiManagerParameter custom_mqtt_password("mqtt_pass", "MQTT Password",   mqttPassword.c_str(), 40);
    WiFiManagerParameter custom_entity_prefix("entity_prefix", "Entity Prefix",   entityPrefix.c_str(), 40);
    WiFiManagerParameter custom_entity_name("entity_name", "Entity Name", entityName.c_str(), 40);
    WiFiManagerParameter custom_encoder_min("encoder_min", "Encoder Min (-1 for no constraint)", String(encoderMin).c_str(), 10);
    WiFiManagerParameter custom_encoder_max("encoder_max", "Encoder Max (-1 for no constraint)", String(encoderMax).c_str(), 10);

    // Add parameters to the WiFiManager
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_password);
    wifiManager.addParameter(&custom_entity_prefix);
    wifiManager.addParameter(&custom_entity_name);
    wifiManager.addParameter(&custom_encoder_min);
    wifiManager.addParameter(&custom_encoder_max);

    // Start the configuration portal
    if (!wifiManager.autoConnect("Big Round Button")) {
        Serial.println("Failed to connect. Restarting...");
        ESP.restart();
    }

    // Save configuration values
    mqttServer = custom_mqtt_server.getValue();
    mqttUsername = custom_mqtt_user.getValue();
    mqttPassword = custom_mqtt_password.getValue();
    entityPrefix = custom_entity_prefix.getValue();
    entityName = custom_entity_name.getValue();
    encoderMin = atoi(custom_encoder_min.getValue());
    encoderMax = atoi(custom_encoder_max.getValue());

    // Save to preferences
    preferences.putString("mqtt_server", mqttServer);
    preferences.putString("mqtt_user", mqttUsername);
    preferences.putString("mqtt_pass", mqttPassword);
    preferences.putString("entity_prefix", entityPrefix);
    preferences.putString("entity_name", entityName);
    preferences.putInt("encoder_min", encoderMin);
    preferences.putInt("encoder_max", encoderMax);

    // Update topics dynamically
    mqttEncoderConfigTopic = "homeassistant/sensor/" + entityPrefix + "_position/config";
    mqttEncoderStateTopic = "homeassistant/sensor/" + entityPrefix + "_position/state";
    mqttButtonConfigTopic = "homeassistant/sensor/" + entityPrefix + "_button/config";
    mqttButtonStateTopic = "homeassistant/sensor/" + entityPrefix + "_button/state";

    // Connect to MQTT broker
    mqttClient.setServer(mqttServer.c_str(), MQTT_PORT);
    reconnectMQTT();

    Serial.println("WiFi and MQTT configured successfully.");
}

// Enter deep sleep after inactivity
void enterDeepSleep() {
    Serial.println("Entering deep sleep due to inactivity...");

    // Save encoder position to NVS
    encoder.tick();
    preferences.putInt("position", encoder.getPosition());
    preferences.end();

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
    // Initialize serial
    Serial.begin(115200);

    // Setup encoder and button pins
    pinMode(PIN_A, INPUT);
    pinMode(PIN_B, INPUT);
    pinMode(PIN_BUTTON, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), updateEncoder, CHANGE);

    // Initialize NVS
    preferences.begin("encoder", false);

    // Load stored values
    mqttServer = preferences.getString("mqtt_server", mqttServer);
    mqttUsername = preferences.getString("mqtt_user", mqttUsername);
    mqttPassword = preferences.getString("mqtt_pass", mqttPassword);
    entityPrefix = preferences.getString("entity_prefix", entityPrefix);
    entityName = preferences.getString("entity_name", entityName);
    encoderMin = preferences.getInt("encoder_min", encoderMin);
    encoderMax = preferences.getInt("encoder_max", encoderMax);

    encoderPosition = preferences.getInt("position", 0);
    encoder.setPosition(encoderPosition);

    // Configure WiFi and MQTT
    configureWiFiAndMQTT();

    // Turn on the LED
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);
    Serial.println("Setup complete");
}

// Main loop
void loop() {
    static int lastPosition = encoderPosition;

    // Handle inactivity
    if (millis() - lastActivityTime > INACTIVITY_THRESHOLD_MS) {
        enterDeepSleep();
    }

    // Only continue if we have a valid MQTT connection
    if (!mqttClient.connected()) {
        return;
    }

    // Make sure we're still connected to the MQTT broker
    mqttClient.loop();

    // Handle encoder position changes
    encoder.tick();
    int newPosition = encoder.getPosition();
    
    // If constraining is enabled
    if (encoderMin != encoderMax) {
        int constrainedPosition = constrain(newPosition, encoderMin, encoderMax);

        if (constrainedPosition != newPosition) {
            encoder.setPosition(constrainedPosition);
            newPosition = constrainedPosition;
        }
    }
    
    // Publish new position if it has changed
    if (newPosition != lastPosition) {
        encoderPosition = newPosition;

        JsonDocument stateDoc;
        stateDoc["position"] = encoderPosition;

        char statePayload[128];
        serializeJson(stateDoc, statePayload);

        mqttClient.publish(mqttEncoderStateTopic.c_str(), statePayload);

        Serial.print("Encoder position: ");
        Serial.println(encoderPosition);

        lastPosition = newPosition;
    }

    // Handle button presses
    bool newButtonState = digitalRead(PIN_BUTTON);

    if (newButtonState != buttonState) {
        buttonState = newButtonState;

        JsonDocument buttonDoc;
        buttonDoc["button"] = buttonState;

        char buttonPayload[128];
        serializeJson(buttonDoc, buttonPayload);

        mqttClient.publish(mqttButtonStateTopic.c_str(), buttonPayload);

        Serial.print("Button state: ");
        Serial.println(buttonState ? "ON" : "OFF");
    }

    delay(250);
}