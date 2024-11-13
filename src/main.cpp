#include <Arduino.h>
#include <WiFi.h>
// #include <Rot

const int pinA = 7;        // S1
const int pinB = 9;        // S2
const int buttonPin = 10;  // Key

int encoderPosition = 0;

int lastEncoded = 0;

void updateEncoder();

void setup() {
    Serial.begin(115200);
    pinMode(8, OUTPUT);

    WiFi.mode(WIFI_STA);
    WiFi.begin("Aether", "cityoftheamstel");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println("Connecting to WiFi..");
    }

    Serial.println("Connected to the WiFi network");
    Serial.println(WiFi.localIP());

    pinMode(pinA, INPUT);       // Encoder channel A (S1)
    pinMode(pinB, INPUT);       // Encoder channel B (S2)
    pinMode(buttonPin, INPUT);  // Key (button)

    // Attach interrupts to encoder pins
    attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), updateEncoder, CHANGE);
}

void loop() {
    int buttonState = digitalRead(buttonPin);
    if (buttonState == LOW) {  // Button is pressed (active LOW)
        Serial.println("Button pressed");
    }
    delay(100);
}

void updateEncoder() {
    // Read both channels
    int MSB = digitalRead(pinA);  // Most significant bit
    int LSB = digitalRead(pinB);  // Least significant bit

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded << 2) | encoded;

    // Determine rotation direction
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        encoderPosition++;
        Serial.print(encoderPosition);
        Serial.println(" | Encoder rotated clockwise");
    }
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        encoderPosition--;
        Serial.print(encoderPosition);
        Serial.println(" | Encoder rotated counter-clockwise");
    }

    lastEncoded = encoded;
}