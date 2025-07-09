#include "Zigbee.h"

#define ZIGBEE_LIGHT_ENDPOINT 10
#define ZIGBEE_WATER_LEVEL_SENSOR_EP 11

// Pins
const uint8_t relayPin       = 12;
const uint8_t resetButtonPin = BOOT_PIN;
const uint8_t powerButtonPin = 11;
const uint8_t waterLevelSensorPin = 13;

// Debounce configuration
volatile bool checkButtonHold = false; // Flag to check if button is held
volatile unsigned long buttonPressStart = 0; // Timestamp of initial button press
const unsigned long debounceDelay = 200;     // General debounce
const unsigned long minHoldTime = 75;       // Minimum press duration to count (ms)

ZigbeeLight zbLight(ZIGBEE_LIGHT_ENDPOINT);
ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(ZIGBEE_WATER_LEVEL_SENSOR_EP);

// Drive relay: true → ON (LOW), false → OFF (HIGH)
void setRelay(bool value) {
  digitalWrite(relayPin, value ? LOW : HIGH);
}

// Interrupt Service Routine (ISR) - triggered on button FALLING edge
void IRAM_ATTR handlePowerButtonPress() {
  // Record the time the button was pressed
  buttonPressStart = millis();
  checkButtonHold = true; // Set flag to process in main loop
}

// Called by Zigbee library on disconnect
void onZbDisconnected() {
  Serial.println("Zigbee disconnected, attempting rejoin…");
}

void setup() {
  Serial.begin(115200);

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // OFF by default
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(powerButtonPin, INPUT); // this has external pullup
  pinMode(waterLevelSensorPin, INPUT); // this has external pullup

  // Attach interrupt to power button pin
  attachInterrupt(digitalPinToInterrupt(powerButtonPin), handlePowerButtonPress, FALLING);

  // Zigbee setup
  zbLight.setManufacturerAndModel("ilmars-engineering-1", "coffee-machine-1");
  zbLight.onLightChange(setRelay); // Callback when Zigbee sends on/off
  Zigbee.addEndpoint(&zbLight);
  Zigbee.addEndpoint(&zbOccupancySensor);

  Serial.println("Starting Zigbee…");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start, rebooting…");
    ESP.restart();
  }

  Serial.print("Waiting for network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nJoined!");
}

void loop() {
  auto value = digitalRead(waterLevelSensorPin);
  static int lastState = 0; // unitialized
  int currentState = value == LOW 
    ? 1  // missing water
    : 2; // Has water

  if (currentState != lastState) {
    zbOccupancySensor.setOccupancy(currentState == 2);
    Serial.print("Has water: ");
    Serial.print(currentState == 2 ? "yes\n" : "no\n");
    lastState = currentState;
  }

  // Process button press if flag was set by ISR
  if (checkButtonHold) {
    checkButtonHold = false; // Clear the flag

    // Wait 100ms and confirm button is still pressed
    delay(minHoldTime);
    if (digitalRead(powerButtonPin) == LOW) {
      bool currentlyOn = (digitalRead(relayPin) == LOW);
      bool relayOn = !currentlyOn;

      Serial.println("Power button held, toggling relay");
      setRelay(relayOn);
      zbLight.setLight(relayOn);
    } else {
      Serial.println("Power button spike ignored");
    }
  }

  // Reconnect logic if Zigbee disconnects
  if (!Zigbee.connected()) {
    Serial.println("Zigbee connection lost, attempting rejoin…");
    static unsigned long lostAt = millis();
    if (millis() - lostAt > 30000) {
      Serial.println("Disconnected >30s, restarting");
      ESP.restart();
    }
  }

  // Factory reset: hold reset button for 3s
  if (digitalRead(resetButtonPin) == LOW) {
    unsigned long t0 = millis();
    while (digitalRead(resetButtonPin) == LOW) {
      if (millis() - t0 > 3000) {
        Serial.println("Factory reset + reboot");
        delay(1000);
        Zigbee.factoryReset();
      }
      delay(100);
    }
  }

  delay(100); // Small delay to reduce loop frequency
}
