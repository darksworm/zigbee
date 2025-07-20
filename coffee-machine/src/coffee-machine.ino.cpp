# 1 "/var/folders/41/ddnwy4p93f3_lt7v0tnq8yy80000gn/T/tmplueptuyw"
#include <Arduino.h>
# 1 "/Users/ilmars/Dev/private/zigbee/coffee-machine/src/coffee-machine.ino"
#include "Zigbee.h"
#include "freertos/semphr.h"

#define ZIGBEE_LIGHT_ENDPOINT 10
#define ZIGBEE_WATER_LEVEL_SENSOR_EP 11

SemaphoreHandle_t ioLock;


const uint8_t relayPin = 12;
const uint8_t resetButtonPin = BOOT_PIN;
const uint8_t powerButtonPin = 11;
const uint8_t waterLevelSensorPin = 13;


volatile bool checkButtonHold = false;
volatile unsigned long buttonPressStart = 0;
const unsigned long minHoldTime = 25;

ZigbeeLight zbLight(ZIGBEE_LIGHT_ENDPOINT);
ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(ZIGBEE_WATER_LEVEL_SENSOR_EP);
void safeSetRelay(bool v);
void setRelayFromZigbee(bool v);
void IRAM_ATTR handlePowerButtonPress();
void onZbDisconnected();
void setup();
void loop();
#line 23 "/Users/ilmars/Dev/private/zigbee/coffee-machine/src/coffee-machine.ino"
void safeSetRelay(bool v)
{
    xSemaphoreTake(ioLock, portMAX_DELAY);
    digitalWrite(relayPin, v ? LOW : HIGH);
    zbLight.setLight(v);
    xSemaphoreGive(ioLock);
}

void setRelayFromZigbee(bool v)
{
    safeSetRelay(v);
}


void IRAM_ATTR handlePowerButtonPress() {

  buttonPressStart = millis();
  checkButtonHold = true;
}


void onZbDisconnected() {
  Serial.println("Zigbee disconnected, attempting rejoin…");
}

void setup() {
  Serial.begin(115200);

  ioLock = xSemaphoreCreateMutex();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(powerButtonPin, INPUT);
  pinMode(waterLevelSensorPin, INPUT);


  attachInterrupt(digitalPinToInterrupt(powerButtonPin), handlePowerButtonPress, FALLING);


  zbLight.setManufacturerAndModel("ilmars-engineering-1", "coffee-machine-1");
  zbLight.onLightChange(setRelayFromZigbee);
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
  static int lastState = 0;
  int currentState = value == LOW
    ? 1
    : 2;

  if (currentState != lastState) {
    zbOccupancySensor.setOccupancy(currentState == 2);
    Serial.print("Has water: ");
    Serial.print(currentState == 2 ? "yes\n" : "no\n");
    lastState = currentState;
  }


  if (checkButtonHold) {
    if (millis() - buttonPressStart >= minHoldTime) {
      if (digitalRead(powerButtonPin) == LOW) {
        bool currentlyOn = (digitalRead(relayPin) == LOW);
        bool relayOn = !currentlyOn;

        Serial.println("Power button held, toggling relay");
        safeSetRelay(relayOn);
        zbLight.setLight(relayOn);
      } else {
        Serial.println("Power button spike ignored");
      }

      checkButtonHold = false;
    }
  }


  if (!Zigbee.connected()) {
    Serial.println("Zigbee connection lost, attempting rejoin…");
    static unsigned long lostAt = millis();
    if (millis() - lostAt > 30000) {
      Serial.println("Disconnected >30s, restarting");
      ESP.restart();
    }
  }


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

  delay(100);
}