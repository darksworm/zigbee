#include "Zigbee.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define ZIGBEE_LIGHT_ENDPOINT 10
#define ZIGBEE_WATER_LEVEL_SENSOR_EP 11

SemaphoreHandle_t ioLock;

// Pins
const uint8_t relayPin       = 12;
const uint8_t resetButtonPin = BOOT_PIN;
const uint8_t powerButtonPin = 11;
const uint8_t waterLevelSensorPin = 13;

// Debounce configuration
volatile bool checkButtonHold = false; // Flag to check if button is held
volatile unsigned long buttonPressStart = 0; // Timestamp of initial button press
const unsigned long minHoldTime = 25;       // Minimum press duration to count (ms)

ZigbeeLight zbLight(ZIGBEE_LIGHT_ENDPOINT);
ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(ZIGBEE_WATER_LEVEL_SENSOR_EP);

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

static TimerHandle_t zbWatchdogTmr;
static uint32_t      disconnectedSince = 0;        // 0 = we’re connected

void IRAM_ATTR zbWatchdogCb(TimerHandle_t)
{
    if (!Zigbee.connected()) {
        if (disconnectedSince == 0) {
            disconnectedSince = esp_log_timestamp();     // first miss
            ESP_LOGW("ZB", "Zigbee lost, watching…");
        } else if (esp_log_timestamp() - disconnectedSince > 30'000) {
            ESP_LOGE("ZB", "Disconnected >30 s, rebooting");
            esp_restart();                                // hard reset
        }
    } else {
        disconnectedSince = 0;                            // link OK
    }
}

void setup() {
  zbWatchdogTmr = xTimerCreate("zbwd",                 // name
                               pdMS_TO_TICKS(10'000),  // fire every 10 s
                               pdTRUE,                 // auto‑reload
                               nullptr,                // ID not used
                               zbWatchdogCb);          // callback
  xTimerStart(zbWatchdogTmr, 0);

  Serial.begin(115200);

  ioLock = xSemaphoreCreateMutex();

  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // OFF by default
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(powerButtonPin, INPUT); // this has external pullup
  pinMode(waterLevelSensorPin, INPUT); // this has external pullup

  // Attach interrupt to power button pin
  attachInterrupt(digitalPinToInterrupt(powerButtonPin), handlePowerButtonPress, FALLING);

  // Zigbee setup
  zbLight.setManufacturerAndModel("ilmars-engineering-1", "coffee-machine-1");
  zbLight.onLightChange(setRelayFromZigbee); // Callback when Zigbee sends on/off
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

      checkButtonHold = false; // Clear the flag
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


  vTaskDelay(pdMS_TO_TICKS(33));   // or even longer
}
