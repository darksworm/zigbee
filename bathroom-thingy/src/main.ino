#ifndef ZIGBEE_MODE_ZCZR
  #error "Zigbee router mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "DHTesp.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#define OTA_MANUFACTURER_CODE               0x5555
#define OTA_UPGRADE_FIRMWARE_VERSION        0x1
#define OTA_UPGRADE_HW_VERSION              0x1
#define OTA_IMAGE_TYPE_CODE                 0x2
#define BUTTON_PIN                          BOOT_PIN

static constexpr uint8_t RELAY_PIN    = 13;
static constexpr uint8_t DHT_PIN      = 12;            // choose a non-USB GPIO
static constexpr auto     DHT_TYPE     = DHTesp::DHT22;

static constexpr uint8_t HUMIDITY_EP  = 10;
static constexpr uint8_t RELAY_EP     = 11;

// FreeRTOS software timer handle
static TimerHandle_t humidityTimer = nullptr;

// Mutex for protecting the relay GPIO
portMUX_TYPE relayMux = portMUX_INITIALIZER_UNLOCKED;

// Zigbee endpoints and DHT driver
ZigbeeTempSensor zbTemp(HUMIDITY_EP);
ZigbeeLight     zbRelay(RELAY_EP);
DHTesp          dht;

// Forward declaration
void reportHumidity();

// Timer callback (runs in the timer service task)
void onHumidityTimer(TimerHandle_t xTimer) {
  reportHumidity();
}

// Handle incoming On/Off commands without recursion
void relayChangeHandler(bool on) {
  Serial.printf("[ZIGBEE] Relay set to %s\n", on ? "ON" : "OFF");
  portENTER_CRITICAL(&relayMux);
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  portEXIT_CRITICAL(&relayMux);
}

// Read and report humidity once
void reportHumidity() {
  float h = dht.getHumidity();
  const char* st = dht.getStatusString();
  Serial.printf("[DHTesp] status=%s, hum=%.2f%%\n", st, isnan(h) ? 0.0f : h);
  if (!isnan(h)) {
    zbTemp.setHumidity(h);
    zbTemp.report();
    Serial.printf("[ZIGBEE] Reported humidity = %.2f%%\n", h);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("[SETUP] Starting...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Relay GPIO setup
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  zbRelay.onLightChange(relayChangeHandler);

  // DHTesp setup on GPIO12 with internal pull-up
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.setup(DHT_PIN, DHT_TYPE);
  Serial.printf("[SETUP] DHTesp initialized on pin %d\n", DHT_PIN);

  // Zigbee humidity+temperature endpoint
  zbTemp.setManufacturerAndModel("ilmars-engineering", "humidity-sensor");

  zbTemp.addHumiditySensor(0, 100, 1);

  // Zigbee relay endpoint
  zbRelay.setManufacturerAndModel("ilmars-engineering", "fan-relay");

  zbRelay.addOTAClient(OTA_UPGRADE_FIRMWARE_VERSION,
                            OTA_UPGRADE_FIRMWARE_VERSION,
                            OTA_UPGRADE_HW_VERSION,
                            OTA_MANUFACTURER_CODE,
                            OTA_IMAGE_TYPE_CODE);

  Zigbee.addEndpoint(&zbTemp);
  Zigbee.addEndpoint(&zbRelay);

  // Start Zigbee in router mode
  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("[ZIGBEE] begin() failed; rebooting...");
    ESP.restart();
  }
  Serial.print("[ZIGBEE] Joining network");
  while (!Zigbee.connected()) {
    Serial.print('.');
    delay(100);
  }
  Serial.println(" connected!");

  // Sync initial relay state without triggering the handler
  zbRelay.onLightChange(nullptr);
  bool initOn = zbRelay.getLightState();
  Serial.printf("[SETUP] Coordinator relay state = %s\n", initOn ? "ON" : "OFF");
  digitalWrite(RELAY_PIN, initOn ? HIGH : LOW);
  zbRelay.setLight(initOn);
  zbRelay.onLightChange(relayChangeHandler);

  // Send a dummy temperature so ZHA completes interview
  zbTemp.setTemperature(0.0f);
  zbTemp.report();

  // Create and start a 5-second FreeRTOS software timer
  humidityTimer = xTimerCreate(
    "HumTimer",                  // timer name
    pdMS_TO_TICKS(5000),         // period = 5000 ms
    pdTRUE,                      // auto-reload
    nullptr,                     // timer ID
    onHumidityTimer              // callback function
  );
  if (humidityTimer) {
    xTimerStart(humidityTimer, 0);
    Serial.println("[SETUP] Humidity timer started (5s).");
  } else {
    Serial.println("[ERROR] Failed to create humidity timer!");
  }

  Serial.println("[SETUP] Ready.\n");

  zbRelay.requestOTAUpdate();
}

void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
    delay(100);
    unsigned long t0 = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (millis() - t0 > 3000) {
        delay(1000);
        Zigbee.factoryReset();
      }
    }
  }

  // Yield to allow the Zigbee core and timer daemon to run
  yield();
}
