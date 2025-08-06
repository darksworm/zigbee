#ifndef ZIGBEE_MODE_ZCZR
  #error "Zigbee router mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "DHTesp.h"
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

// -----------------------------------------------------------------------------
// Constants & configuration
// -----------------------------------------------------------------------------

#define OTA_MANUFACTURER_CODE               0x5555
#define OTA_UPGRADE_FIRMWARE_VERSION        0x1
#define OTA_UPGRADE_HW_VERSION              0x1
#define OTA_IMAGE_TYPE_CODE                 0x2
#define BUTTON_PIN                          BOOT_PIN

// Endpoints
static constexpr uint8_t HUMIDITY_EP     = 10;
static constexpr uint8_t RELAY_EP        = 11;
static constexpr uint8_t LED_STRIP_EP    = 12;

// Pins
static constexpr uint8_t RELAY_PIN       = 13;
static constexpr uint8_t DHT_PIN         = 12;
static constexpr uint8_t LED_PIN         = 10;

// DHT configuration
static constexpr auto     DHT_TYPE        = DHTesp::DHT22;

// LED strip configuration
static constexpr uint8_t  LED_COUNT       = 60;
static CRGB               leds[LED_COUNT];

// FreeRTOS timer handle
static TimerHandle_t      humidityTimer   = nullptr;

// Mutexes
portMUX_TYPE relayMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ledMux   = portMUX_INITIALIZER_UNLOCKED;

// Dirty flag for FastLED
volatile bool ledsDirty = false;

// Zigbee objects
ZigbeeTempSensor            zbTemp(HUMIDITY_EP);
ZigbeeLight                 zbRelay(RELAY_EP);
ZigbeeColorDimmableLight    zbLed(LED_STRIP_EP);
DHTesp                      dht;

// Transition engine structure
struct Transition {
  CRGB     from;
  CRGB     to;
  uint8_t  fromLvl;
  uint8_t  toLvl;
  uint32_t startMs;
  uint32_t durMs;
  bool     active;
} trans = {};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

inline CRGB currentColor() {
  return leds[0];
}

inline uint8_t currentLevel() {
  return FastLED.getBrightness();
}

static void markLedsDirty() {
  portENTER_CRITICAL(&ledMux);
    ledsDirty = true;
  portEXIT_CRITICAL(&ledMux);
}

static void applyFrame(const CRGB &c, uint8_t lvl) {
  FastLED.setBrightness(lvl);
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    leds[i] = (lvl > 0) ? c : CRGB::Black;
  }
  markLedsDirty();
}

static void startTransition(bool state,
                            uint8_t r, uint8_t g, uint8_t b,
                            uint8_t lvl,
                            uint32_t durMs = 800) {
  if (lvl == 254) lvl = 255;
  uint8_t maxc = std::max({r, g, b});
  if (maxc) {
    float k = 255.0f / maxc;
    r = roundf(r * k);
    g = roundf(g * k);
    b = roundf(b * k);
  }
  if (!state) { r = g = b = 0; }
  trans.from    = currentColor();
  trans.fromLvl = currentLevel();
  trans.to      = CRGB(r, g, b);
  trans.toLvl   = lvl;
  trans.startMs = millis();
  trans.durMs   = durMs;
  trans.active  = true;
}

static void serviceTransition() {
  if (!trans.active) return;
  uint32_t elapsed = millis() - trans.startMs;
  if (elapsed >= trans.durMs) {
    applyFrame(trans.to, trans.toLvl);
    trans.active = false;
    return;
  }
  uint8_t amt = (elapsed * 255UL) / trans.durMs;
  CRGB    c   = blend(trans.from, trans.to, amt);
  uint8_t lvl = lerp8by8(trans.fromLvl, trans.toLvl, amt);
  applyFrame(c, lvl);
}

// -----------------------------------------------------------------------------
// Zigbee callbacks
// -----------------------------------------------------------------------------

void relayChangeHandler(bool on) {
  Serial.printf("[ZIGBEE] Relay set to %s\n", on ? "ON" : "OFF");
  portENTER_CRITICAL(&relayMux);
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  portEXIT_CRITICAL(&relayMux);
}

void setLedHandler(bool state, uint8_t r, uint8_t g, uint8_t b, uint8_t lvl) {
  startTransition(state, r, g, b, lvl);
}

void identifyLed(uint16_t time) {
  static bool blink = false;
  trans.active = false;
  if (time == 0) return;
  blink = !blink;
  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    leds[i] = blink ? CRGB::White : CRGB::Black;
  }
  markLedsDirty();
}

// ----------------------------------------------------------------------------
// Humidity reporting
// ----------------------------------------------------------------------------

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

void onHumidityTimer(TimerHandle_t xTimer) {
  reportHumidity();
}

// -----------------------------------------------------------------------------
// Setup & Loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("[SETUP] Starting...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  zbRelay.onLightChange(relayChangeHandler);

  // DHT
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.setup(DHT_PIN, DHT_TYPE);
  Serial.printf("[SETUP] DHTesp initialized on pin %d\n", DHT_PIN);

  // FastLED
  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, LED_COUNT);
  FastLED.setBrightness(255);
  FastLED.show();
  zbLed.onLightChange(setLedHandler);
  zbLed.onIdentify(identifyLed);

  // Zigbee endpoints
  zbTemp.setManufacturerAndModel("ilmars-engineering", "humidity-sensor");
  zbTemp.addHumiditySensor(0, 100, 1);
  zbRelay.setManufacturerAndModel("ilmars-engineering", "fan-relay");
  zbLed.setManufacturerAndModel("ilmars-engineering", "led-strip");

  // OTA clients
  zbRelay.addOTAClient(OTA_UPGRADE_FIRMWARE_VERSION,
                       OTA_UPGRADE_FIRMWARE_VERSION,
                       OTA_UPGRADE_HW_VERSION,
                       OTA_MANUFACTURER_CODE,
                       OTA_IMAGE_TYPE_CODE);

  Zigbee.addEndpoint(&zbTemp);
  Zigbee.addEndpoint(&zbRelay);
  Zigbee.addEndpoint(&zbLed);

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

  // Sync initial relay state
  zbRelay.onLightChange(nullptr);
  bool initOn = zbRelay.getLightState();
  digitalWrite(RELAY_PIN, initOn ? HIGH : LOW);
  zbRelay.setLight(initOn);
  zbRelay.onLightChange(relayChangeHandler);

  // Dummy temperature for interview
  zbTemp.setTemperature(0.0f);
  zbTemp.report();

  // Humidity timer
  humidityTimer = xTimerCreate("HumTimer",
                               pdMS_TO_TICKS(5000),
                               pdTRUE,
                               nullptr,
                               onHumidityTimer);
  if (humidityTimer) {
    xTimerStart(humidityTimer, 0);
    Serial.println("[SETUP] Humidity timer started (5s).");
  }

  Serial.println("[SETUP] Ready.");

  // OTA
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

  // Transition engine for LEDs
  serviceTransition();

  // FastLED show if dirty
  if (ledsDirty) {
    portENTER_CRITICAL(&ledMux);
      ledsDirty = false;
    portEXIT_CRITICAL(&ledMux);
    FastLED.show();
  }

  yield();
}

