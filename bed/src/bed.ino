#ifndef ZIGBEE_MODE_ZCZR
  #error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <FastLED.h>

#define ZIGBEE_RGB_LIGHT_ENDPOINT 10
#define CONTACT_SWITCH_ENDPOINT_NUMBER 11
#define BUTTON_PIN                BOOT_PIN
#define BED_BUTTON_PIN 8

// Each strip has 1 booster LED (idx 0) + 12 real LEDs (1..12)
static constexpr uint8_t SKIP       = 1;
static constexpr uint8_t REAL_LEDS  = 12;
static constexpr uint8_t TOTAL_LEDS = SKIP + REAL_LEDS;

// Per-strip pixel buffers
CRGB leds0[TOTAL_LEDS];
CRGB leds1[TOTAL_LEDS];

ZigbeeColorDimmableLight zbColorLight(ZIGBEE_RGB_LIGHT_ENDPOINT);
ZigbeeContactSwitch zbContactSwitch = ZigbeeContactSwitch(CONTACT_SWITCH_ENDPOINT_NUMBER);

//–– Zigbee → FastLED callback ––
void setRGBLight(bool    state,
                 uint8_t red,
                 uint8_t green,
                 uint8_t blue,
                 uint8_t level)
{
  if (level == 254) {
    level = 255;
  }

  FastLED.setBrightness(level);

  leds0[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (level > 1) {
      leds0[i].r = red;
      leds0[i].g = green;
      leds0[i].b = blue;
    } else {
      leds0[i] = CRGB::Black;
    }
  }
  leds1[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (level > 1) {
      leds1[i].r = red;
      leds1[i].g = green;
      leds1[i].b = blue;
    } else {
      leds1[i] = CRGB::Black;
    }
  }

  FastLED.show();
}

void identify(uint16_t time)
{
  static bool blink = false;
  log_d("Identify for %d sec", time);
  if (time == 0) {
    zbColorLight.restoreLight();
    return;
  }
  blink = !blink;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    leds0[i] = leds1[i] =
      blink ? CRGB::White : CRGB::Black;
  }
  FastLED.show();
}

void setup()
{
  Serial.begin(115200);

  // Use the generic FastLED driver (bit-bang)
  FastLED.setBrightness(255);
  FastLED.setMaxPowerInMilliWatts(3000);

  // strip 0 → GP14
  auto &ctl0 = FastLED.addLeds<WS2812B, 20, GRB>(leds0, TOTAL_LEDS);
  ctl0.setCorrection(TypicalLEDStrip);
  auto &ctl1 = FastLED.addLeds<WS2812B, 19, GRB>(leds1, TOTAL_LEDS);
  ctl1.setCorrection(TypicalLEDStrip);

  // clear all pixels (booster + real)
  for (uint8_t i = 0; i < TOTAL_LEDS; ++i) {
    leds0[i] = leds1[i] = CRGB::Black;
  }
  FastLED.show();

  // button for factory-reset / brightness step
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BED_BUTTON_PIN, INPUT);

  // Zigbee callbacks & endpoint
  zbColorLight.onLightChange(setRGBLight);
  zbColorLight.onIdentify(identify);
  zbColorLight.setManufacturerAndModel("Espressif", "ZBColor4Strip");

  Serial.println("Adding Zigbee endpoint");
  Zigbee.addEndpoint(&zbColorLight);
  Zigbee.addEndpoint(&zbContactSwitch);

  if (!Zigbee.begin(ZIGBEE_ROUTER)) {
    Serial.println("Zigbee failed to start, rebooting");
    ESP.restart();
  }
  Serial.print("Joining network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println(" connected");

  zbContactSwitch.setClosed();
}

bool stable     = HIGH;          // debounced state
bool candidate  = HIGH;          // possible new state
unsigned long lastFlip = 0;
#define DEBOUNCE_MS 25

void loop()
{
  // long press = factory reset, short press = +50 level
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(100);
    unsigned long t0 = millis();
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (millis() - t0 > 3000) {
        Serial.println("Factory reset in 1s");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
    zbColorLight.setLightLevel(
      zbColorLight.getLightLevel() + 50
    );
  }

  bool raw = digitalRead(BED_BUTTON_PIN);

  if (raw != candidate) {             // first time we see a flip
    candidate = raw;
    lastFlip  = millis();             // start debounce timer
  }

  if ( (millis() - lastFlip) >= DEBOUNCE_MS &&
       candidate != stable) {         // stayed steady long enough
    stable = candidate;               // accept it

    if (stable == LOW) {              // pressed
      zbContactSwitch.setOpen();
      Serial.println("setting open");
    } else {
      zbContactSwitch.setClosed();
      Serial.println("setting closed");
    }
  }

  FastLED.delay(33);
}
