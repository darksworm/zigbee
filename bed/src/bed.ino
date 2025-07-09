#ifndef ZIGBEE_MODE_ED
  #error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <FastLED.h>

#define ZIGBEE_RGB_LIGHT_ENDPOINT 10
#define BUTTON_PIN                BOOT_PIN

// Each strip has 1 booster LED (idx 0) + 12 real LEDs (1..12)
static constexpr uint8_t SKIP       = 1;
static constexpr uint8_t REAL_LEDS  = 12;
static constexpr uint8_t TOTAL_LEDS = SKIP + REAL_LEDS;

// Per-strip pixel buffers
CRGB leds0[TOTAL_LEDS];
CRGB leds1[TOTAL_LEDS];
CRGB leds2[TOTAL_LEDS];
CRGB leds3[TOTAL_LEDS];

ZigbeeColorDimmableLight zbColorLight(ZIGBEE_RGB_LIGHT_ENDPOINT);

//–– Zigbee → FastLED callback ––
void setRGBLight(bool    state,
                 uint8_t red,
                 uint8_t green,
                 uint8_t blue,
                 uint8_t level)
{
  float scale = (state && level) ? float(level) / 255.0f : 0.0f;

  // strip 0 on GP14
  leds0[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (scale > 0) {
      leds0[i].r = red   * scale;
      leds0[i].g = green * scale;
      leds0[i].b = blue  * scale;
    } else {
      leds0[i] = CRGB::Black;
    }
  }
  // strip 1 on GP13
  leds1[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (scale > 0) {
      leds1[i].r = red   * scale;
      leds1[i].g = green * scale;
      leds1[i].b = blue  * scale;
    } else {
      leds1[i] = CRGB::Black;
    }
  }
  // strip 2 on GP12
  leds2[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (scale > 0) {
      leds2[i].r = red   * scale;
      leds2[i].g = green * scale;
      leds2[i].b = blue  * scale;
    } else {
      leds2[i] = CRGB::Black;
    }
  }
  // strip 3 on GP11
  leds3[0] = CRGB::Black;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    if (scale > 0) {
      leds3[i].r = red   * scale;
      leds3[i].g = green * scale;
      leds3[i].b = blue  * scale;
    } else {
      leds3[i] = CRGB::Black;
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
    leds0[i] = leds1[i] = leds2[i] = leds3[i] =
      blink ? CRGB::White : CRGB::Black;
  }
  FastLED.show();
}

void setup()
{
  Serial.begin(115200);

  // Use the generic FastLED driver (bit-bang)
  FastLED.setBrightness(255);

  // strip 0 → GP14
  auto &ctl0 = FastLED.addLeds<WS2812B>(leds0, TOTAL_LEDS);
  ctl0.setPin(14).setCorrection(TypicalLEDStrip);
  // strip 1 → GP13
  auto &ctl1 = FastLED.addLeds<WS2812B>(leds1, TOTAL_LEDS);
  ctl1.setPin(13).setCorrection(TypicalLEDStrip);
  // strip 2 → GP12
  auto &ctl2 = FastLED.addLeds<WS2812B>(leds2, TOTAL_LEDS);
  ctl2.setPin(12).setCorrection(TypicalLEDStrip);
  // strip 3 → GP11
  auto &ctl3 = FastLED.addLeds<WS2812B>(leds3, TOTAL_LEDS);
  ctl3.setPin(11).setCorrection(TypicalLEDStrip);

  // clear all pixels (booster + real)
  for (uint8_t i = 0; i < TOTAL_LEDS; ++i) {
    leds0[i] = leds1[i] = leds2[i] = leds3[i] = CRGB::Black;
  }
  FastLED.show();

  // button for factory-reset / brightness step
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Zigbee callbacks & endpoint
  zbColorLight.onLightChange(setRGBLight);
  zbColorLight.onIdentify(identify);
  zbColorLight.setManufacturerAndModel("Espressif", "ZBColor4Strip");

  Serial.println("Adding Zigbee endpoint");
  Zigbee.addEndpoint(&zbColorLight);
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start, rebooting");
    ESP.restart();
  }
  Serial.print("Joining network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println(" connected");
}

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
  delay(100);
}
