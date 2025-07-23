#ifndef ZIGBEE_MODE_ZCZR
  #error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include <EEPROM.h>
#include "Zigbee.h"
#include <FastLED.h>

// -----------------------------------------------------------------------------
// Constants & configuration
// -----------------------------------------------------------------------------
#define ZIGBEE_RGB_LIGHT_ENDPOINT   10
#define CONTACT_SWITCH_ENDPOINT_NUMBER 11
#define BUTTON_PIN                  BOOT_PIN
#define BED_BUTTON_PIN              8
#define EEPROM_SIZE                 5

// Each strip has 1 booster LED (idx 0) + N real LEDs (1..N)
static constexpr uint8_t SKIP        = 0;      // index to skip (booster), keep as 0 per original code
static constexpr uint8_t REAL_LEDS   = 30;
static constexpr uint8_t TOTAL_LEDS  = SKIP + REAL_LEDS;

// Transition duration (ms)
static constexpr uint32_t DEFAULT_TRANSITION_MS = 3000;  // programmable

// If SKIP == 0 we read color from LED 1, otherwise from SKIP
static constexpr uint8_t FIRST_VISIBLE_LED = (SKIP == 0) ? 1 : SKIP;

// -----------------------------------------------------------------------------
// LED buffers
// -----------------------------------------------------------------------------
CRGB leds0[TOTAL_LEDS];

// -----------------------------------------------------------------------------
// Zigbee endpoints
// -----------------------------------------------------------------------------
ZigbeeColorDimmableLight zbColorLight(ZIGBEE_RGB_LIGHT_ENDPOINT);
ZigbeeContactSwitch      zbContactSwitch(CONTACT_SWITCH_ENDPOINT_NUMBER);

// -----------------------------------------------------------------------------
// Persistent config
// -----------------------------------------------------------------------------
struct RGBCfg {
    bool    state;
    uint8_t red, green, blue, level;

    void log() const {
      Serial.print(F("state 0x"));  Serial.println(state, HEX);
      Serial.print(F("R "));        Serial.println(red);
      Serial.print(F("G "));        Serial.println(green);
      Serial.print(F("B "));        Serial.println(blue);
      Serial.print(F("L "));        Serial.println(level);
      Serial.println();             // blank line
    }
} cfg;

void saveRGB(const RGBCfg &c) {
  EEPROM.put(0, c);
  EEPROM.commit();
}

void loadRGB(RGBCfg &c) {
  EEPROM.get(0, c);
}

// -----------------------------------------------------------------------------
// LED update synchronization
// -----------------------------------------------------------------------------
volatile bool   ledsDirty = false;
portMUX_TYPE    ledMux    = portMUX_INITIALIZER_UNLOCKED;

void markLedsDirty() {
  portENTER_CRITICAL_ISR(&ledMux);
  ledsDirty = true;
  portEXIT_CRITICAL_ISR(&ledMux);
}

// -----------------------------------------------------------------------------
// Transition engine
// -----------------------------------------------------------------------------
struct Transition {
  CRGB     from;
  CRGB     to;
  uint8_t  fromLvl;
  uint8_t  toLvl;
  uint32_t startMs;
  uint32_t durMs;
  bool     active;
} trans = {};

inline CRGB currentShownColor() {
  return leds0[FIRST_VISIBLE_LED];
}

inline uint8_t currentShownLevel() {
  return FastLED.getBrightness();
}

static void applyFrame(const CRGB &c, uint8_t lvl) {
  FastLED.setBrightness(lvl);

  // Keep booster behavior as before if you want it black; comment out next line if not needed
  // leds0[0] = CRGB::Black;

  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    leds0[i] = (lvl > 1) ? c : CRGB::Black;
  }
  markLedsDirty();
}

void startTransition(bool state,
                     uint8_t r, uint8_t g, uint8_t b,
                     uint8_t lvl,
                     uint32_t durMs = DEFAULT_TRANSITION_MS)
{
  if (lvl == 254) lvl = 255;

  // Normalize to full scale like original code
  uint8_t maxc = std::max({r, g, b});
  if (maxc) {
    float k = 255.0f / maxc;
    r = roundf(r * k);
    g = roundf(g * k);
    b = roundf(b * k);
  }

  if (!state) { r = g = b = 0; }

  trans.from    = currentShownColor();
  trans.fromLvl = currentShownLevel();
  trans.to      = CRGB(r, g, b);
  trans.toLvl   = lvl;
  trans.startMs = millis();
  trans.durMs   = durMs;
  trans.active  = true;

  // Save TARGET values (so power cycle resumes at requested color)
  saveRGB({state, r, g, b, lvl});
}

void serviceTransition() {
  if (!trans.active) return;

  uint32_t elapsed = millis() - trans.startMs;
  if (elapsed >= trans.durMs) {
    applyFrame(trans.to, trans.toLvl);
    trans.active = false;
    return;
  }

  // amount 0..255
  uint8_t amt = (elapsed * 255UL) / trans.durMs;

  CRGB    c   = blend(trans.from, trans.to, amt);
  uint8_t lvl = lerp8by8(trans.fromLvl, trans.toLvl, amt);

  applyFrame(c, lvl);
}

// -----------------------------------------------------------------------------
// Zigbee → FastLED callback (now just starts a transition)
// -----------------------------------------------------------------------------
void setRGBLight(bool state,
                 uint8_t red,
                 uint8_t green,
                 uint8_t blue,
                 uint8_t level)
{
  // Interrupt any running fade and start a new one from the current on-screen state
  startTransition(state, red, green, blue, level);
}

// -----------------------------------------------------------------------------
// Identify callback
// -----------------------------------------------------------------------------
void identify(uint16_t time)
{
  static bool blink = false;
  log_d("Identify for %d sec", time);

  // Stop transition so it doesn't fight our blink pattern
  trans.active = false;

  if (time == 0) {
    zbColorLight.restoreLight();
    return;
  }
  blink = !blink;
  for (uint8_t i = SKIP; i < TOTAL_LEDS; ++i) {
    leds0[i] = blink ? CRGB::White : CRGB::Black;
  }
  markLedsDirty();
}

// -----------------------------------------------------------------------------
// Setup & loop
// -----------------------------------------------------------------------------
bool stable     = HIGH;          // debounced state
bool candidate  = HIGH;          // possible new state
unsigned long lastFlip = 0;
#define DEBOUNCE_MS 25

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  FastLED.setBrightness(255);
  FastLED.setMaxPowerInMilliWatts(4000);

  auto &ctl0 = FastLED.addLeds<WS2812B, 12, RGB>(leds0, TOTAL_LEDS);
  ctl0.setCorrection(TypicalLEDStrip);

  // Clear all pixels (booster + real)
  for (uint8_t i = 0; i < TOTAL_LEDS; ++i) {
    leds0[i] = CRGB::Black;
  }
  FastLED.show();

  // Buttons
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BED_BUTTON_PIN, INPUT);

  // Zigbee callbacks & endpoint
  zbColorLight.onLightChange(setRGBLight);
  zbColorLight.onIdentify(identify);
  zbColorLight.setManufacturerAndModel("ilmars-engineering", "color-tube");

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

  // Load last saved color and fade to it
  RGBCfg s;
  loadRGB(s);
  if (s.state == 0xFF && s.red == 0xFF && s.green == 0xFF && s.blue == 0xFF && s.level == 0xFF) {
    // Default if EEPROM empty
    startTransition(false, 255, 135, 16, 0); // state=false => will fade to black
  } else {
    startTransition(s.state, s.red, s.green, s.blue, s.level);
  }
}

void loop() {
  // Long press = factory reset, short press = +50 level
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

  // Bed button debounce -> contact switch
  bool raw = digitalRead(BED_BUTTON_PIN);
  if (raw != candidate) {             // first time we see a flip
    candidate = raw;
    lastFlip  = millis();             // start debounce timer
  }
  if ((millis() - lastFlip) >= DEBOUNCE_MS && candidate != stable) {
    stable = candidate;               // accept it
    if (stable == LOW) {              // pressed
      zbContactSwitch.setOpen();
      Serial.println("setting open");
    } else {
      zbContactSwitch.setClosed();
      Serial.println("setting closed");
    }
  }

  // Tick the transition engine
  serviceTransition();

  // Only the Arduino task touches FastLED
  if (ledsDirty) {
    portENTER_CRITICAL(&ledMux);
    ledsDirty = false;
    portEXIT_CRITICAL(&ledMux);
    FastLED.show();        // single, thread‑safe call
  }

  FastLED.delay(33);
}

