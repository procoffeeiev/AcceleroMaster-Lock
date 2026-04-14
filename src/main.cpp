#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// LIS3DH I2C registers
#define LIS3DH_ADDR     0x18
#define CFG1REG         0x20   // CTRL_REG1
#define AXREG           0x28   // OUT_X_L
#define AUTO_INC        0x80   // sub-address auto-increment bit

// Gesture / signal config
#define NUM_GESTURES    3
#define SIGLEN          50     // 500ms @ 100Hz per gesture — fits in 2.5KB SRAM
#define MATCH_THRESHOLD 0.85f

// Hardware pins (Circuit Playground Classic)
#define PIN_LEFT_BTN    19     // active-HIGH (board pull-down)
#define PIN_RIGHT_BTN    5     // active-HIGH
#define PIN_NEOPIXEL    17
#define PIN_LED         13
#define NEOPIXEL_COUNT  10

// State machine states
#define STATE_IDLE        0    // waiting: Left=record, Right=unlock (if key set)
#define STATE_RECORD_WAIT 1    // in record mode, waiting for Right to start gesture
#define STATE_RECORD_CAP  2    // actively sampling key[gestureIdx]
#define STATE_UNLOCK_WAIT 3    // in unlock mode, waiting for Right to start gesture
#define STATE_UNLOCK_CAP  4    // actively sampling ans[gestureIdx]
#define STATE_UNLOCKED    5    // all 3 gestures matched
#define STATE_FAILED      6    // at least one gesture failed

typedef struct {
  int16_t x[SIGLEN];
  int16_t y[SIGLEN];
  int16_t z[SIGLEN];
  int16_t len = 0;
} signal;

signal key[NUM_GESTURES];
signal ans[NUM_GESTURES];
bool   key_recorded = false;
int    state        = STATE_IDLE;
int    gestureIdx   = 0;
int    sampleCount  = 0;

Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// --- LED helpers ---

void showAll(uint32_t c) {
  for (int i = 0; i < NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, c);
  pixels.show();
}

void clearPixels() {
  pixels.clear();
  pixels.show();
}

// Pixels 0-2 = gesture 0, 3-5 = gesture 1, 6-8 = gesture 2, pixel 9 = mode indicator
void markGesture(int idx, uint32_t c) {
  int base = idx * 3;
  for (int i = base; i < base + 3; i++) pixels.setPixelColor(i, c);
  pixels.show();
}

// --- Button helper (rising-edge detection, debounced by 10ms loop cadence) ---

bool readButton(uint8_t pin) {
  static bool lastLeft  = false;
  static bool lastRight = false;
  bool &last = (pin == PIN_LEFT_BTN) ? lastLeft : lastRight;
  bool cur  = (digitalRead(pin) == HIGH);
  bool edge = cur && !last;
  last = cur;
  return edge;
}

// --- Accelerometer via I2C ---

void record(signal &a) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(AXREG | AUTO_INC);        // 0xA8: OUT_X_L with auto-increment
  Wire.endTransmission(false);         // repeated start — keep bus
  uint8_t n = Wire.requestFrom((uint8_t)LIS3DH_ADDR, (uint8_t)6);
  if (n < 6) {
    a.x[a.len] = a.y[a.len] = a.z[a.len] = 0;
  } else {
    uint8_t xlo = Wire.read(), xhi = Wire.read();
    uint8_t ylo = Wire.read(), yhi = Wire.read();
    uint8_t zlo = Wire.read(), zhi = Wire.read();
    a.x[a.len] = (int16_t)((xhi << 8) | xlo);
    a.y[a.len] = (int16_t)((yhi << 8) | ylo);
    a.z[a.len] = (int16_t)((zhi << 8) | zlo);
  }
  a.len++;
}

// --- Cosine similarity ---

float match(signal &k, signal &a) {
  float d = 0, nk = 0, na = 0;
  for (int i = 0; i < SIGLEN; i++) {
    d  += (float)k.x[i] * a.x[i] + (float)k.y[i] * a.y[i] + (float)k.z[i] * a.z[i];
    nk += (float)k.x[i] * k.x[i] + (float)k.y[i] * k.y[i] + (float)k.z[i] * k.z[i];
    na += (float)a.x[i] * a.x[i] + (float)a.y[i] * a.y[i] + (float)a.z[i] * a.z[i];
  }
  if (nk < 1.0f || na < 1.0f) return 0.0f;  // degenerate signal — no match
  return d / (sqrtf(nk) * sqrtf(na));
}

// --- State handlers ---

void handleIdle(bool leftBtn, bool rightBtn) {
  if (leftBtn) {
    // Enter record mode — clear any previous key
    for (int i = 0; i < NUM_GESTURES; i++) key[i].len = 0;
    key_recorded = false;
    gestureIdx   = 0;
    clearPixels();
    pixels.setPixelColor(9, pixels.Color(0, 0, 40));  // pixel 9: record-mode indicator
    markGesture(0, pixels.Color(0, 0, 40));            // gesture 0 prompt
    state = STATE_RECORD_WAIT;
  } else if (rightBtn && key_recorded) {
    // Enter unlock mode
    gestureIdx = 0;
    clearPixels();
    pixels.setPixelColor(9, pixels.Color(0, 40, 40));  // pixel 9: unlock-mode indicator
    markGesture(0, pixels.Color(0, 40, 40));
    state = STATE_UNLOCK_WAIT;
  } else if (rightBtn && !key_recorded) {
    // No key set — flash red briefly to signal error
    showAll(pixels.Color(40, 0, 0));
    delay(300);
    clearPixels();
  }
}

void handleRecordWait(bool rightBtn) {
  if (rightBtn) {
    key[gestureIdx].len = 0;
    sampleCount = 0;
    state = STATE_RECORD_CAP;
  }
}

void handleRecordCap() {
  record(key[gestureIdx]);
  sampleCount++;
  // Blink red LED during capture so user has timing feedback
  digitalWrite(PIN_LED, sampleCount % 10 < 5 ? HIGH : LOW);

  if (sampleCount >= SIGLEN) {
    digitalWrite(PIN_LED, LOW);
    markGesture(gestureIdx, pixels.Color(0, 40, 0));  // green: gesture captured

    gestureIdx++;
    if (gestureIdx >= NUM_GESTURES) {
      // All 3 gestures recorded — key saved
      key_recorded = true;
      delay(200);
      showAll(pixels.Color(20, 20, 20));  // white flash: key saved
      delay(500);
      clearPixels();
      state = STATE_IDLE;
    } else {
      // Prompt next gesture
      markGesture(gestureIdx, pixels.Color(0, 0, 40));
      state = STATE_RECORD_WAIT;
    }
    sampleCount = 0;
  }
}

void handleUnlockWait(bool rightBtn) {
  if (rightBtn) {
    ans[gestureIdx].len = 0;
    sampleCount = 0;
    state = STATE_UNLOCK_CAP;
  }
}

void handleUnlockCap() {
  record(ans[gestureIdx]);
  sampleCount++;
  digitalWrite(PIN_LED, sampleCount % 10 < 5 ? HIGH : LOW);

  if (sampleCount >= SIGLEN) {
    digitalWrite(PIN_LED, LOW);
    float sim = match(key[gestureIdx], ans[gestureIdx]);
    bool  ok  = (sim >= MATCH_THRESHOLD);

    Serial.print("Gesture "); Serial.print(gestureIdx);
    Serial.print(" similarity: "); Serial.println(sim);

    markGesture(gestureIdx, ok ? pixels.Color(0, 40, 0) : pixels.Color(40, 0, 0));

    if (!ok) {
      delay(300);
      showAll(pixels.Color(40, 0, 0));
      delay(3000);
      clearPixels();
      state = STATE_IDLE;
    } else {
      gestureIdx++;
      if (gestureIdx >= NUM_GESTURES) {
        delay(300);
        showAll(pixels.Color(0, 40, 0));
        delay(3000);
        clearPixels();
        state = STATE_IDLE;
      } else {
        markGesture(gestureIdx, pixels.Color(0, 40, 40));  // cyan: next gesture prompt
        state = STATE_UNLOCK_WAIT;
      }
    }
    sampleCount = 0;
  }
}

// --- Arduino entry points ---

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LEFT_BTN,  INPUT);
  pinMode(PIN_RIGHT_BTN, INPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pixels.begin();
  pixels.setBrightness(40);
  clearPixels();

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(CFG1REG);
  Wire.write(0x47);          // ODR=100Hz, normal mode, XYZ enabled
  Wire.endTransmission();
  delay(10);
}

void loop() {
  bool leftBtn  = readButton(PIN_LEFT_BTN);
  bool rightBtn = readButton(PIN_RIGHT_BTN);

  switch (state) {
  case STATE_IDLE:        handleIdle(leftBtn, rightBtn);  break;
  case STATE_RECORD_WAIT: handleRecordWait(rightBtn);     break;
  case STATE_RECORD_CAP:  handleRecordCap();              break;
  case STATE_UNLOCK_WAIT: handleUnlockWait(rightBtn);     break;
  case STATE_UNLOCK_CAP:  handleUnlockCap();              break;
  }

  delay(10);  // 100Hz sample cadence
}
