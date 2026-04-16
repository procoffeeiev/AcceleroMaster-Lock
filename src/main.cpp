#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ── Hardware ───────────────────────────────────────────────────────────────────
#define PIN_NEO    17
#define NEO_COUNT  10
#define PIN_BTN_L  19        // left  button: record
#define PIN_BTN_R  5         // right button: unlock
#define PIN_LED    13        // onboard red LED (blinks during capture)

// ── LIS3DH SPI ────────────────────────────────────────────────────────────────
#define READREG  0x80
#define INCREG   0x40
#define AXREG    0x28
#define CFG1REG  0x20

// ── Signal ────────────────────────────────────────────────────────────────────
#define SIGLEN        50     // samples per gesture (0.5 s @ 100 Hz)
#define NUM_GESTURES  3      // gestures in the key/unlock sequence

typedef struct {
  int16_t x[SIGLEN];
  int16_t y[SIGLEN];
  int16_t z[SIGLEN];
  int16_t len;
} signal;

// ── State machine ──────────────────────────────────────────────────────────────
#define STATE_IDLE        0  // L=record  R=unlock
#define STATE_REC_WAIT    1  // recording: press R to start current gesture
#define STATE_REC_CAP     2  // capturing key gesture[gestureIdx]
#define STATE_UNL_WAIT    3  // unlock: press R to start current gesture
#define STATE_UNL_CAP     4  // capturing verification gesture
#define STATE_UNLOCKED    5  // all gestures matched → green
#define STATE_FAILED      6  // mismatch → blink red

// ── Algorithms (dev-selectable via Serial: send '0'–'3') ─────────────────────
#define ALGO_COSINE   0
#define ALGO_PEARSON  1
#define ALGO_GRADIENT 2
#define ALGO_ENERGY   3
#define NUM_ALGOS     4

const float THRESHOLDS[NUM_ALGOS] = {0.85f, 0.75f, 0.80f, 0.80f};

// ── Globals ────────────────────────────────────────────────────────────────────
static int           state      = STATE_IDLE;
static int           algo       = ALGO_COSINE;
static bool          keySet     = false;
static int           gestureIdx = 0;
static unsigned long timer      = 0;
static int           blinkCnt   = 0;
static bool          lastL = false, lastR = false;
static bool          btnL  = false, btnR  = false;

// key[g] stores the 3 recorded gestures; ans is reused per verification
static signal key[NUM_GESTURES];
static signal ans;

Adafruit_NeoPixel strip(NEO_COUNT, PIN_NEO, NEO_GRB + NEO_KHZ800);

// ─────────────────────────────────────────────────────────────────────────────
// Display helpers
// Pixel layout: gesture g occupies pixels g*3 .. g*3+2  (3 gestures × 3 = 9)
// Pixel 9 stays dark (reserved / unused).
//
// Colour semantics (same for record and unlock, only "current" colour differs):
//   already done   → red
//   current        → yellow (record) | blue (unlock)
//   not yet done   → green
// ─────────────────────────────────────────────────────────────────────────────
static void showAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NEO_COUNT; i++)
    strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

static void showIdle() {
  if (keySet) showAll(15, 0, 0);      // dim red: device is locked
  else        { strip.clear(); strip.show(); }  // off: no key set
}

// cur = gestureIdx being shown; isRecord selects yellow vs blue for current
static void showSequence(int cur, bool isRecord) {
  strip.clear();
  for (int g = 0; g < NUM_GESTURES; g++) {
    uint32_t col;
    if      (g <  cur) col = strip.Color(20,  0,  0);               // red: done
    else if (g == cur) col = isRecord ? strip.Color(20, 20, 0)       // yellow: recording
                                     : strip.Color( 0,  0, 20);     // blue: unlocking
    else               col = strip.Color( 0, 20,  0);               // green: not yet
    strip.setPixelColor(g*3,   col);
    strip.setPixelColor(g*3+1, col);
    strip.setPixelColor(g*3+2, col);
  }
  strip.show();
}

// ─────────────────────────────────────────────────────────────────────────────
// Button input – rising-edge detection, 10 ms debounce via loop cadence
// ─────────────────────────────────────────────────────────────────────────────
static void readButtons() {
  bool curL = digitalRead(PIN_BTN_L);
  bool curR = digitalRead(PIN_BTN_R);
  btnL = curL && !lastL;
  btnR = curR && !lastR;
  lastL = curL;
  lastR = curR;
}

// ─────────────────────────────────────────────────────────────────────────────
// Accelerometer sample via SPI (LIS3DH – preserves original framework)
// ─────────────────────────────────────────────────────────────────────────────
static void record(signal &a) {
  PORTB &= ~(1 << PB4);
  SPI.transfer(AXREG | READREG | INCREG);
  uint8_t lo, hi;
  lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF);
  a.x[a.len] = (int16_t)((hi << 8) | lo);
  lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF);
  a.y[a.len] = (int16_t)((hi << 8) | lo);
  lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF);
  a.z[a.len] = (int16_t)((hi << 8) | lo);
  PORTB |= (1 << PB4);
  a.len++;
}

// ─────────────────────────────────────────────────────────────────────────────
// Algorithms
// ─────────────────────────────────────────────────────────────────────────────
static float matchCosine(signal &k, signal &a) {
  float d = 0, nk = 0, na = 0;
  for (int i = 0; i < SIGLEN; i++) {
    d  += (float)k.x[i]*a.x[i] + (float)k.y[i]*a.y[i] + (float)k.z[i]*a.z[i];
    nk += (float)k.x[i]*k.x[i] + (float)k.y[i]*k.y[i] + (float)k.z[i]*k.z[i];
    na += (float)a.x[i]*a.x[i] + (float)a.y[i]*a.y[i] + (float)a.z[i]*a.z[i];
  }
  if (nk < 1.0f || na < 1.0f) return 0.0f;
  return d / (sqrtf(nk) * sqrtf(na));
}

static float matchPearson(signal &k, signal &a) {
  int16_t *kA[3] = {k.x, k.y, k.z};
  int16_t *aA[3] = {a.x, a.y, a.z};
  float rSum = 0.0f;
  int   cnt  = 0;
  for (int ax = 0; ax < 3; ax++) {
    float mk = 0, ma = 0;
    for (int i = 0; i < SIGLEN; i++) { mk += kA[ax][i]; ma += aA[ax][i]; }
    mk /= SIGLEN;  ma /= SIGLEN;
    float cov = 0, vk = 0, va = 0;
    for (int i = 0; i < SIGLEN; i++) {
      float dk = kA[ax][i] - mk, da = aA[ax][i] - ma;
      cov += dk*da;  vk += dk*dk;  va += da*da;
    }
    if (vk < 1.0f || va < 1.0f) continue;
    rSum += cov / sqrtf(vk * va);
    cnt++;
  }
  return cnt > 0 ? rSum / cnt : 0.0f;
}

static float matchGradient(signal &k, signal &a) {
  float d = 0, nk = 0, na = 0;
  for (int i = 0; i < SIGLEN - 1; i++) {
    float kx = k.x[i+1]-k.x[i], ky = k.y[i+1]-k.y[i], kz = k.z[i+1]-k.z[i];
    float ax = a.x[i+1]-a.x[i], ay = a.y[i+1]-a.y[i], az = a.z[i+1]-a.z[i];
    d  += kx*ax + ky*ay + kz*az;
    nk += kx*kx + ky*ky + kz*kz;
    na += ax*ax + ay*ay + az*az;
  }
  if (nk < 1.0f || na < 1.0f) return 0.0f;
  return d / (sqrtf(nk) * sqrtf(na));
}

static float matchEnergy(signal &k, signal &a) {
  float kx=0, ky=0, kz=0, ax=0, ay=0, az=0;
  for (int i = 0; i < SIGLEN; i++) {
    kx += (float)k.x[i]*k.x[i];  ky += (float)k.y[i]*k.y[i];  kz += (float)k.z[i]*k.z[i];
    ax += (float)a.x[i]*a.x[i];  ay += (float)a.y[i]*a.y[i];  az += (float)a.z[i]*a.z[i];
  }
  float kt = kx+ky+kz, at = ax+ay+az;
  if (kt < 1.0f || at < 1.0f) return 0.0f;
  float tvd = (fabsf(kx/kt - ax/at) + fabsf(ky/kt - ay/at) + fabsf(kz/kt - az/at)) * 0.5f;
  return 1.0f - tvd;
}

float match(signal &k, signal &a) {
  switch (algo) {
    case ALGO_COSINE:   return matchCosine(k, a);
    case ALGO_PEARSON:  return matchPearson(k, a);
    case ALGO_GRADIENT: return matchGradient(k, a);
    case ALGO_ENERGY:   return matchEnergy(k, a);
    default:            return matchCosine(k, a);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Dev: serial algorithm selection  (send '0'–'3')
// ─────────────────────────────────────────────────────────────────────────────
static void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c >= '0' && c < '0' + NUM_ALGOS) {
    algo = c - '0';
    Serial.print(F("[dev] algo="));
    switch (algo) {
      case ALGO_COSINE:   Serial.println(F("Cosine similarity"));   break;
      case ALGO_PEARSON:  Serial.println(F("Pearson correlation")); break;
      case ALGO_GRADIENT: Serial.println(F("Gradient cosine"));     break;
      case ALGO_ENERGY:   Serial.println(F("Energy ratio"));        break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  pinMode(PIN_BTN_L, INPUT);
  pinMode(PIN_BTN_R, INPUT);
  pinMode(PIN_LED,   OUTPUT);

  strip.begin();
  strip.setBrightness(120);
  strip.clear();
  strip.show();

  delay(100);
  SPI.begin();
  DDRB  |=  (1 << PB4);
  PORTB |=  (1 << PB4);
  PORTB &= ~(1 << PB4);
  SPI.transfer(CFG1REG);
  SPI.transfer(0x47);               // 100 Hz ODR, XYZ enabled
  PORTB |=  (1 << PB4);
  delay(1000);

  showIdle();
  Serial.println(F("Accelerometer Lock ready. L=Record  R=Unlock"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop  (10 ms tick → 100 Hz)
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  delay(10);
  readButtons();
  handleSerial();

  switch (state) {

  // ── IDLE ─────────────────────────────────────────────────────────────────
  case STATE_IDLE:
    if (btnL) {
      gestureIdx = 0;
      state      = STATE_REC_WAIT;
      showSequence(0, true);
      Serial.println(F("Record: press R to capture each gesture"));
    } else if (btnR) {
      if (keySet) {
        gestureIdx = 0;
        state      = STATE_UNL_WAIT;
        showSequence(0, false);
        Serial.println(F("Unlock: press R to capture each gesture"));
      } else {
        showAll(40, 0, 0);
        delay(300);
        showIdle();
        Serial.println(F("No key recorded yet!"));
      }
    }
    break;

  // ── RECORD WAIT ──────────────────────────────────────────────────────────
  case STATE_REC_WAIT:
    if (btnR) {
      key[gestureIdx].len = 0;
      state = STATE_REC_CAP;
    } else if (btnL) {
      state = STATE_IDLE;
      showIdle();
      Serial.println(F("Record cancelled."));
    }
    break;

  // ── RECORD CAPTURE ───────────────────────────────────────────────────────
  case STATE_REC_CAP:
    record(key[gestureIdx]);
    digitalWrite(PIN_LED, (key[gestureIdx].len & 8) ? HIGH : LOW);
    if (key[gestureIdx].len >= SIGLEN) {
      digitalWrite(PIN_LED, LOW);
      Serial.print(F("Gesture ")); Serial.print(gestureIdx); Serial.println(F(" saved."));
      gestureIdx++;
      if (gestureIdx < NUM_GESTURES) {
        state = STATE_REC_WAIT;
        showSequence(gestureIdx, true);   // advance: current=yellow, done=red, todo=green
      } else {
        keySet = true;
        showAll(20, 20, 20);              // white flash: full key saved
        delay(500);
        state = STATE_IDLE;
        showIdle();
        Serial.println(F("Key saved."));
      }
    }
    break;

  // ── UNLOCK WAIT ──────────────────────────────────────────────────────────
  case STATE_UNL_WAIT:
    if (btnR) {
      ans.len = 0;
      state   = STATE_UNL_CAP;
    } else if (btnL) {
      state = STATE_IDLE;
      showIdle();
      Serial.println(F("Unlock cancelled."));
    }
    break;

  // ── UNLOCK CAPTURE ───────────────────────────────────────────────────────
  case STATE_UNL_CAP:
    record(ans);
    digitalWrite(PIN_LED, (ans.len & 8) ? HIGH : LOW);
    if (ans.len >= SIGLEN) {
      digitalWrite(PIN_LED, LOW);
      float score = match(key[gestureIdx], ans);
      bool  ok    = (score >= THRESHOLDS[algo]);
      Serial.print(F("Gesture ")); Serial.print(gestureIdx);
      Serial.print(F(" score: ")); Serial.print(score, 4);
      Serial.println(ok ? F(" OK") : F(" FAIL"));

      if (!ok) {
        blinkCnt = 0;
        timer    = millis();
        state    = STATE_FAILED;
        showAll(60, 0, 0);
      } else {
        gestureIdx++;
        if (gestureIdx < NUM_GESTURES) {
          state = STATE_UNL_WAIT;
          showSequence(gestureIdx, false); // advance: done=red, current=blue, todo=green
        } else {
          state = STATE_UNLOCKED;
          showAll(0, 60, 0);              // all green: unlocked
          timer = millis();
          Serial.println(F("UNLOCKED"));
        }
      }
    }
    break;

  // ── UNLOCKED – steady green for 2 s ──────────────────────────────────────
  case STATE_UNLOCKED:
    if (millis() - timer >= 2000) {
      state = STATE_IDLE;
      showIdle();
    }
    break;

  // ── FAILED – blink red 3× then return to idle ────────────────────────────
  case STATE_FAILED:
    if (millis() - timer >= 200) {
      timer = millis();
      blinkCnt++;
      if (blinkCnt & 1) { strip.clear(); strip.show(); }
      else              showAll(60, 0, 0);
      if (blinkCnt >= 6) {
        blinkCnt = 0;
        state    = STATE_IDLE;
        showIdle();
        Serial.println(F("FAILED"));
      }
    }
    break;
  }
}
