#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#define READREG 0x80
#define INCREG  0x40
#define AXREG   0x28
#define CFG1REG 0x20
#define SIGLEN 50
#define NUM_GESTURES 3

#define ALGO_COSINE  // swap to ALGO_PEARSON, ALGO_GRADIENT, or ALGO_ENERGY

typedef struct {
    int16_t x[SIGLEN];
    int16_t y[SIGLEN];
    int16_t z[SIGLEN];
    int16_t len = 0;
} Signal;

Adafruit_NeoPixel strip(10, 17, NEO_GRB + NEO_KHZ800);

static bool btnL, btnR, lastL = false, lastR = false;
static bool keySet = false;
static int gi = 0;
static Signal key[NUM_GESTURES];
static Signal ans;

// ── Forward declarations ──────────────────────────────────────────────────────
static void tick();
static void showAll(uint8_t r, uint8_t g, uint8_t b);
static void showGestures(int cur, int len, bool isRec);
static void readAccel(Signal &s);
static bool match(Signal &k, Signal &a);

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    DDRD &= ~(1 << PD4); PORTD &= ~(1 << PD4);
    DDRF &= ~(1 << PF6); PORTF &= ~(1 << PF6);
    DDRC |= (1 << PC7);

    strip.begin(); strip.setBrightness(120);
    delay(100); SPI.begin();

    DDRB |= (1 << PB4); PORTB |= (1 << PB4);
    PORTB &= ~(1 << PB4);
    SPI.transfer(CFG1REG); SPI.transfer(0x47);
    PORTB |= (1 << PB4);
    delay(1000);

    gi = 0; showGestures(0, -1, true);

REC_WAIT:
    tick();
    if (btnL) { key[gi].len = 0; goto REC_CAP; }
    if (btnR && keySet) { showAll(0, 60, 0); goto UNLOCKED; }
    if (btnR) { gi = 0; showGestures(0, -1, true); }
    goto REC_WAIT;

REC_CAP:
    tick();
    readAccel(key[gi]);
    if (key[gi].len & 8) PORTC |= (1 << PC7); else PORTC &= ~(1 << PC7);
    if (key[gi].len % 10 == 0) showGestures(gi, key[gi].len, true);
    if (key[gi].len < SIGLEN) goto REC_CAP;
    PORTC &= ~(1 << PC7);
    if (++gi < NUM_GESTURES) { showGestures(gi, -1, true); goto REC_WAIT; }
    keySet = true; showAll(20, 20, 20); delay(500); showAll(15, 0, 0);
    goto LOCKED;

LOCKED:
    tick();
    if (btnR) { gi = 0; showGestures(0, -1, false); goto UNL_WAIT; }
    goto LOCKED;

UNLOCKED:
    tick();
    if (btnL) { showAll(15, 0, 0); goto LOCKED; }
    if (btnR) { gi = 0; showGestures(0, -1, true); goto REC_WAIT; }
    goto UNLOCKED;

UNL_WAIT:
    tick();
    if (btnR) { ans.len = 0; goto UNL_CAP; }
    if (btnL) { showAll(15, 0, 0); goto LOCKED; }
    goto UNL_WAIT;

UNL_CAP:
    tick();
    readAccel(ans);
    if (ans.len & 8) PORTC |= (1 << PC7); else PORTC &= ~(1 << PC7);
    if (ans.len % 10 == 0) showGestures(gi, ans.len, false);
    if (ans.len < SIGLEN) goto UNL_CAP;
    PORTC &= ~(1 << PC7);
    if (!match(key[gi], ans)) { showAll(60, 0, 0); goto FAILED; }
    if (++gi < NUM_GESTURES) { showGestures(gi, -1, false); goto UNL_WAIT; }
    showAll(0, 60, 0);
    goto UNLOCKED;

FAILED:
    {
        int cnt = 0, t = 0;
        while (cnt < 6) {
            tick();
            if (++t >= 20) {
                t = 0;
                if (cnt & 1) { strip.clear(); strip.show(); } else showAll(60, 0, 0);
                cnt++;
            }
        }
    }
    gi = 0; showGestures(0, -1, false);
    goto UNL_WAIT;
}

void loop() {}

// ─────────────────────────────────────────────────────────────────────────────

static void tick() {
    delay(10);
    bool L = PIND & (1 << PD4);
    bool R = PINF & (1 << PF6);
    btnL = L && !lastL; btnR = R && !lastR;
    lastL = L; lastR = R;
}

static void showAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < 10; i++)
        strip.setPixelColor(i, strip.Color(r, g, b));
    strip.show();
}

static void showGestures(int cur, int len, bool isRec) {
    uint32_t done = isRec ? strip.Color(20, 0, 0) : strip.Color(0, 20, 0);
    uint32_t todo = isRec ? strip.Color(0, 20, 0) : strip.Color(20, 0, 0);
    strip.clear();
    for (int g = 0; g < NUM_GESTURES; g++) {
        for (int p = 0; p < 3; p++) {
            uint32_t c = g < cur ? done
                       : g > cur ? todo
                       : (len < 0 || p < len*3/SIGLEN) ? strip.Color(20, 20, 0)
                       : strip.Color(5, 5, 0);
            strip.setPixelColor(g*3+p, c);
        }
    }
    strip.show();
}

static void readAccel(Signal &s) {
    PORTB &= ~(1 << PB4);
    SPI.transfer(AXREG | READREG | INCREG);
    uint8_t lo, hi;
    lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF); s.x[s.len] = (int16_t)((hi << 8) | lo);
    lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF); s.y[s.len] = (int16_t)((hi << 8) | lo);
    lo = SPI.transfer(0xFF); hi = SPI.transfer(0xFF); s.z[s.len] = (int16_t)((hi << 8) | lo);
    PORTB |= (1 << PB4);
    s.len++;
}

static bool match(Signal &k, Signal &a) {
    float d = 0, nk = 0, na = 0;
#if defined(ALGO_PEARSON)
    for (int ax = 0; ax < 3; ax++) {
        int16_t *kA = ax==0?k.x:ax==1?k.y:k.z;
        int16_t *aA = ax==0?a.x:ax==1?a.y:a.z;
        float mk = 0, ma = 0;
        for (int i = 0; i < SIGLEN; i++) { mk += kA[i]; ma += aA[i]; }
        mk /= SIGLEN; ma /= SIGLEN;
        float cov = 0, vk = 0, va = 0;
        for (int i = 0; i < SIGLEN; i++) {
            float dk = kA[i]-mk, da = aA[i]-ma;
            cov += dk*da; vk += dk*dk; va += da*da;
        }
        if (vk < 1 || va < 1) continue;
        d += cov/sqrtf(vk*va); nk += 1; na += 1;
    }
    return nk >= 1 && d/nk >= 0.75f;
#elif defined(ALGO_GRADIENT)
    for (int i = 0; i < SIGLEN-1; i++) {
        float kx=k.x[i+1]-k.x[i], ky=k.y[i+1]-k.y[i], kz=k.z[i+1]-k.z[i];
        float ax=a.x[i+1]-a.x[i], ay=a.y[i+1]-a.y[i], az=a.z[i+1]-a.z[i];
        d += kx*ax+ky*ay+kz*az; nk += kx*kx+ky*ky+kz*kz; na += ax*ax+ay*ay+az*az;
    }
    return nk >= 1 && na >= 1 && d/(sqrtf(nk)*sqrtf(na)) >= 0.80f;
#elif defined(ALGO_ENERGY)
    float kx=0,ky=0,kz=0,ax=0,ay=0,az=0;
    for (int i = 0; i < SIGLEN; i++) {
        kx+=(float)k.x[i]*k.x[i]; ky+=(float)k.y[i]*k.y[i]; kz+=(float)k.z[i]*k.z[i];
        ax+=(float)a.x[i]*a.x[i]; ay+=(float)a.y[i]*a.y[i]; az+=(float)a.z[i]*a.z[i];
    }
    float kt=kx+ky+kz, at=ax+ay+az;
    return kt >= 1 && at >= 1 &&
           1-(fabsf(kx/kt-ax/at)+fabsf(ky/kt-ay/at)+fabsf(kz/kt-az/at))*0.5f >= 0.80f;
#else
    for (int i = 0; i < SIGLEN; i++) {
        d  += (float)k.x[i]*a.x[i]+(float)k.y[i]*a.y[i]+(float)k.z[i]*a.z[i];
        nk += (float)k.x[i]*k.x[i]+(float)k.y[i]*k.y[i]+(float)k.z[i]*k.z[i];
        na += (float)a.x[i]*a.x[i]+(float)a.y[i]*a.y[i]+(float)a.z[i]*a.z[i];
    }
    return nk >= 1 && na >= 1 && d/(sqrtf(nk)*sqrtf(na)) >= 0.85f;
#endif
}
