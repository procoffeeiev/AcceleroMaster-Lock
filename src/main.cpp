// KinetiKey: 3-gesture "lock" using the board LIS3DH accelerometer only (XYZ).
// Gyro is not read; course allows accelerometer and/or gyro.
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

#define NEO_COUNT  10
#define READREG  0x80
#define INCREG   0x40
#define AXREG    0x28
#define CFG1REG  0x20

// Left btn=PD4  Right btn=PF6  LED=PC7
#define BTN_L  (PIND & (1<<PD4))
#define BTN_R  (PINF & (1<<PF6))
#define LED_ON  PORTC |=  (1<<PC7)
#define LED_OFF PORTC &= ~(1<<PC7)

#define SIGLEN        50
#define NUM_GESTURES   3
// Built-in magnetic speaker (Circuit Playground Classic).
#define SPEAKER_PIN 5

static void beep(uint16_t hz, uint16_t ms) {
  tone(SPEAKER_PIN, hz, ms);
  delay((uint32_t)ms + 15U);
  noTone(SPEAKER_PIN);
}

typedef struct {
  int16_t x[SIGLEN], y[SIGLEN], z[SIGLEN];
  int16_t len;
} signal;

// ── State machine ─────────────────────────────────────────────────────────────
// LOCKED    all red.    L=set pw (first time)  R=start unlock attempt
// REC_WAIT  sequence display.  L=capture next  R=cancel → LOCKED
// REC_CAP   capturing.  done→REC_WAIT or LOCKED (white flash)
// UNL_WAIT  sequence display.  R=capture next  L=cancel → LOCKED
// UNL_CAP   capturing.  match→UNL_WAIT or UNLOCKED  fail→FAILED
// FAILED    blink red → UNL_WAIT gi=0  (first 3 remain yellow, retry)
// UNLOCKED  all green.  L=lock  R=change pw → REC_WAIT
#define STATE_LOCKED    0
#define STATE_REC_WAIT  1
#define STATE_REC_CAP   2
#define STATE_UNL_WAIT  3
#define STATE_UNL_CAP   4
#define STATE_FAILED    5
#define STATE_UNLOCKED  6

#define ALGO_COSINE   0
#define ALGO_PEARSON  1
#define ALGO_GRADIENT 2
#define ALGO_ENERGY   3
#define NUM_ALGOS     4
// Min score to pass one gesture. Each array slot is a different metric: scales are related but not
// identical. Cosine / Pearson / gradient behave like correlation (often ~0.6–1.0 for good match).
// Energy is 1 - 0.5 * sum|Δfraction per axis| (see matchEnergy). Retune the slot for each mode if
// you change Serial algo (0–3) and see false rejects or false accepts.
const float THRESHOLDS[NUM_ALGOS] = {0.85f, 0.75f, 0.80f, 0.75f};

static int           state    = STATE_LOCKED;
static int           algo     = ALGO_COSINE;
static bool          keySet   = false;
static int           gi       = 0;
static unsigned long timer    = 0;
static int           blinkCnt = 0;
static bool          lastL    = false, lastR = false;
static bool          btnL     = false, btnR  = false;

static signal key[NUM_GESTURES];
static signal ans;

Adafruit_NeoPixel strip(NEO_COUNT, 17, NEO_GRB + NEO_KHZ800);

static void showAll(uint8_t r, uint8_t g, uint8_t b) {
  for (int i=0;i<NEO_COUNT;i++) strip.setPixelColor(i,strip.Color(r,g,b));
  strip.show();
}

// Record: recorded=red  current=yellow  unrecorded=green
static void showRecSeq(int cur) {
  strip.clear();
  for (int g=0;g<NUM_GESTURES;g++) {
    uint32_t c = g<cur?strip.Color(20,0,0):g==cur?strip.Color(20,20,0):strip.Color(0,20,0);
    strip.setPixelColor(g*3,c); strip.setPixelColor(g*3+1,c); strip.setPixelColor(g*3+2,c);
  }
  strip.show();
}

// Unlock: matched=green  current=yellow  locked=red
static void showUnlSeq(int cur) {
  strip.clear();
  for (int g=0;g<NUM_GESTURES;g++) {
    uint32_t c = g<cur?strip.Color(0,20,0):g==cur?strip.Color(20,20,0):strip.Color(20,0,0);
    strip.setPixelColor(g*3,c); strip.setPixelColor(g*3+1,c); strip.setPixelColor(g*3+2,c);
  }
  strip.show();
}

// During capture: fill current gesture pixels as samples arrive
static void showCapProgress(int cur, int16_t len, bool isRec) {
  strip.clear();
  for (int g=0;g<NUM_GESTURES;g++) {
    if (g<cur) {
      uint32_t c=isRec?strip.Color(20,0,0):strip.Color(0,20,0);
      strip.setPixelColor(g*3,c);strip.setPixelColor(g*3+1,c);strip.setPixelColor(g*3+2,c);
    } else if (g==cur) {
      int f=(len*3)/SIGLEN;
      for (int p=0;p<3;p++)
        strip.setPixelColor(g*3+p, p<f?strip.Color(20,20,0):strip.Color(5,5,0));
    } else {
      uint32_t c=isRec?strip.Color(0,20,0):strip.Color(20,0,0);
      strip.setPixelColor(g*3,c);strip.setPixelColor(g*3+1,c);strip.setPixelColor(g*3+2,c);
    }
  }
  strip.show();
}

static uint8_t spiRead8(uint8_t reg) {
  PORTB &= ~(1 << PB4);
  SPI.transfer(reg | READREG);
  uint8_t v = SPI.transfer(0xFF);
  PORTB |= (1 << PB4);
  return v;
}

static void record(signal &a) {
  PORTB &= ~(1<<PB4);
  SPI.transfer(AXREG|READREG|INCREG);
  uint8_t lo,hi;
  lo=SPI.transfer(0xFF);hi=SPI.transfer(0xFF);a.x[a.len]=(int16_t)((hi<<8)|lo);
  lo=SPI.transfer(0xFF);hi=SPI.transfer(0xFF);a.y[a.len]=(int16_t)((hi<<8)|lo);
  lo=SPI.transfer(0xFF);hi=SPI.transfer(0xFF);a.z[a.len]=(int16_t)((hi<<8)|lo);
  PORTB |= (1<<PB4);
  a.len++;
}

static float matchCosine(signal &k,signal &a){
  float d=0,nk=0,na=0;
  for(int i=0;i<SIGLEN;i++){d+=(float)k.x[i]*a.x[i]+(float)k.y[i]*a.y[i]+(float)k.z[i]*a.z[i];nk+=(float)k.x[i]*k.x[i]+(float)k.y[i]*k.y[i]+(float)k.z[i]*k.z[i];na+=(float)a.x[i]*a.x[i]+(float)a.y[i]*a.y[i]+(float)a.z[i]*a.z[i];}
  if(nk<1||na<1)return 0;return d/(sqrtf(nk)*sqrtf(na));
}
static float matchPearson(signal &k,signal &a){
  int16_t *kA[3]={k.x,k.y,k.z},*aA[3]={a.x,a.y,a.z};float rSum=0;int cnt=0;
  for(int ax=0;ax<3;ax++){float mk=0,ma=0;for(int i=0;i<SIGLEN;i++){mk+=kA[ax][i];ma+=aA[ax][i];}mk/=SIGLEN;ma/=SIGLEN;float cov=0,vk=0,va=0;for(int i=0;i<SIGLEN;i++){float dk=kA[ax][i]-mk,da=aA[ax][i]-ma;cov+=dk*da;vk+=dk*dk;va+=da*da;}if(vk<1||va<1)continue;rSum+=cov/sqrtf(vk*va);cnt++;}
  return cnt>0?rSum/cnt:0;
}
static float matchGradient(signal &k,signal &a){
  float d=0,nk=0,na=0;
  for(int i=0;i<SIGLEN-1;i++){float kx=k.x[i+1]-k.x[i],ky=k.y[i+1]-k.y[i],kz=k.z[i+1]-k.z[i],ax=a.x[i+1]-a.x[i],ay=a.y[i+1]-a.y[i],az=a.z[i+1]-a.z[i];d+=kx*ax+ky*ay+kz*az;nk+=kx*kx+ky*ky+kz*kz;na+=ax*ax+ay*ay+az*az;}
  if(nk<1||na<1)return 0;return d/(sqrtf(nk)*sqrtf(na));
}
static float matchEnergy(signal &k,signal &a){
  float kx=0,ky=0,kz=0,ax=0,ay=0,az=0;
  for(int i=0;i<SIGLEN;i++){kx+=(float)k.x[i]*k.x[i];ky+=(float)k.y[i]*k.y[i];kz+=(float)k.z[i]*k.z[i];ax+=(float)a.x[i]*a.x[i];ay+=(float)a.y[i]*a.y[i];az+=(float)a.z[i]*a.z[i];}
  float kt=kx+ky+kz,at=ax+ay+az;if(kt<1||at<1)return 0;
  return 1-(fabsf(kx/kt-ax/at)+fabsf(ky/kt-ay/at)+fabsf(kz/kt-az/at))*0.5f;
}
float match(signal &k,signal &a){
  switch(algo){case ALGO_PEARSON:return matchPearson(k,a);case ALGO_GRADIENT:return matchGradient(k,a);case ALGO_ENERGY:return matchEnergy(k,a);default:return matchCosine(k,a);}
}

void setup() {
  Serial.begin(9600);
  pinMode(SPEAKER_PIN, OUTPUT);
  noTone(SPEAKER_PIN);
  DDRD &= ~(1<<PD4);
  PORTD |= (1<<PD4);   // input + pull-up on left button
  DDRF &= ~(1<<PF6);
  PORTF |= (1<<PF6);   // input + pull-up on right button
  DDRC|=(1<<PC7);
  strip.begin();strip.setBrightness(120);
  delay(100);SPI.begin();
  DDRB|=(1<<PB4);PORTB|=(1<<PB4);
  PORTB&=~(1<<PB4);SPI.transfer(CFG1REG);SPI.transfer(0x47);PORTB|=(1<<PB4);
  delay(1000);
  gi=0; showRecSeq(0); state=STATE_REC_WAIT;
  Serial.println(F("No key — set password."));
}

void loop() {
  delay(10);
  bool curL=BTN_L,curR=BTN_R;
  btnL=curL&&!lastL;btnR=curR&&!lastR;
  lastL=curL;lastR=curR;

  if(Serial.available()){char c=Serial.read();if(c>='0'&&c<'0'+NUM_ALGOS){algo=c-'0';Serial.print(F("[dev] algo="));Serial.println(algo);}}

  switch(state){

  case STATE_LOCKED:
    if(btnR){ gi=0; showUnlSeq(0); state=STATE_UNL_WAIT; }
    break;

  case STATE_UNLOCKED:
    if(btnL){                               // lock
      state=STATE_LOCKED; showAll(15,0,0);
    }else if(btnR){                         // change password
      gi=0; showRecSeq(0); state=STATE_REC_WAIT;
    }
    break;

  case STATE_REC_WAIT:                      // L=capture  R=cancel
    if(btnL){
      key[gi].len=0; state=STATE_REC_CAP;
    }else if(btnR){
      if(keySet){ state=STATE_UNLOCKED; showAll(0,60,0); } // cancel pw change → back unlocked
      else{ gi=0; showRecSeq(0); }                         // no pw yet: reset, can't escape
    }
    break;

  case STATE_REC_CAP:
    record(key[gi]);
    if(key[gi].len&8)LED_ON;else LED_OFF;
    if(key[gi].len%10==0) showCapProgress(gi,key[gi].len,true);
    if(key[gi].len>=SIGLEN){
      LED_OFF;
      Serial.print(F("G"));Serial.print(gi);Serial.println(F(" saved"));
      beep((uint16_t)(520 + gi * 120), 55); // short cue per recorded gesture
      if(++gi<NUM_GESTURES){
        showRecSeq(gi); state=STATE_REC_WAIT;
      }else{
        keySet=true; showAll(20,20,20); delay(500);
        state=STATE_LOCKED; showAll(15,0,0);
        Serial.println(F("Key saved."));
        beep(784, 70);
        beep(988, 90); // full password stored
      }
    }
    break;

  case STATE_UNL_WAIT:                      // R=capture  L=cancel
    if(btnR){
      ans.len=0; state=STATE_UNL_CAP;
    }else if(btnL){
      state=STATE_LOCKED; showAll(15,0,0);
    }
    break;

  case STATE_UNL_CAP:
    record(ans);
    if(ans.len&8)LED_ON;else LED_OFF;
    if(ans.len%10==0) showCapProgress(gi,ans.len,false);
    if(ans.len>=SIGLEN){
      LED_OFF;
      float score=match(key[gi],ans);
      bool ok=score>=THRESHOLDS[algo];
      Serial.print(F("G"));Serial.print(gi);Serial.print(' ');Serial.print(score,4);Serial.println(ok?F(" OK"):F(" FAIL"));
      if(!ok){
        beep(180, 160); // failed gesture
        blinkCnt=0;timer=millis();state=STATE_FAILED;showAll(60,0,0);
      }else if(++gi<NUM_GESTURES){
        beep(880, 45); // gesture passed
        showUnlSeq(gi); state=STATE_UNL_WAIT;
      }else{
        state=STATE_UNLOCKED; showAll(0,60,0);
        Serial.println(F("UNLOCKED"));
        beep(523, 50);
        beep(659, 50);
        beep(784, 80); // unlocked triplet
      }
    }
    break;

  case STATE_FAILED:
    if(millis()-timer>=200){
      timer=millis();
      if(blinkCnt&1){strip.clear();strip.show();}else showAll(60,0,0);
      if(++blinkCnt>=6){
        blinkCnt=0;
        gi=0; showUnlSeq(0); state=STATE_UNL_WAIT;  // first 3 yellow, ready to retry
        Serial.println(F("FAILED"));
      }
    }
    break;
  }
}
