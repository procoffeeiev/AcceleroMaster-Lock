#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#define READREG 0x80 // spi config
#define INCREG 0x40
#define AXREG 0x28
#define CFG1REG 0x20
#define SEGDIM 3   // key dimensions
#define SIGLEN 200 // key length
typedef struct {   // signal sequence struct
  int16_t x[SIGLEN];
  int16_t y[SIGLEN];
  int16_t z[SIGLEN];
  int16_t len = 0;
} signal;
int state = 0;
signal key;
signal ans;
void setup() {
  Serial.begin(9600);
  delay(100); // spi config
  SPI.begin();
  DDRB |= (1 << PB4);
  PORTB |= (1 << PB4);
  PORTB &= ~(1 << PB4);
  SPI.transfer(CFG1REG);
  SPI.transfer(0x47);
  PORTB |= (1 << PB4);
  delay(1000);
}
void loop() {
  delay(10); // 100hz sample rate
  switch (state) {
  case 1: // recording key
    break;
  case 2: // recording answer
    break;
  default: // unlocked waiting
    break;
  }
}
float match(signal &key, signal &ans) { // return similarity of 2 signals
  float d = 0, nkey = 0, nans = 0;
  for (int i = 0; i < SIGLEN; i++) {
    d += (float)key.x[i] * ans.x[i] + (float)key.y[i] * ans.y[i] +
         (float)key.z[i] * ans.z[i];
    nkey += (float)key.x[i] * key.x[i] + (float)key.y[i] * key.y[i] +
            (float)key.z[i] * key.z[i];
    nans += (float)ans.x[i] * ans.x[i] + (float)ans.y[i] * ans.y[i] +
            (float)ans.z[i] * ans.z[i];
  }
  return d / (sqrtf(nkey) * sqrtf(nans));
}
void record(signal &a) { // record one sample
  PORTB &= ~(1 << PB4);
  SPI.transfer(AXREG | READREG | INCREG);
  uint8_t val1 = SPI.transfer(0xFF);
  uint8_t val2 = SPI.transfer(0xFF);
  a.x[a.len] = (val2 << 8) | val1;
  val1 = SPI.transfer(0xFF);
  val2 = SPI.transfer(0xFF);
  a.y[a.len] = (val2 << 8) | val1;
  val1 = SPI.transfer(0xFF);
  val2 = SPI.transfer(0xFF);
  a.z[a.len] = (val2 << 8) | val1;
  PORTB |= (1 << PB4);
  a.len++;
}
