# AcceleroMaster-Lock

An accelerometer-based 3-gesture combination lock running on the Adafruit Circuit Playground Classic (ATmega32u4).

## Hardware

| Component | Detail |
|---|---|
| Board | Adafruit Circuit Playground Classic |
| MCU | ATmega32u4 @ 8MHz, 2.5KB SRAM, 28KB Flash |
| Accelerometer | LIS3DH (built-in), I2C at address 0x18 |
| Buttons | Left = pin 19, Right = pin 5 (active-HIGH) |
| NeoPixels | 10 RGB LEDs on pin 17 |
| Red LED | Pin 13 |

## How It Works

The lock records a sequence of 3 motion gestures as a key. To unlock, the user reproduces those same 3 gestures. Each gesture is a 500ms accelerometer recording (50 samples at 100Hz across X/Y/Z axes). Unlock success is determined by comparing each recorded gesture against the corresponding key gesture using **cosine similarity** — a value of 1.0 is a perfect match. All 3 gestures must score at or above the threshold (default: 0.85).

## Usage

### Recording a Key
1. Press **Left** to enter record mode.
2. For each of the 3 gestures: perform your motion, then press **Right** to capture it.
3. The red LED blinks during the 500ms capture window.
4. After all 3 gestures, pixels flash white and the key is saved.

### Unlocking
1. Press **Right** to enter unlock mode (only available after a key is recorded).
2. Reproduce each gesture and press **Right** to capture it, as above.
3. Each gesture is compared immediately:
   - **Pass:** that gesture's NeoPixels turn green, next gesture is prompted.
   - **Fail:** all NeoPixels flash red for 3 seconds, returns to idle.
4. All 3 pass → all 10 NeoPixels solid green for 3 seconds (unlocked).

### LED Feedback

| Color | Meaning |
|---|---|
| Blue (pixels 0-2/3-5/6-8) | Waiting for next gesture in record mode |
| Cyan (pixels 0-2/3-5/6-8) | Waiting for next gesture in unlock mode |
| Red LED blinking | Gesture capture in progress (500ms) |
| Green (3 pixels) | Individual gesture captured / matched |
| White (all) | Key successfully saved |
| Green (all) | Unlock successful |
| Red (all) | Unlock failed |

NeoPixels 0-2 correspond to gesture 1, 3-5 to gesture 2, 6-8 to gesture 3. Pixel 9 is a mode indicator (blue = record mode, cyan = unlock mode).

## Configuration

All tuneable parameters are `#define`s at the top of `main.cpp`:

| Define | Default | Description |
|---|---|---|
| `SIGLEN` | 50 | Samples per gesture (50 = 500ms at 100Hz) |
| `MATCH_THRESHOLD` | 0.85 | Minimum cosine similarity to accept a gesture match |
| `NUM_GESTURES` | 3 | Number of gestures in the combination |

**Tuning the threshold:** Open the serial monitor at 9600 baud. The similarity score for each gesture is printed during unlock attempts. If legitimate attempts fail, lower the threshold toward 0.80. If wrong gestures are accepted, raise it toward 0.90.

## Key Functions

| Function | Description |
|---|---|
| `record(signal &a)` | Reads one X/Y/Z sample from the LIS3DH over I2C and appends it to signal `a` |
| `match(signal &k, signal &a)` | Returns cosine similarity [0, 1] between key signal `k` and attempt signal `a` |
| `readButton(pin)` | Returns true on the rising edge of a button press (debounced by the 10ms loop cadence) |
| `markGesture(idx, color)` | Lights the 3 NeoPixels corresponding to gesture `idx` with the given color |

## State Machine

```
IDLE ──Left──────────────────→ RECORD_WAIT
IDLE ──Right (key set)───────→ UNLOCK_WAIT
RECORD_WAIT ──Right──────────→ RECORD_CAP
RECORD_CAP ──50 samples──────→ RECORD_WAIT (gestures 1-2) / IDLE (gesture 3, key saved)
UNLOCK_WAIT ──Right──────────→ UNLOCK_CAP
UNLOCK_CAP ──50 samples──────→ UNLOCK_WAIT (pass, gestures 1-2)
                              → IDLE (fail → red LEDs / pass all 3 → green LEDs)
```

## Build

```bash
pio run           # compile
pio run -t upload # compile and flash
```

RAM usage at default settings: ~88% (2256 / 2560 bytes). Do not increase `SIGLEN` or `NUM_GESTURES` without verifying RAM stays under 2560 bytes.
