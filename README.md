# 🍓 raspberry_grabber_arduino

Firmware for an **Arduino UNO** that runs a **raspberry gripper + ripeness gate + size classifier + trapdoor drop** system.

The intended, “honest” workflow is:

1) **Detect ripeness** (LDR + controlled illumination).  
2) **If RIPE → grasp** (FSR-based gentle contact detection).  
3) **Estimate diameter** from gripper servo angle and classify **BIG vs SMALL**.  
4) **Tilt a classifier ramp** (see-saw) to route into the correct bin.  
5) Optionally **open trapdoor** and play a melody for feedback.

> Interaction is **Serial-only**. The button input remains from debugging during incremental development and is not used in the final flow.

---

## 1) Hardware Summary

### Actuators (3 servos)
- **Gripper servo (standard size)**: opens/closes gripper to grab the berry.
- **Classifier servo (standard size)**: tilts a flat ramp (see-saw) to route berry into the **BIG** or **SMALL** bin.
- **Trapdoor servo (standard size)**: unlocks/opens the drop door to release berries into the collecting bins.

### Sensors
- **FSR**: Force Sensing Resistor (the one that comes with its board + onboard voltage divider).
- **LDR + illumination LED**: reflectance measurement with ambient subtraction (LED OFF vs LED ON).

### Feedback
- 4 LEDs: **RIPE**, **UNRIPE**, **BIG**, **SMALL**
- **Passive buzzer** driven by `tone()`

### Power
- **USB only** (Arduino UNO powered by USB).

---

## 2) Pin Map (Arduino UNO)

| Subsystem | Signal | Pin |
|---|---:|---:|
| FSR | Analog input | A0 |
| LDR | Analog input | A1 |
| Gripper servo | PWM | 9 |
| Classifier servo | PWM | 10 |
| Trapdoor servo | PWM | 11 |
| Illumination LED (for LDR) | Digital out | 12 |
| BIG LED | Digital out | 2 |
| SMALL LED | Digital out | 3 |
| RIPE LED | Digital out | 5 |
| UNRIPE LED | Digital out | 4 |
| Buzzer (passive) | Digital out | 6 |
| Debug button (unused in final UI) | Input pullup | 7 |
| `releasePin` (legacy) | Unused | 8 |

Notes:
- Illumination LED uses a **220 Ω** resistor.
- LDR resistor value was not recorded.

---

## 3) Dependencies

- Arduino IDE
- `Servo.h`
- `pitches.h` (installed/available via Arduino IDE examples; provides note constants like `NOTE_E5`)

---

## 4) Serial Commands (User Interface)

Open Serial Monitor @ **9600 baud** and send:

| Key | Action | Intended use |
|---|---|---|
| `A` | Run ripeness detection | Normal operation. If **RIPE**, transitions automatically to grasping. |
| `G` | Manual grasp | **Debug only** for validating FSR + gripping behavior. |
| `R` | Release | Opens trapdoor and (optionally) plays melody + LED flashing. |
| `Z` | Force IDLE | Returns to idle state from any state (where checked). |

### “No cheating” note about `G`
`G` exists only to debug and validate the gripping/FSR logic. The intended run always starts with `A` (DETECTION) so grasping is not done manually to bypass ripeness detection.

---

## 5) Finite State Machine (FSM)

### States (as implemented)
- `IDLE`
- `DETECTION`
- `GRASPING`
- `GRABBING` (back-off)
- `GRABBED` (diameter mapping + size classification + reset)
- `RELEASE`

### FSM Diagram (copy-safe)

~~~text
IDLE
  |  'A'
  v
DETECTION  -- (UNRIPE or UNCERTAIN) -->  IDLE
  |
  |  (RIPE)
  v
GRASPING  -- (contact detected OR fully closed) -->  GRABBING  -->  GRABBED  -->  IDLE

Debug path:
IDLE  -- 'G' -->  GRASPING   (debug-only)

Release path:
IDLE  -- 'R' -->  RELEASE  -- 'Z' -->  IDLE
~~~

---

## 6) How the Code Works (State-by-State, Code-Accurate)

This section describes what the **current code** actually does, including guards and quirks.

### 6.1 IDLE
- Forces gripper open:
  - `currentAngle = openAngle` (140°)
- Resets key state variables:
  - `ripeness = UNCERTAIN`
  - `raspberrySize = NONE`
  - `contactDetected = false`
- Uses a one-time init gate `taco` to “home” mechanisms:
  - classifier → `noneAngle` (90°)
  - trapdoor → `lockAngle` (100°)
  - turns all indicator LEDs on briefly
- Blinks the four LEDs at ~1 Hz using a timer:
  - toggles every `toggleInterval = 1000 ms`
- Waits for Serial commands:
  - `A` → `DETECTION`
  - `G` → `GRASPING` (debug)
  - `R` → `RELEASE`

---

### 6.2 DETECTION (Ripeness decision)
DETECTION performs a **single** reflectance measurement (`net`) and classifies ripeness using hysteresis.

**Measurement method (ambient subtraction):**
1) LED OFF → measure ambient LDR
2) LED ON → measure illuminated LDR
3) `net = onVal - ambient` (clamped to 0 if negative)

Sampling constants:
- `ldr_samples = 50`
- `ldr_delay_ms = 5`
- `ldr_settle_ms = 50`

**Classification (hysteresis):**
- if `net > highThreshold` (75) → `UNRIPE`
- else if `net < lowThreshold` (65) → `RIPE`
- else → `UNCERTAIN`

Transitions:
- RIPE:
  - turns RIPE LED on
  - sets `ignoreContactOnce = true` to prevent false contact detection on first GRASPING loop
  - → `GRASPING`
- UNRIPE:
  - turns UNRIPE LED on
  - → `IDLE`
- UNCERTAIN:
  - prints status
  - → `IDLE`

Serial prints include:
- `LDR net = <value>`
- `Ripeness: RIPE/UNRIPE/UNCERTAIN`

---

### 6.3 GRASPING (Gentle adaptive closing using FSR)
GRASPING closes the gripper progressively, using FSR to avoid excessive force.

**FSR processing each loop:**
- `raw = analogRead(fsrPin)`
- low-pass filter:
  - `fsrFiltered = fsrFiltered + alpha * (raw - fsrFiltered)`
  - `alpha = 0.3`

**Derivative:**
- `dRaw = raw - rawPrev`

**Normalized force:**
- `norm = (fsrFiltered - rawMin) / (rawMax - rawMin)` → constrained to [0, 1]

**Adaptive closing rate:**
- `rate = maxRateDegPerLoop * (1 - norm)`
- `maxRateDegPerLoop = 2.0 deg/loop`
- Meaning: closes faster when force is low, slows as force increases.

**Important guard (`ignoreContactOnce`):**
- On entry from DETECTION→GRASPING, code skips one iteration of contact detection:
  - sets `rawPrev = raw`
  - clears `ignoreContactOnce`
  - `break;`

**Contact detection trigger (two-condition):**
- `(raw > contactRawThres) && (dRaw > contactDeltaThres)`
- `contactRawThres = 17`
- `contactDeltaThres = 9`

If contact is detected:
- `contactDetected = true`
- → `GRABBING`

Fail-safe closure:
- If `currentAngle <= closedAngle` (60°), transitions to `GRABBING` even without detected contact.

---

### 6.4 GRABBING (Back-off step)
After detecting contact (or reaching fully closed), GRABBING performs a quick “back-off” to reduce squeezing.

Current implementation:
- `currentAngle = min(currentAngle + backoffDeg, openAngle)`
- `backoffDeg = 12`
- `delay(200)`
- → `GRABBED`

Note:
- A multi-tick back-off exists in commented code (`backoffTicksNeeded`), but the final behavior is a **single back-off**.

---

### 6.5 GRABBED (Diameter estimation, size classification, reset)
GRABBED computes an approximate diameter from the final gripper angle and classifies BIG/SMALL.

**Angle → diameter mapping:**
- The code maps angle linearly from:
  - `closedAngle (60°)` → `minDiameter (15 mm)`
  - `openAngle (140°)` → `maxDiameter (62 mm)`

The helper:
- `mapRaspberry(angleDeg)` uses Arduino `map()` internally (integer mapping).

The code applies a small correction after back-off:
- `raspberryDiam = mapRaspberry(currentAngle - backoffDeg/4)`

**Size classification rule (as implemented):**
- `BIG` if `diameter > 20.0`
- else `SMALL`

Real-world size meaning (project spec):
- **SMALL ≈ 20 mm or less**
- **BIG ≈ ~30 mm**
(The implemented threshold is 20 mm; adjust if you later want a different boundary.)

**Classifier ramp actuation:**
- BIG → classifier servo to `bigAngle = 140°`
- SMALL → classifier servo to `smallAngle = 50°`
- LEDs:
  - BIG LED ON if BIG
  - SMALL LED ON if SMALL

Special case (“nothing grabbed” heuristic):
- If `(raw < contactRawThres) && !contactDetected` it prints:
  - `"Nothing Grabbed, just closed"`
  - and sets classifier to `smallAngle`.

Reset behavior (as coded):
- waits 4 s
- opens gripper, `currentAngle = openAngle`
- waits 1.5 s
- sets `taco = true`
- → `IDLE`

---

### 6.6 RELEASE (Trapdoor + melody + LED choreography)
RELEASE opens the trapdoor and can play a melody with synchronized LED flashes.

Trapdoor:
- `trapdoor.write(unlockAngle)` where `unlockAngle = 10°`

Melody:
- Uses `melody[]` and `durations[]`
- For each note:
  - `duration = 1000 / durations[note]`
  - `tone(buzzerPin, melody[note], duration)`
  - pause `duration * 1.70`
  - LEDs flash based on note:
    - `NOTE_D5` → RIPE LED
    - `NOTE_E5` → UNRIPE LED
    - `NOTE_F5` → BIG LED
    - `NOTE_G5` → SMALL LED

Important implementation detail:
- The melody loop is guarded by `if(!taco) { ... }`.
- In many runs, `taco` is set `true` before entering RELEASE, so the melody may not execute unless `taco` is false at entry.
- If you want melody to always play on `R`, remove that guard.

Exit:
- `Z` returns to `IDLE`.

---

## 7) Key Parameters (Final / Validated Values)

### Servos
- Gripper:
  - `openAngle = 140`
  - `closedAngle = 60`
- Classifier:
  - `noneAngle = 90`
  - `bigAngle = 140`
  - `smallAngle = 50`
- Trapdoor:
  - `lockAngle = 100`
  - `unlockAngle = 10`

### FSR grasp control
- Filter:
  - `alpha = 0.3`
- Contact detection thresholds:
  - `contactRawThres = 17`
  - `contactDeltaThres = 9`
- Back-off:
  - `backoffDeg = 12`
- Adaptive closing:
  - `maxRateDegPerLoop = 2.0`
- Loop timing:
  - `delay(20)` → ~50 Hz loop

### LDR ripeness detection
- Sampling:
  - `ldr_samples = 50`
  - `ldr_delay_ms = 5`
  - `ldr_settle_ms = 50`
- Hysteresis thresholds:
  - `highThreshold = 75` → UNRIPE
  - `lowThreshold = 65` → RIPE

### Diameter model
- `minDiameter = 15 mm`
- `maxDiameter = 62 mm`
- BIG/SMALL threshold:
  - BIG if `diameter > 20 mm`
  - else SMALL

---

## 8) What You’ll See in Serial Output

Printed continuously:
- `raw = <FSR raw>`
- state label:
  - `IDLE`, `GRASPING`, `GRABBING`, `GRABBED`, `RELEASE`

During DETECTION:
- `LDR net = <value>`
- `Ripeness: ...`

During GRABBED:
- `Raspberry Diameter in mm: <value>`
- `Raspberry is: BIG/SMALL`

---

## 9) Troubleshooting

### Servo resets / jitter on USB power
USB-only power can be marginal for 3 standard servos. If you see resets:
- avoid moving multiple servos simultaneously
- add bulk capacitance near servo power rails (e.g., 470–1000 µF)
- (recommended future improvement) use an external regulated 5V servo supply with common ground

### False “contact” triggers
If GRASPING triggers contact too early:
- increase `contactRawThres` and/or `contactDeltaThres`
- reduce noise / improve wiring
- keep the `ignoreContactOnce` guard after DETECTION → GRASPING

### Too many UNCERTAIN ripeness results
If `net` often falls between thresholds:
- widen hysteresis band:
  - raise `highThreshold`
  - lower `lowThreshold`
- improve illumination geometry and reduce ambient light influence

---

## 10) Cleanup / TODO (Optional polish)
- Remove legacy/unused items:
  - `releasePin`
  - `buttonPressed` variable (button is not used in final UI)
  - `normThres` (currently unused)
- Make RELEASE melody unconditional (remove `if(!taco)`)
- Split code into modules if needed (`fsm`, `sensing`, `actuators`)

---
## 11) Getting Started (Quick Run)

1. Open the Arduino IDE.
2. Make sure you have:
   - `Servo.h` available (built-in)
   - `pitches.h` available (Arduino IDE examples / install as needed)
3. Select:
   - **Board:** Arduino UNO
   - **Port:** your Arduino COM/tty port
4. Upload the sketch.
5. Open **Serial Monitor**:
   - **Baud:** 9600
   - **Line ending:** “No line ending” (recommended)
6. Place a raspberry in the sensing/gripper area.
7. Send command:
   - `A` to run **ripeness detection** (normal flow)
   - If **RIPE**, the system will auto-grasp and classify BIG/SMALL.
   - If **UNRIPE** or **UNCERTAIN**, it returns to IDLE.

Optional:
- Send `R` to open trapdoor (release behavior).
- Send `Z` to force the system back to IDLE.

---

## 12) Typical Usage Sequence (What the operator does)

### Normal run (intended)
1. System is in **IDLE** (LEDs blink).
2. Operator sends `A`.
3. System enters **DETECTION**, prints `LDR net = ...` and ripeness result.
4. If RIPE:
   - RIPE LED turns on
   - system enters **GRASPING** automatically
5. System detects gentle contact, backs off, then enters **GRABBED**
6. Prints estimated diameter and BIG/SMALL result
7. Classifier ramp tilts to the correct bin
8. System resets to **IDLE**

### Debug run (FSR/gripper validation only)
- Operator sends `G` while in IDLE to test GRASPING behavior.
- This exists only to validate force sensing / servo behavior during development.

---

## 13) Wiring Notes (Minimal but important)

- All grounds must be common: Arduino GND, servo grounds, sensor grounds.
- Servos can draw significant current even if powered by USB-only Arduino; if behavior is unstable, see Troubleshooting.
- Illumination LED uses a **220 Ω** resistor in series.
- FSR is the version that comes with its board + voltage divider (so you can wire it directly to A0 and GND/5V as designed).

---

## 14) Safety / Handling Notes

- This code assumes mechanical end-stops and servo angles are safe for your gripper geometry.
- Verify `openAngle` and `closedAngle` mechanically before running continuous cycles.
- Keep hands clear of the gripper during GRASPING/GRABBING.

---

## 15) Known Behavioral Quirks (From Current Code)

These are not “bugs” necessarily—just things to be aware of:

1) **Release melody may not always play**  
   In `RELEASE`, the melody loop is inside `if(!taco) { ... }`.  
   Depending on prior states, `taco` may be `true` when entering RELEASE, which skips the melody/LED show.

2) **Button is read but not used for control**  
   `readButtonPressed()` runs each loop and returns `buttonPressed`, but it is not used in state transitions.

3) **Diameter estimation uses Arduino `map()`**  
   `map()` is integer-based; the result is returned as `float` but is still quantized.  
   For higher precision you can replace it with a float linear interpolation.

---

## 17) Credits / Acknowledgements

- Arduino `Servo` library
- Arduino `tone()` / `pitches.h` note definitions (Arduino examples)

---

