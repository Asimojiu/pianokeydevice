/****************************************************************
  ESP32-S3  |  A4988 stepper  |  HX711 load cell  |  SSD1306 OLED
  ----------------------------------------------------------------
  • Non-blocking stepper via hardware-timer ISR
  • HX711 every 25 ms → live weight
  • Reverses when Δ > 30 g OR W > 150 g, then rewinds STEPS_PER_REV
  • OLED: direction | big realtime weight | graph
           + down-weight, up-weight, after-touch distance
  • Enhanced Button Control:
    - Double click: Stop/Start cycle
    - Double click + long press: Stop + go downward
    - Triple click + long press: Stop + go upward
****************************************************************/

#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* --------------------------- Pins --------------------------- */
#define BUTTON_PIN 0
#define HX711_DATA_PIN 5
#define HX711_CLK_PIN 6
#define EN_PIN 7
#define RST_PIN 15
#define SLP_PIN 16
#define STEP_PIN 17
#define DIR_PIN 18
#define SDA_PIN 20
#define SCL_PIN 19
/* ------------------------------------------------------------ */

/* ----------- Stepper timing & mechanics --------------------- */
#define STEP_DELAY_US 800  // μs per step  → speed
#define STEPS_PER_REV 2500
/* ------------------------------------------------------------ */

/* --------------- Load-cell behaviour ------------------------ */
#define CALIB_FACTOR 6285.6
#define MEAS_INTERVAL_MS 5  // polling period
#define DELTA1_G 5.0f
#define DELTA2_G 30.0f
#define MAX_WEIGHT_G 150.0f

#define DOWN_DELAY_STEPS 150  // after contact
#define UP_DELAY_STEPS 150    // before release
#define AT_MIN_STEPS 250      // let-off must be ≥100 steps after contact

volatile bool autoTared = false;          // becomes true once we've tared this stroke
const unsigned long AUTO_TARE_STEP = 50;  // forward-step index at which to tare
/* ------------------------------------------------------------ */
//wind back contrl
bool doingInitialWindBack = false;
bool waitingAfterWindBack = false;
uint32_t windBackStopTime = 0;
/* ------------------- OLED parameters ------------------------ */
#define OLED_W 128
#define OLED_H 64
#define OLED_ADDR1 0x3C
#define OLED_ADDR2 0x3D
int8_t downGraphIdx = -1;  // -1 = not captured
int8_t upGraphIdx = -1;
/* ------------------------------------------------------------ */

/* ----------------------- State ------------------------------ */
enum MotionState {
  STOPPED,
  MOVING_FORWARD,
  MOVING_BACKWARD
};
volatile MotionState curState = MOVING_FORWARD;

volatile unsigned long forwardSteps = 0;  // counts only FWD steps
volatile long stepsRemaining = 0;         // BWD steps left
volatile bool stepPinHigh = false;

unsigned long firstDeltaStep = 0;  // contact index
bool delta10Captured = false;

float prevWeight = 0.0f;

/* ------------- Down / up / after-touch metrics -------------- */
float downWeight = 0.0f;
float upWeight = 0.0f;
float aftertouchDist = 0.0f;
bool downCaptured = false;
bool upCaptured = false;
bool atCaptured = false;

unsigned long letOffStep = 0;
bool letOffSeen = false;
bool prevDeltaPos = true;  // for sign change




/* ---------- Scrolling weight history for graph -------------- */
const uint8_t MAX_SAMPLES = 60;
int16_t weightHist[MAX_SAMPLES];   // 0.1 g units
uint32_t tsHist[MAX_SAMPLES];       // ms timestamp for each sample  ← NEW
uint8_t histIdx = 0;
bool histFilled = false;

/* ---- Tiny reverse ring-buffer (for 50-step look-back) ------ */
#define REV_BUF_SZ 600
float revBuf[REV_BUF_SZ];
uint16_t revIdx = 0;
uint16_t backStepsTaken = 0;

/* ------------------- Multi-Click Button Control ------------- */
#define CLICK_TIMEOUT_MS 800      // Time window for multi-clicks (longer than multi long press)
#define LONG_PRESS_MS 1500        // Long press threshold for single click
#define MULTI_LONG_PRESS_MS 600   // Shorter long press for multi-click sequences
#define SLEEP_LONG_PRESS_MS 3000  // Extra long press for sleep (single click)

struct ButtonState {
  bool prevLow = false;
  uint32_t pressStartTime = 0;
  uint32_t releaseTime = 0;
  uint8_t clickCount = 0;
  bool waitingForMoreClicks = false;
  bool longPressDetected = false;
  bool goingToSleep = false;
  bool manualControlActive = false;       // NEW: tracks if we're in manual control mode
  MotionState manualDirection = STOPPED;  // NEW: direction for manual control
};

ButtonState btnState;

/* --------------------- Objects ------------------------------ */
HX711 scale;
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
hw_timer_t* stepTimer = nullptr;

/* ==================== Timer Control ========================= */
void startStepperTimer() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerStart(stepTimer);
#else
  timerAlarmEnable(stepTimer);
#endif
}

void stopStepperTimer() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerStop(stepTimer);
#else
  timerAlarmDisable(stepTimer);
#endif
}

/* ==================== Interrupts ============================ */
void IRAM_ATTR stepISR() {
  if (curState == STOPPED) return;  // Don't step when stopped

  stepPinHigh = !stepPinHigh;
  digitalWrite(STEP_PIN, stepPinHigh);

  if (!stepPinHigh) {  // count falling edges
    if (curState == MOVING_FORWARD) {
      ++forwardSteps;
    } else {
      --stepsRemaining;
      ++backStepsTaken;
    }
  }

  /* Finished rewinding? */
  if (curState == MOVING_BACKWARD && stepsRemaining <= 0) {
    curState = MOVING_FORWARD;
    forwardSteps = 0;
    delta10Captured = false;
    downCaptured = upCaptured = atCaptured = false;
    letOffSeen = false;
    autoTared = false;
    downGraphIdx = -1;
    upGraphIdx = -1;
  }
}

/* ================= Helper functions ========================= */
void pushSample(float w) {
  weightHist[histIdx] = (int16_t)(w * 10.0f);     // 0.1 g
  tsHist[histIdx]     = millis();                  // ← NEW
  histIdx = (histIdx + 1) % MAX_SAMPLES;
  if (histIdx == 0) histFilled = true;
}

// Find ring index of the sample whose timestamp is closest to t (ms)
uint8_t histIndexByTime(uint32_t t) {
  const uint8_t samples = histFilled ? MAX_SAMPLES : histIdx;
  if (samples == 0) return 0;

  uint8_t bestRing = 0;
  uint32_t bestErr = 0xFFFFFFFF;

  // Oldest→newest order across the ring
  for (uint8_t i = 0; i < samples; ++i) {
    uint8_t ring = (histIdx + i) % MAX_SAMPLES;
    uint32_t err = (tsHist[ring] > t) ? (tsHist[ring] - t) : (t - tsHist[ring]);
    if (err < bestErr) { bestErr = err; bestRing = ring; }
  }
  return bestRing;
}

float steps2mm(unsigned long steps) {  // empirical factor
  return (steps * 10.0f) / 1005.0f;
}

void resetCycle() {
  forwardSteps = 0;
  delta10Captured = false;
  downCaptured = upCaptured = atCaptured = false;
  letOffSeen = false;
  autoTared = false;
  downGraphIdx = -1;
  upGraphIdx = -1;
  downWeight = 0.0f;
  upWeight = 0.0f;
  aftertouchDist = 0.0f;
}

// ===== Inertia Measurement Config =====
const float TOUCH_LEVEL_G      = 2.0f;
const float SLOPE_THRESH_G     = 2.0f;
const uint8_t SLOPE_WIN        = 3;
const uint8_t STABLE_WIN       = 8;
const float STABLE_DFDT_G      = 0.1f;
const float STABLE_VAR_G2      = 0.20f;

bool inertiaActive = false;       // currently tracking inertia segment
bool inertiaFoundThisCycle = false; // true if we detected in this forward stroke

uint32_t tTouchUs = 0, tStableUs = 0;
float FsteadyG = 0.0f;
double impulseNs = 0.0;
float lastG = 0.0f;
uint32_t lastTsUs = 0;

struct Samp { float g; uint32_t us; };
const uint8_t WIN = 20;
Samp win[WIN]; uint8_t widx = 0; uint8_t wfilled = 0;

inline void pushWin(float g, uint32_t us){
  win[widx] = {g, us};
  widx = (widx + 1) % WIN;
  if (wfilled < WIN) wfilled++;
}

bool computeSlopeVar(float& slope_g_per_samp, float& var_g2, float& mean_g) {
  const uint8_t n = min(wfilled, (uint8_t)max((int)STABLE_WIN, (int)SLOPE_WIN));
  if (n < 2) return false;
  float sum=0, sum2=0;
  for (uint8_t i=0;i<n;i++){
    const Samp& s = win[(widx + WIN - 1 - i) % WIN];
    sum += s.g; sum2 += s.g * s.g;
  }
  mean_g = sum / n;
  var_g2 = (sum2 / n) - (mean_g * mean_g);
  const Samp& a = win[(widx + WIN - 1) % WIN];
  const Samp& b = win[(widx + WIN - 1 - min(SLOPE_WIN, (uint8_t)(n-1))) % WIN];
  slope_g_per_samp = (a.g - b.g) / (float)min(SLOPE_WIN, (uint8_t)(n-1));
  return true;
}

void onForceSample(float g){
  // Only run during forward movement
  if (curState != MOVING_FORWARD) return;

  const uint32_t nowUs = micros();
  pushWin(g, nowUs);

  float slope, var, mean;
  if (!computeSlopeVar(slope, var, mean)) return;

  static bool armed = true;

  // Detect touch
  if (armed && slope > SLOPE_THRESH_G && g > TOUCH_LEVEL_G) {
    inertiaActive = true;
    tTouchUs = nowUs;
    impulseNs = 0.0;
    FsteadyG = 0.0f;
    armed = false;
  }

  // Accumulate impulse
  if (inertiaActive) {
    if (lastTsUs != 0) {
      const float dt_s = (nowUs - lastTsUs) * 1e-6f;
      const float g_above = max(0.0f, ((lastG + g) * 0.5f) - FsteadyG);
      impulseNs += (double)g_above * 0.00980665 * dt_s;
    }
  }

  // Detect stability
  const bool stable = (fabs(slope) < STABLE_DFDT_G) && (var < STABLE_VAR_G2);
  if (inertiaActive && stable) {
    tStableUs = nowUs;
    FsteadyG = mean;
    inertiaActive = false;
    inertiaFoundThisCycle = true;  // mark success

    const float duration_ms = (tStableUs - tTouchUs) * 1e-3f;
    Serial.printf("INERTIA: dur=%.1f ms, impulse=%.4f N·s, Fsteady=%.1f g\n",
                  duration_ms, impulseNs, FsteadyG);
  }

  // Re-arm
  if (!armed && !inertiaActive && g < (TOUCH_LEVEL_G * 0.5f)) {
    armed = true;
  }

  lastG = g;
  lastTsUs = nowUs;
}

// Call at the end of each forward cycle
void checkInertiaEndOfCycle() {
  if (curState != MOVING_FORWARD && !inertiaFoundThisCycle) {
    Serial.println("INERTIA: nothing found in forward cycle");
  }
  if (curState != MOVING_FORWARD) {
    inertiaFoundThisCycle = false; // reset for next cycle
  }
}

/* ---------------- Behaviour & metric logic ------------------ */
void updateBehaviour(float w) {
  if (curState == STOPPED || btnState.manualControlActive) return;  // Don't update behavior when stopped or in manual control

  const float delta = w - prevWeight;
  bool deltaPos = (delta > 0.0f);

  /* ---------- FIRST CONTACT (Δ>10 g) ------------------ */
  if (!delta10Captured && delta > DELTA1_G) {
    Serial.println("touching");
    firstDeltaStep = forwardSteps;
    delta10Captured = true;
  }

  /* ---------- DOWN-WEIGHT (50 steps after contact) ----- */
  if (delta10Captured && !downCaptured && forwardSteps >= firstDeltaStep + DOWN_DELAY_STEPS) {
    downWeight = w;
    downCaptured = true;
    // Use current time to find the nearest plotted sample
    downGraphIdx = histIndexByTime(millis());   // ← replace old assignment
  }

  /* ---------- LET-OFF detection (after-touch) ---------- */
  if (delta10Captured && !letOffSeen && forwardSteps >= firstDeltaStep + AT_MIN_STEPS && prevDeltaPos && !deltaPos) {  // first local peak
    letOffStep = forwardSteps;
    letOffSeen = true;
    Serial.println("Let-off detected");
  }

  /* ---------- TRIGGER REVERSAL ------------------------- */
  if (curState == MOVING_FORWARD && (delta > DELTA2_G || w > MAX_WEIGHT_G)) {
    checkInertiaEndOfCycle();   // <<< NEW
    curState = MOVING_BACKWARD;
    stepsRemaining = STEPS_PER_REV;
    backStepsTaken = 0;
  }

  /* ---------- Capture weights while rewinding ---------- */
  if (curState == MOVING_BACKWARD) {
    // Store current weight in reverse buffer
    revBuf[revIdx] = w;
    revIdx = (revIdx + 1) % REV_BUF_SZ;

    /* FIXED: Capture upWeight 150 steps BEFORE actual release */
    if (!upCaptured && backStepsTaken >= UP_DELAY_STEPS && w < 2.0f) {
      // (keep your upWeight from revBuf if you like)
      uint16_t currentIdx = (revIdx + REV_BUF_SZ - 1) % REV_BUF_SZ;
      uint16_t captureIdx = (currentIdx + REV_BUF_SZ - UP_DELAY_STEPS) % REV_BUF_SZ;
      upWeight  = revBuf[captureIdx];
      upCaptured = true;

      // Convert "150 steps ago" into a timestamp and map to the nearest history sample.
      // NOTE: step ISR toggles STEP pin every interrupt, and you count on falling edges.
      // Your STEP timing is STEP_DELAY_US per edge, so 1 step ~ STEP_DELAY_US μs (as coded).
      const uint32_t stepDelayMs = STEP_DELAY_US / 1000;          // μs → ms
      const uint32_t captureTimeMs = millis() - (UP_DELAY_STEPS * stepDelayMs);
      upGraphIdx = histIndexByTime(captureTimeMs);                 // ← NEW

      Serial.printf("Up-weight captured: %.1fg (mapped to graph t-%.u ms)\n",
                    upWeight, (unsigned)(UP_DELAY_STEPS * stepDelayMs));
    }
  }

  /* ---------- AFTER-TOUCH distance --------------------- */
  if (curState == MOVING_BACKWARD && letOffSeen && !atCaptured) {
    aftertouchDist = steps2mm(forwardSteps - letOffStep);
    atCaptured = true;  // freezes the value
  }

  prevWeight = w;
  prevDeltaPos = deltaPos;
}

/* ==================== Button Handling ==================== */
void processMultiClick(uint8_t clicks, bool finalLongPress) {
  Serial.printf("Processing: %d clicks, final long press: %s\n", clicks, finalLongPress ? "YES" : "NO");

  switch (clicks) {
    case 1:
      if (finalLongPress) {
        // Single long press = deep sleep (original behavior)
        btnState.goingToSleep = true;
        Serial.println("Single long press - preparing for deep sleep");
      } else {
        // Single short press = tare (original behavior)
        Serial.println("Single press → TARE");
        scale.tare(10);
        prevWeight = 0.0f;
        resetCycle();
      }
      break;

    case 2:
      // Double click
      if (finalLongPress) {
        // Double click + long press = stop + go downward
        Serial.println("Double + long press: STOP + GO DOWN");
        curState = STOPPED;
        stopStepperTimer();
        resetCycle();
        // Force downward movement
        digitalWrite(DIR_PIN, LOW);  // Set direction to down
        curState = MOVING_BACKWARD;
        stepsRemaining = STEPS_PER_REV;
        backStepsTaken = 0;
        startStepperTimer();
      } else {
        // Double click = stop/start toggle
        if (curState == STOPPED) {
          Serial.println("Double click: START cycle");
          curState = MOVING_FORWARD;
          resetCycle();
          startStepperTimer();
        } else {
          Serial.println("Double click: STOP cycle");
          curState = STOPPED;
          stopStepperTimer();
          resetCycle();
        }
      }
      break;

    case 3:
      if (finalLongPress) {
        // Triple click + long press = stop + go upward
        Serial.println("Triple + long press: STOP + GO UP");
        curState = STOPPED;
        stopStepperTimer();
        resetCycle();
        // Force upward movement
        digitalWrite(DIR_PIN, HIGH);  // Set direction to up
        curState = MOVING_FORWARD;
        forwardSteps = 0;
        startStepperTimer();
      } else {
        // Triple click without long press = just stop
        Serial.println("Triple click: STOP cycle");
        curState = STOPPED;
        stopStepperTimer();
        resetCycle();
      }
      break;

    default:
      Serial.printf("Unhandled click pattern: %d clicks\n", clicks);
      break;
  }
}

void handleButton() {
  const uint32_t now = millis();
  bool btnLow = (digitalRead(BUTTON_PIN) == LOW);

  // Button pressed
  if (btnLow && !btnState.prevLow) {
    btnState.pressStartTime = now;
    btnState.longPressDetected = false;

    if (!btnState.waitingForMoreClicks) {
      btnState.clickCount = 0;
    }
    Serial.printf("Button pressed (current click count: %d, waiting: %s)\n",
                  btnState.clickCount, btnState.waitingForMoreClicks ? "YES" : "NO");
  }

  // Check for long press while button is held
  if (btnLow && !btnState.longPressDetected) {
    uint32_t pressDuration = now - btnState.pressStartTime;

    // Use different thresholds for single vs multi-click
    uint32_t longPressThreshold = (btnState.clickCount == 0 && !btnState.waitingForMoreClicks)
                                    ? LONG_PRESS_MS
                                    : MULTI_LONG_PRESS_MS;

    // Debug: Show duration every 100ms while held
    static uint32_t lastDurationPrint = 0;
    if (now - lastDurationPrint >= 100) {
      Serial.printf("Holding button: %lu ms (target: %lu ms, mode: %s)\n",
                    pressDuration, longPressThreshold,
                    (btnState.clickCount == 0 && !btnState.waitingForMoreClicks) ? "SINGLE" : "MULTI");
      lastDurationPrint = now;
    }

    if (pressDuration >= longPressThreshold) {
      btnState.longPressDetected = true;

      // Determine if this should be sleep or multi-click+long press
      if (btnState.clickCount == 0 && !btnState.waitingForMoreClicks) {
        // True single press - check if long enough for sleep
        if (pressDuration >= SLEEP_LONG_PRESS_MS) {
          btnState.goingToSleep = true;
          Serial.println("Sleep long press detected (single press)");
        } else {
          Serial.println("Regular long press detected (single press)");
        }
      } else {
        // Multi-click sequence + long press - START MOTOR IMMEDIATELY
        Serial.printf("Long press detected in multi-click sequence (clicks so far: %d)\n", btnState.clickCount);

        // Stop current cycle and start manual control
        curState = STOPPED;
        stopStepperTimer();
        resetCycle();
        btnState.manualControlActive = true;

        if (btnState.clickCount == 1) {
          // Double click + long press = go down
          Serial.println("MANUAL CONTROL: Moving DOWN (while button held)");
          btnState.manualDirection = MOVING_FORWARD;
          digitalWrite(DIR_PIN, LOW);  // Set direction to down
          curState = MOVING_FORWARD;
          stepsRemaining = 999999;  // Large number for continuous movement
          backStepsTaken = 0;
        } else if (btnState.clickCount == 2) {
          // Triple click + long press = go up
          Serial.println("MANUAL CONTROL: Moving UP (while button held)");
          btnState.manualDirection = MOVING_BACKWARD;
          digitalWrite(DIR_PIN, HIGH);  // Set direction to up
          curState = MOVING_BACKWARD;
          forwardSteps = 0;
        }

        startStepperTimer();
      }
    }
  }

  // Button released
  if (!btnLow && btnState.prevLow) {
    uint32_t totalHoldTime = now - btnState.pressStartTime;
    btnState.releaseTime = now;
    btnState.clickCount++;

    Serial.printf("Button released - Click %d detected (held for: %lu ms, long press: %s)\n",
                  btnState.clickCount, totalHoldTime, btnState.longPressDetected ? "YES" : "NO");

    // If manual control was active, stop it
    if (btnState.manualControlActive) {
      Serial.println("MANUAL CONTROL: Stopping motor (button released)");
      curState = STOPPED;
      stopStepperTimer();
      btnState.manualControlActive = false;
      btnState.manualDirection = STOPPED;
      btnState.clickCount = 0;
      btnState.waitingForMoreClicks = false;
      return;  // Don't process further
    }

    // Handle other long press cases
    if (btnState.longPressDetected) {
      if (btnState.goingToSleep) {
        Serial.println("Processing sleep command");
        // Don't reset states - let main loop handle sleep
      } else {
        // Single press long press = tare
        Serial.println("Single long press: TARE");
        scale.tare(10);
        prevWeight = 0.0f;
        resetCycle();
        btnState.clickCount = 0;
        btnState.waitingForMoreClicks = false;
      }
    } else {
      // Short press - start/continue multi-click sequence
      btnState.waitingForMoreClicks = true;
    }
  }

  // Check for timeout on multi-click sequence (only for short presses and NOT while button is held)
  if (btnState.waitingForMoreClicks && !btnState.longPressDetected && !btnState.goingToSleep && !btnLow && (now - btnState.releaseTime >= CLICK_TIMEOUT_MS)) {
    Serial.printf("Multi-click timeout - processing %d clicks\n", btnState.clickCount);
    processMultiClick(btnState.clickCount, false);
    btnState.clickCount = 0;
    btnState.waitingForMoreClicks = false;
  }

  btnState.prevLow = btnLow;
}

/* ---------------------- OLED screen ------------------------- */
void drawScreen(float w) {
  display.clearDisplay();
  const int16_t GRAPH_CLIP_G_TENTHS = 2000;  // ignore values above 60.0 g

  /* Big realtime weight */
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.printf("%5.1f g", w);

  /* Direction & distance (small) */
  display.setTextSize(1);
  display.setCursor(0, 20);
  if (btnState.manualControlActive) {
    display.print("MANUAL: ");
    display.print(btnState.manualDirection == MOVING_FORWARD ? "DOWN" : "UP");
  } else if (curState == STOPPED) {
    display.print("DIR: STOPPED");
  } else if (curState == MOVING_FORWARD) {
    display.print("DIR: FWD ");
  } else {
    display.print("DIR: REV ");
    display.printf("Dist %.1fmm", steps2mm(forwardSteps - firstDeltaStep));
  }

  /* Down / Up / AT metrics (update live, freeze once captured) */
  display.setCursor(0, 32);
  display.printf("Dn:%4.1fg Up:%4.1fg", downWeight, upWeight);
  display.setCursor(0, 42);
  display.printf("AT:%4.1fmm", aftertouchDist);

  /* ----------- scrolling graph (bottom) ---------------- */
  const uint8_t samples = histFilled ? MAX_SAMPLES : histIdx;
  if (samples > 1) {
    int16_t minV = weightHist[0], maxV = weightHist[0];
    for (uint8_t i = 1; i < samples; ++i) {
      int16_t v = weightHist[i];
      if (abs(v) > GRAPH_CLIP_G_TENTHS) continue;  // skip spikes
      if (v < minV) minV = v;
      if (v > maxV) maxV = v;
    }
    if (minV == maxV) maxV = minV + 1;

    const uint8_t y0 = 50, y1 = 63;
    uint8_t lastX = 0, lastY = 0;
    for (uint8_t i = 0; i < samples; ++i) {
      uint8_t ring = (histIdx + i) % MAX_SAMPLES;
      int16_t v = weightHist[ring];

      uint8_t x = (uint16_t)i * (OLED_W - 1) / (samples - 1);
      uint8_t y = map(v, minV, maxV, y1, y0);
      if (i) display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
      lastX = x;
      lastY = y;

      // ----- Draw markers -----
      if (ring == downGraphIdx) {
        display.drawLine(x, y0, x, y1, SSD1306_WHITE);  // vertical bar for down
      }
      if (ring == upGraphIdx) {
        display.drawLine(x, y0, x, y1, SSD1306_WHITE);  // vertical bar for up
      }
    }
  }

  display.display();
}

/* ==================== Deep Sleep ========================== */
#include "esp_sleep.h"

void enterDeepSleep() {
  Serial.println("Going to deep-sleep…");

  /* 1. Park the motor driver */
  digitalWrite(SLP_PIN, LOW);  // A4988 sleep
  digitalWrite(EN_PIN, HIGH);  // disable outputs

  /* 2. Power-down peripherals */
  scale.power_down();  // HX711 ~1 µA
  display.ssd1306_command(SSD1306_DISPLAYOFF);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  timerEnd(stepTimer);
#else
  timerAlarmDisable(stepTimer);
  timerDetachInterrupt(stepTimer);
  timerEnd(stepTimer);
#endif

  /* 3. Configure wake source */
  esp_sleep_enable_ext1_wakeup(1ULL << BUTTON_PIN, ESP_EXT1_WAKEUP_ALL_LOW);

  Serial.flush();
  esp_deep_sleep_start();  // never returns
}

/* ======================= SETUP ============================== */
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(EN_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(SLP_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(SLP_PIN, HIGH);
  digitalWrite(DIR_PIN, LOW);

  /* OLED */
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR1) && !display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR2))
    Serial.println("OLED not found!");

  /* HX711 */
  scale.begin(HX711_DATA_PIN, HX711_CLK_PIN);
  scale.set_scale(CALIB_FACTOR);
  scale.tare(10);
  prevWeight = scale.get_units(5);

  /* Timer setup */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  const uint32_t baseFreq = 1'000'000;
  stepTimer = timerBegin(baseFreq);
  timerAttachInterrupt(stepTimer, &stepISR);
  timerAlarm(stepTimer, STEP_DELAY_US, true, 0);
#else
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &stepISR, true);
  timerAlarmWrite(stepTimer, STEP_DELAY_US, true);
  timerAlarmEnable(stepTimer);

#endif
  //Initial wind-back request
  doingInitialWindBack = true;  // <- global flag
  curState = MOVING_BACKWARD;
  stepsRemaining = 1005;  // <- initial windup steps
  backStepsTaken = 0;
  digitalWrite(DIR_PIN, HIGH);
  startStepperTimer();

  Serial.println("System ready. Button controls:");
  Serial.println("- Single press: Tare");
  Serial.println("- Single long press (3s): Deep sleep");
  Serial.println("- Double press: Stop/Start cycle");
  Serial.println("- Double + HOLD (0.6s): Manual DOWN (while held)");
  Serial.println("- Triple + HOLD (0.6s): Manual UP (while held)");
}



/* ======================== LOOP ============================== */
void loop() {
  // Ignore button for the first 2000 ms after boot
  static const uint32_t BUTTON_IGNORE_TIME_MS = 2000;
  if (millis() < BUTTON_IGNORE_TIME_MS) return;

  const uint32_t now = millis();
  static uint32_t lastPoll = 0;

  // --- Initial wind-back handling ---
  if (doingInitialWindBack) {
    if (scale.is_ready()) {
      float w = scale.get_units(1);
      if (w < -20.0f || stepsRemaining <= 0) {
        Serial.printf("Initial wind-back stop: weight=%.1fg, stepsRemaining=%ld\n", w, stepsRemaining);
        stopStepperTimer();
        curState = STOPPED;
        doingInitialWindBack = false;

        // Start waiting period
        waitingAfterWindBack = true;
        windBackStopTime = now;
      }
    }
    return;  // skip normal cycle during wind-back
  }

  // --- Waiting period after wind-back ---
  if (waitingAfterWindBack) {
    if (now - windBackStopTime >= 1500) {  // 1.5 seconds
      Serial.println("Starting forward cycle after wind-back");
      resetCycle();
      curState = MOVING_FORWARD;
      startStepperTimer();
      waitingAfterWindBack = false;
    }
    return;  // skip normal logic during wait
  }

  /* Handle button input */
  handleButton();

  /* Check if going to sleep */
  if (btnState.goingToSleep) {
    // Wait for button release before sleeping
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(1);
    }
    enterDeepSleep();
  }

  /* -------- HX711 polling -------- */
  if (now - lastPoll >= MEAS_INTERVAL_MS && scale.is_ready()) {
    lastPoll = now;
    float w = scale.get_units(1);
    pushSample(w);
    onForceSample(w);       // <<< NEW inertia tracking
    updateBehaviour(w);
    drawScreen(w);
  }

  /* ---------- Auto-tare 30 steps into every forward stroke -------- */
  if (curState == MOVING_FORWARD && !autoTared && forwardSteps >= AUTO_TARE_STEP) {
    scale.tare(10);     // zero the load-cell (10 samples)
    prevWeight = 0.0f;  // avoid a huge Δ on the next reading
    autoTared = true;   // remember we've done it
    Serial.println("Auto-tare completed @ step 30");
  }

  /* Align DIR pin continuously */
  if (curState != STOPPED) {
    digitalWrite(DIR_PIN, curState == MOVING_FORWARD ? LOW : HIGH);
  }
}