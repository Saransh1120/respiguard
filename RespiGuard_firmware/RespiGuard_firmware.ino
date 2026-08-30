/*
  RespiGuard — hardware firmware.

  This is wokwi_sketch.txt with the simulation removed. The detection logic
  below — baseline learning, deviation scoring, the exertion veto, the state
  machine, the display and the button — is character-for-character the logic
  that was proven in simulation. It was ported by replacing sampleSensors()
  and nothing else, which was the plan from the start.

  What is now real:
    MAX30102   SpO2, heart rate and respiratory rate      (max30102.h)
    INMP441    wheeze, via log-mel and the trained model  (wheeze.h)
    BMI270     movement, for the exertion veto
    BME680     air quality
    SSD1306    display

  Two differences from the simulation, both deliberate:

  The baseline window is three days, not twenty seconds. A personal baseline
  built over twenty seconds is whatever the wearer happened to be doing during
  those twenty seconds. Three days covers sleep, rest and exertion.

  A vital that cannot be measured is dropped, not guessed. If the optical
  sensor loses chest contact, sampleSensors() leaves the previous value in
  place and marks the sample stale rather than feeding a number computed from
  ambient light into the baseline. A monitor that invents readings when it
  cannot see is worse than one that says nothing.

  Libraries: Adafruit SSD1306, Adafruit GFX, Adafruit BME680, SparkFun BMI270.

  The classifier deliberately needs no machine-learning library. Its weights are
  compiled in from wheeze_model.h and its forward pass is in classifier.h, which
  is less code than the interpreter would have been and leaves the arithmetic on
  the device open to inspection.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME680.h>
#include <SparkFun_BMI270_Arduino_Library.h>

#include "max30102.h"
#include "wheeze.h"
#include "classifier.h"

// ------------------------------------------------------------------ pins
#define PIN_SDA        8
#define PIN_SCL        9
#define PIN_LED_GREEN  15
#define PIN_LED_RED    16
#define PIN_MOTOR      17
#define PIN_BUTTON     18
#define PIN_BATTERY    1

#define OLED_W 128
#define OLED_H 64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
Adafruit_BME680  bme;
BMI270           imu;
MAX30102         ppg;
WheezeFrontEnd   mic;
WheezeClassifier classifier;

// -------------------------------------------------------------- tuning
// Every threshold here is a deviation from the wearer's own baseline, not an
// absolute number. Asthma presents differently in every person, so a fixed
// "SpO2 below 94" rule produces constant false alarms in some people and
// misses real attacks in others.
const uint32_t BASELINE_SECONDS   = 3UL * 24 * 3600;  // three days
const float    RR_RISE_FRAC       = 0.20;
const float    HR_RISE_FRAC       = 0.15;
const float    SPO2_DROP_POINTS   = 3.0;
const float    WHEEZE_THRESHOLD   = 0.50;
const float    ACTIVITY_VETO      = 0.35;

const int   SCORE_WATCH = 2;
const int   SCORE_ALERT = 4;
const uint32_t ALERT_HOLD_MS = 4000;

// -------------------------------------------------------------- state
enum State { STATE_LEARNING, STATE_OK, STATE_WATCH, STATE_ALERT };
State state = STATE_LEARNING;

const char *stateName();

struct Vitals {
  float spo2;
  float rr;
  float hr;
  float wheeze;
  float activity;
  float voc;
};

Vitals now;
bool   vitalsValid = false;   // false while the optical sensor has no contact

double baseSpo2 = 0, baseRr = 0, baseHr = 0;
uint32_t baseCount = 0;
bool baselineReady = false;

uint32_t lastSample = 0;
uint32_t stateEnteredAt = 0;
uint32_t lastBuzz = 0;
uint32_t eventsMarked = 0;
int lastScore = 0;
char lastReason[24] = "";

bool bmePresent = false, imuPresent = false;

// ------------------------------------------------------------- BMI270
/*
  Distance of the acceleration magnitude from 1 g, smoothed.

  Identical maths to the MPU6050 stand-in used in simulation; only the driver
  underneath changed. The BMI270 needs an 8 KB configuration blob uploaded at
  power-on before it will produce data at all, which is why this one sensor
  uses a library rather than raw I2C like the others.
*/
float imuActivity() {
  if (!imuPresent) return 0.0f;
  imu.getSensorData();

  float x = imu.data.accelX, y = imu.data.accelY, z = imu.data.accelZ;
  float mag = sqrtf(x * x + y * y + z * z);
  float dev = fabsf(mag - 1.0f);

  static float smooth = 0;
  smooth = smooth * 0.7f + dev * 0.3f;
  return constrain(smooth, 0.0f, 1.0f);
}

// ------------------------------------------------------- real sensors
/*
  The one function the port was supposed to touch.

  Each vital is taken from the part that actually measures it. Where a reading
  cannot be trusted the previous value stands and `vitalsValid` goes false, so
  the baseline never absorbs a number the hardware did not really see.
*/
void sampleSensors() {
  // Wheeze first: the classifier runs on four seconds of audio that the front
  // end has been accumulating in the background since the last call.
  if (mic.ready()) {
    static float patch[WZ_N_MELS * WZ_FRAMES];
    if (mic.patch(patch)) now.wheeze = classifier.predict(patch);
  } else {
    now.wheeze = 0.0f;
  }

  now.activity = imuActivity();

  if (bmePresent && bme.performReading()) {
    now.voc = bme.gas_resistance / 1000.0f;   // kOhm
  }

  // The optical sensor. Without contact the ratio-of-ratios is computed out of
  // ambient light and returns a confident, meaningless number.
  if (!ppg.present() || !ppg.contact() || !ppg.ready()) {
    vitalsValid = false;
    return;
  }

  float s = ppg.spo2();
  float h = ppg.heartRate();
  float r = ppg.respiratoryRate();

  // A single NaN means that one estimator failed this second — the finger
  // moved, the window straddled a motion artefact. Keeping the last good value
  // is right; treating the whole sample as valid when a vital is missing is not.
  bool ok = !isnan(s) && !isnan(h) && !isnan(r);
  if (!isnan(s)) now.spo2 = s;
  if (!isnan(h)) now.hr   = h;
  if (!isnan(r)) now.rr   = r;
  vitalsValid = ok;
}

// ----------------------------------------------------------- baseline
void updateBaseline() {
  // Only real readings shape the personal baseline.
  if (!vitalsValid) return;

  baseSpo2 += now.spo2;
  baseRr   += now.rr;
  baseHr   += now.hr;
  baseCount++;

  if (baseCount >= BASELINE_SECONDS) {
    baseSpo2 /= baseCount;
    baseRr   /= baseCount;
    baseHr   /= baseCount;
    baselineReady = true;
    state = STATE_OK;
    stateEnteredAt = millis();

    Serial.println();
    Serial.println("  baseline learned:");
    Serial.printf("    SpO2 %.1f %%   RR %.1f /min   HR %.0f bpm\n",
                  baseSpo2, baseRr, baseHr);
    Serial.println("  monitoring now.");
    Serial.println();
  }
}

void resetBaseline() {
  baseSpo2 = baseRr = baseHr = 0;
  baseCount = 0;
  baselineReady = false;
  state = STATE_LEARNING;
  stateEnteredAt = millis();
  lastReason[0] = 0;
  Serial.println("  baseline cleared, relearning");
}

// -------------------------------------------------------------- scoring
int scoreDeviation() {
  int score = 0;
  lastReason[0] = 0;

  bool rrHigh    = now.rr   > baseRr * (1.0f + RR_RISE_FRAC);
  bool hrHigh    = now.hr   > baseHr * (1.0f + HR_RISE_FRAC);
  bool spo2Low   = now.spo2 < baseSpo2 - SPO2_DROP_POINTS;
  bool wheezing  = now.wheeze > WHEEZE_THRESHOLD;

  // Wheeze and desaturation carry more weight because neither is explained by
  // ordinary activity. Rate changes alone are weak evidence.
  if (wheezing) { score += 2; strncpy(lastReason, "wheeze", sizeof(lastReason)); }
  if (spo2Low)  { score += 2; strncpy(lastReason, "SpO2 drop", sizeof(lastReason)); }
  if (rrHigh)   { score += 1; if (!lastReason[0]) strncpy(lastReason, "RR rise", sizeof(lastReason)); }
  if (hrHigh)   { score += 1; if (!lastReason[0]) strncpy(lastReason, "HR rise", sizeof(lastReason)); }

  // The exertion veto. Running up stairs raises RR and HR and drops SpO2 a
  // little; without this the device would alarm on every PE class.
  if (now.activity > ACTIVITY_VETO && !wheezing) {
    score -= 2;
    strncpy(lastReason, "moving", sizeof(lastReason));
  }

  return max(score, 0);
}

void updateState() {
  if (!baselineReady) return;

  // Scoring a stale sample would compare the baseline against numbers from
  // whenever contact was last good, which could be minutes ago.
  if (!vitalsValid) return;

  int score = scoreDeviation();
  lastScore = score;

  State target;
  if (score >= SCORE_ALERT)      target = STATE_ALERT;
  else if (score >= SCORE_WATCH) target = STATE_WATCH;
  else                           target = STATE_OK;

  bool calming = target < state;
  if (calming && millis() - stateEnteredAt < ALERT_HOLD_MS) return;

  if (target != state) {
    state = target;
    stateEnteredAt = millis();
    Serial.printf("  -> %s  (score %d, %s)\n", stateName(), score,
                  lastReason[0] ? lastReason : "within baseline");
  }
}

const char *stateName() {
  switch (state) {
    case STATE_LEARNING: return "LEARNING";
    case STATE_OK:       return "OK";
    case STATE_WATCH:    return "WATCH";
    default:             return "ALERT";
  }
}

// --------------------------------------------------------------- battery
/* Two 100 k resistors halve the cell voltage into the ADC. */
float batteryVolts() {
  return analogReadMilliVolts(PIN_BATTERY) * 2.0f / 1000.0f;
}

// ------------------------------------------------------------- outputs
void driveOutputs() {
  bool alert = (state == STATE_ALERT);
  bool watch = (state == STATE_WATCH);

  digitalWrite(PIN_LED_GREEN, state == STATE_OK);

  if (alert) {
    digitalWrite(PIN_LED_RED, (millis() / 250) % 2);
  } else if (watch) {
    digitalWrite(PIN_LED_RED, (millis() / 800) % 2);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
  }

  // Haptic: a double pulse every four seconds while in alert. The delays here
  // are short enough not to starve the audio front end of a hop.
  if (alert && millis() - lastBuzz > 4000) {
    lastBuzz = millis();
    for (int i = 0; i < 2; i++) {
      digitalWrite(PIN_MOTOR, HIGH); delay(120);
      digitalWrite(PIN_MOTOR, LOW);  delay(100);
    }
  }
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("RespiGuard");

  const char *s = stateName();
  int w = strlen(s) * 6;
  if (state == STATE_ALERT || state == STATE_WATCH) {
    display.fillRect(OLED_W - w - 4, 0, w + 4, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  }
  display.setCursor(OLED_W - w - 2, 1);
  display.print(s);
  display.setTextColor(SSD1306_WHITE);
  display.drawFastHLine(0, 11, OLED_W, SSD1306_WHITE);

  if (!vitalsValid && state != STATE_LEARNING) {
    // Saying so is the honest screen. A frozen set of numbers with no notice
    // reads as a working device.
    display.setCursor(0, 24);
    display.print("No chest contact");
    display.setCursor(0, 36);
    display.print("readings paused");
    display.display();
    return;
  }

  if (state == STATE_LEARNING) {
    display.setCursor(0, 16);
    display.print("Learning baseline");
    // Three days in seconds is not a number that reads well, so show hours.
    display.setCursor(0, 28);
    display.printf("%lu / %lu h", (unsigned long)(baseCount / 3600),
                   (unsigned long)(BASELINE_SECONDS / 3600));
    int barW = (int)((float)baseCount / BASELINE_SECONDS * (OLED_W - 4));
    display.drawRect(0, 42, OLED_W, 10, SSD1306_WHITE);
    display.fillRect(2, 44, constrain(barW, 0, OLED_W - 4), 6, SSD1306_WHITE);
    display.setCursor(0, 56);
    display.printf("WZ %.2f  %.2fV", now.wheeze, batteryVolts());
  } else {
    display.setCursor(0, 16);
    display.printf("SpO2 %4.1f%%  b%4.1f", now.spo2, baseSpo2);
    display.setCursor(0, 26);
    display.printf("RR   %4.1f    b%4.1f", now.rr, baseRr);
    display.setCursor(0, 36);
    display.printf("HR   %4.0f    b%4.0f", now.hr, baseHr);
    display.setCursor(0, 46);
    display.printf("WZ %.2f  ACT %.2f", now.wheeze, now.activity);

    display.setCursor(0, 56);
    if (lastReason[0]) display.printf("%s  score %d", lastReason, lastScore);
    else               display.printf("within baseline");
  }

  display.display();
}

// -------------------------------------------------------------- button
void handleButton() {
  static bool wasDown = false;
  static uint32_t downAt = 0;

  bool down = (digitalRead(PIN_BUTTON) == LOW);

  if (down && !wasDown) {
    downAt = millis();
  } else if (!down && wasDown) {
    uint32_t held = millis() - downAt;
    if (held > 1500) {
      resetBaseline();
    } else if (held > 30) {
      eventsMarked++;
      Serial.printf("  event %lu marked by wearer\n", (unsigned long)eventsMarked);
      digitalWrite(PIN_MOTOR, HIGH); delay(80); digitalWrite(PIN_MOTOR, LOW);
    }
  }
  wasDown = down;
}

// --------------------------------------------------------------- setup
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("  OLED not found at 0x3C");
  }
  display.clearDisplay();
  display.display();

  bool ppgOk = ppg.begin();
  imuPresent = (imu.beginI2C(0x68) == BMI2_OK) || (imu.beginI2C(0x69) == BMI2_OK);
  bmePresent = bme.begin(0x76) || bme.begin(0x77);
  if (bmePresent) {
    bme.setGasHeater(320, 150);   // 320 C for 150 ms
  }
  bool micOk = mic.begin();
  bool modelOk = classifier.begin();

  Serial.println();
  Serial.println("========================================");
  Serial.println("  RespiGuard - hardware");
  Serial.println("========================================");
  Serial.printf("  MAX30102  %s\n", ppgOk      ? "found" : "MISSING");
  Serial.printf("  BMI270    %s\n", imuPresent ? "found" : "MISSING");
  Serial.printf("  BME680    %s\n", bmePresent ? "found" : "MISSING");
  Serial.printf("  INMP441   %s\n", micOk      ? "started" : "FAILED");
  Serial.printf("  model     %s\n", modelOk    ? "loaded" : "FAILED");
  Serial.printf("  battery   %.2f V\n", batteryVolts());
  Serial.printf("  learning baseline for %lu h\n",
                (unsigned long)(BASELINE_SECONDS / 3600));
  Serial.println("  short press = mark event, hold 1.5 s = relearn");
  Serial.println();

  stateEnteredAt = millis();
}

void loop() {
  handleButton();

  // Both of these must run far more often than once a second. The optical FIFO
  // holds only 32 samples — a third of a second at 100 Hz — and the audio front
  // end needs a hop taken every 16 ms or the I2S DMA buffers overrun.
  ppg.poll();
  mic.poll();

  if (millis() - lastSample >= 1000) {
    lastSample = millis();

    sampleSensors();
    if (!baselineReady) updateBaseline();
    else                updateState();

    Serial.printf("  SpO2 %5.1f  RR %5.1f  HR %5.0f  WZ %.2f  ACT %.2f  [%s]%s\n",
                  now.spo2, now.rr, now.hr, now.wheeze, now.activity, stateName(),
                  vitalsValid ? "" : "  (no contact)");
  }

  driveOutputs();

  // The screen is the slowest thing on the I2C bus. Redrawing it every pass
  // starves the sensor polling above, so it gets its own cadence.
  static uint32_t lastDraw = 0;
  if (millis() - lastDraw >= 250) {
    lastDraw = millis();
    drawScreen();
  }
}
