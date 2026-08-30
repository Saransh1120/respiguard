/*
  RespiGuard - Hardware Self Test
  --------------------------------
  Uploads once and checks every component on the board, then prints a
  pass/fail table over Serial at 115200 baud.

  Requires: Arduino IDE, Boards Manager -> "esp32" by Espressif, version 3.0.0
  or newer. No external libraries are needed - every sensor is probed over raw
  I2C, so a chip that answers here is wired correctly even if you have not
  installed its driver library yet.

  Board settings that matter (Tools menu):
    Board            ESP32S3 Dev Module
    USB CDC On Boot  Enabled          <-- without this the Serial Monitor stays blank
    Flash Size       4MB (or whatever your board has)
    PSRAM            OPI PSRAM (only if your board actually has PSRAM)

  Type "r" in the Serial Monitor at any time to run the tests again.
*/

#include <Wire.h>
#include <ESP_I2S.h>

// ---------------------------------------------------------------- pin map
// Change these to match how you actually wired the board.
#define PIN_SDA        8
#define PIN_SCL        9

#define PIN_I2S_BCLK   4     // INMP441 SCK
#define PIN_I2S_WS     5     // INMP441 WS
#define PIN_I2S_DIN    6     // INMP441 SD

#define PIN_LED_GREEN  15
#define PIN_LED_RED    16
#define PIN_MOTOR      17    // through the 1k resistor into the 2N2222 base
#define PIN_BUTTON     18    // other side of the button goes to GND
#define PIN_VBAT       1     // junction of the two 100k resistors

// ------------------------------------------------------- expected I2C parts
#define ADDR_OLED      0x3C
#define ADDR_MAX30102  0x57
#define ADDR_BME680_A  0x76
#define ADDR_BME680_B  0x77
#define ADDR_BMI270_A  0x68
#define ADDR_BMI270_B  0x69

// Battery divider: two equal 100k resistors, so the pin sees half the pack.
#define VBAT_DIVIDER   2.0f
#define VBAT_MIN_MV    3000
#define VBAT_MAX_MV    4300

// ---------------------------------------------------------------- results
struct Result {
  const char *name;
  bool pass;
  bool skipped;
  char detail[48];
};

Result results[12];
int resultCount = 0;

void record(const char *name, bool pass, const char *fmt, ...) {
  if (resultCount >= (int)(sizeof(results) / sizeof(results[0]))) return;
  Result &r = results[resultCount++];
  r.name = name;
  r.pass = pass;
  r.skipped = false;
  va_list args;
  va_start(args, fmt);
  vsnprintf(r.detail, sizeof(r.detail), fmt, args);
  va_end(args);
}

void recordSkip(const char *name, const char *why) {
  if (resultCount >= (int)(sizeof(results) / sizeof(results[0]))) return;
  Result &r = results[resultCount++];
  r.name = name;
  r.pass = false;
  r.skipped = true;
  snprintf(r.detail, sizeof(r.detail), "%s", why);
}

// ------------------------------------------------------------ I2C helpers
bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Reads one register. Returns false if the chip does not answer.
bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  value = Wire.read();
  return true;
}

void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void ssd1306Cmd(uint8_t addr, uint8_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write(0x00);   // control byte: command
  Wire.write(cmd);
  Wire.endTransmission();
}

// ------------------------------------------------------------ test: I2C bus
int testI2CBus() {
  int found = 0;
  Serial.println("  scanning 0x08 .. 0x77 ...");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2cPing(addr)) {
      Serial.printf("    device at 0x%02X\n", addr);
      found++;
    }
  }
  record("I2C bus", found > 0, "%d device(s) responded", found);
  return found;
}

// ------------------------------------------------------------ test: OLED
void testOLED() {
  if (!i2cPing(ADDR_OLED)) {
    record("OLED SSD1306", false, "no answer at 0x3C");
    return;
  }

  static const uint8_t initSeq[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
    0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
  };
  for (uint8_t c : initSeq) ssd1306Cmd(ADDR_OLED, c);

  // 0xA5 lights every pixel regardless of what is in display RAM, so this is
  // a true panel test and needs no font data.
  Serial.println("  watch the screen - it should flash white three times");
  for (int i = 0; i < 3; i++) {
    ssd1306Cmd(ADDR_OLED, 0xA5);
    delay(220);
    ssd1306Cmd(ADDR_OLED, 0xA4);
    delay(220);
  }
  record("OLED SSD1306", true, "0x3C ok, panel flashed");
}

// ------------------------------------------------------------ test: BME680
void testBME680() {
  uint8_t addr = 0;
  if (i2cPing(ADDR_BME680_A))      addr = ADDR_BME680_A;
  else if (i2cPing(ADDR_BME680_B)) addr = ADDR_BME680_B;

  if (!addr) {
    record("BME680", false, "not at 0x76 or 0x77");
    return;
  }

  uint8_t id = 0;
  if (!i2cRead8(addr, 0xD0, id)) {
    record("BME680", false, "0x%02X found, no reply to reg read", addr);
    return;
  }
  // 0x61 is the BME680. A BMP280 breakout sold as a BME680 reads 0x58.
  record("BME680", id == 0x61, "addr 0x%02X, chip id 0x%02X", addr, id);
}

// ------------------------------------------------------------ test: MAX30102
void testMAX30102() {
  if (!i2cPing(ADDR_MAX30102)) {
    record("MAX30102", false, "no answer at 0x57");
    return;
  }
  uint8_t part = 0, rev = 0;
  i2cRead8(ADDR_MAX30102, 0xFF, part);
  i2cRead8(ADDR_MAX30102, 0xFE, rev);
  record("MAX30102", part == 0x15, "part id 0x%02X, rev 0x%02X", part, rev);
}

// ------------------------------------------------------------ test: BMI270
void testBMI270() {
  uint8_t addr = 0;
  if (i2cPing(ADDR_BMI270_A))      addr = ADDR_BMI270_A;
  else if (i2cPing(ADDR_BMI270_B)) addr = ADDR_BMI270_B;

  if (!addr) {
    record("BMI270", false, "not at 0x68 or 0x69");
    return;
  }

  uint8_t id = 0;
  i2cRead8(addr, 0x00, id);   // first read wakes the I2C interface
  delay(2);
  i2cRead8(addr, 0x00, id);
  record("BMI270", id == 0x24, "addr 0x%02X, chip id 0x%02X", addr, id);
}

// ------------------------------------------------------------ test: mic
void testMicrophone() {
  I2SClass i2s;
  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_WS, -1, PIN_I2S_DIN);

  if (!i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO)) {
    record("INMP441 mic", false, "I2S driver failed to start");
    return;
  }

  Serial.println("  say something or tap the mic now ...");

  const int frames = 512;
  int32_t buf[frames];
  double peak = 0;

  // Two seconds of listening, keeping the loudest block we see.
  for (int block = 0; block < 16; block++) {
    size_t got = i2s.readBytes((char *)buf, sizeof(buf));
    int n = got / sizeof(int32_t);
    if (n <= 0) continue;

    double sumSq = 0;
    for (int i = 0; i < n; i++) {
      double s = (double)(buf[i] >> 14);   // 32-bit frame down to a sane range
      sumSq += s * s;
    }
    double rms = sqrt(sumSq / n);
    if (rms > peak) peak = rms;
  }
  i2s.end();

  // A live INMP441 sitting in a quiet room still reads a few hundred counts.
  // A dead or miswired one sticks at zero.
  bool alive = peak > 50;
  record("INMP441 mic", alive, "peak level %.0f", peak);
}

// ------------------------------------------------------------ test: LEDs
void testLEDs() {
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  Serial.println("  green LED blinks twice, then red LED blinks twice");

  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH); delay(250);
    digitalWrite(PIN_LED_GREEN, LOW);  delay(250);
  }
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_LED_RED, HIGH); delay(250);
    digitalWrite(PIN_LED_RED, LOW);  delay(250);
  }
  // The pin cannot tell us whether light actually came out, so this one is
  // yours to judge.
  recordSkip("LEDs", "watch them - did both blink?");
}

// ------------------------------------------------------------ test: motor
void testMotor() {
  pinMode(PIN_MOTOR, OUTPUT);
  Serial.println("  motor should buzz twice");
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_MOTOR, HIGH); delay(300);
    digitalWrite(PIN_MOTOR, LOW);  delay(300);
  }
  recordSkip("Vibration motor", "did you feel two buzzes?");
}

// ------------------------------------------------------------ test: button
void testButton() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  if (digitalRead(PIN_BUTTON) == LOW) {
    record("Button", false, "reads pressed while idle - check wiring");
    return;
  }

  Serial.println("  press the function button within 5 seconds ...");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    if (digitalRead(PIN_BUTTON) == LOW) {
      record("Button", true, "press detected after %lu ms", millis() - start);
      delay(300);
      return;
    }
    delay(10);
  }
  record("Button", false, "no press seen in 5 s");
}

// ------------------------------------------------------------ test: battery
void testBattery() {
  int mv = analogReadMilliVolts(PIN_VBAT);
  int pack = (int)(mv * VBAT_DIVIDER);

  if (mv < 100) {
    record("Battery sense", false, "pin reads %d mV - divider missing?", mv);
    return;
  }
  bool sane = pack >= VBAT_MIN_MV && pack <= VBAT_MAX_MV;
  record("Battery sense", sane, "pack %d mV (pin %d mV)", pack, mv);
}

// ------------------------------------------------------------ report
void printReport() {
  int passed = 0, failed = 0, manual = 0;

  Serial.println();
  Serial.println("===================================================================");
  Serial.println("  RESULT");
  Serial.println("===================================================================");

  for (int i = 0; i < resultCount; i++) {
    const Result &r = results[i];
    const char *mark;
    if (r.skipped)   { mark = "[ ?? ]"; manual++; }
    else if (r.pass) { mark = "[ OK ]"; passed++; }
    else             { mark = "[FAIL]"; failed++; }
    Serial.printf("  %s  %-16s %s\n", mark, r.name, r.detail);
  }

  Serial.println("-------------------------------------------------------------------");
  Serial.printf("  %d passed, %d failed, %d for you to confirm by eye\n",
                passed, failed, manual);

  if (failed == 0) {
    Serial.println("  Every automatic check passed. Wire up the rest and move on.");
  } else {
    Serial.println();
    Serial.println("  Where to look first:");
    Serial.println("   - a whole row of I2C failures usually means SDA and SCL are");
    Serial.println("     swapped, or a module is not getting 3.3 V");
    Serial.println("   - one chip missing while the others answer means that");
    Serial.println("     module's own wiring, not the bus");
    Serial.println("   - devices that come and go point at the parallel pull-ups;");
    Serial.println("     lift them on two of the four breakouts");
  }
  Serial.println("===================================================================");
  Serial.println("  Send 'r' to run again.");
  Serial.println();
}

// ------------------------------------------------------------ run
void runAllTests() {
  resultCount = 0;

  Serial.println();
  Serial.println("===================================================================");
  Serial.println("  RespiGuard hardware self test");
  Serial.printf("  chip %s, %d MHz, flash %lu MB\n",
                ESP.getChipModel(), getCpuFrequencyMhz(),
                (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)));
  Serial.println("===================================================================");

  Serial.println("\n> I2C bus");        testI2CBus();
  Serial.println("\n> OLED");           testOLED();
  Serial.println("\n> BME680");         testBME680();
  Serial.println("\n> MAX30102");       testMAX30102();
  Serial.println("\n> BMI270");         testBMI270();
  Serial.println("\n> Microphone");     testMicrophone();
  Serial.println("\n> LEDs");           testLEDs();
  Serial.println("\n> Motor");          testMotor();
  Serial.println("\n> Button");         testButton();
  Serial.println("\n> Battery sense");  testBattery();

  printReport();
}

void setup() {
  Serial.begin(115200);

  // Wait for the Serial Monitor, but do not hang forever on battery power.
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);

  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_MOTOR, LOW);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);   // slow and forgiving; breadboards do not like 400 kHz

  delay(300);
  runAllTests();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') runAllTests();
  }
  delay(50);
}
