/*
  MAX30102 — SpO2, heart rate and respiratory rate from one finger of light.

  The part gives raw red and infrared counts and nothing else; every number
  RespiGuard uses out of it is computed here. Three quantities come from the
  same 100 Hz infrared stream, which is why they live in one file:

    SpO2  ratio-of-ratios on the two wavelengths
    HR    peak detection on the pulsatile part of the infrared signal
    RR    the slow envelope that breathing impresses on that same signal

  The third one is the one people are surprised by. Breathing moves venous
  return and thoracic pressure, so each breath modulates the amplitude of the
  pulse waveform at 0.1-0.5 Hz. Pulling the respiratory rate out of the PPG
  costs nothing extra in parts and is well established in the literature. It is
  also the reason RespiGuard needs no separate breathing sensor.

  Raw I2C throughout. The register map is small and the FIFO read is the only
  part that matters, so a library would add a dependency for very little.
*/

#pragma once
#include <Wire.h>

#define MAX30102_ADDR 0x57

// Register map, only the parts used here.
#define REG_INTR_STATUS_1 0x00
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C  // red
#define REG_LED2_PA       0x0D  // infrared
#define REG_PART_ID       0xFF

// Sampling geometry. 100 Hz over an eight-second window is long enough to hold
// several breaths, which is what the respiratory estimate needs; the heart rate
// would be happy with a quarter of that.
static const int   PPG_HZ      = 100;
static const int   PPG_WINDOW  = PPG_HZ * 8;   // 800 samples
static const float PPG_DT      = 1.0f / PPG_HZ;

// Mean infrared count above which the sensor is considered to be against
// skin. Provisional - see contact() below.
static const double CONTACT_IR_THRESHOLD = 50000.0;

class MAX30102 {
 public:
  bool begin() {
    if (readReg(REG_PART_ID) != 0x15) return false;

    writeReg(REG_MODE_CONFIG, 0x40);   // reset
    delay(100);
    // No sample averaging, rollover enabled, almost-full at 15.
    //
    // This register is worth reading twice. Bits 7:5 are SMP_AVE: 0x4F sets
    // them to 010, which averages four samples and drops the FIFO's real
    // output rate to 25 Hz while PPG_HZ below still says 100. Every derived
    // number - heart rate, respiratory rate, the window length - would then be
    // wrong by a factor of four. Bit 4 is FIFO_ROLLOVER_EN, and in 0x4F it is
    // 0, so a late poll() would freeze the buffer rather than overwrite it.
    // 0x1F sets SMP_AVE to 1 and rollover on, which is what the rest of this
    // file has always assumed.
    writeReg(REG_FIFO_CONFIG, 0x1F);
    writeReg(REG_MODE_CONFIG, 0x03);   // SpO2 mode: red + IR
    // 4096 nA full scale, 411 us pulse width, 100 Hz. The long pulse width buys
    // 18-bit resolution, which the ratio-of-ratios needs to be stable through
    // clothing and chest contact rather than a clean fingertip.
    writeReg(REG_SPO2_CONFIG, 0x27);
    writeReg(REG_LED1_PA, 0x24);       // ~7 mA
    writeReg(REG_LED2_PA, 0x24);
    clearFifo();
    present_ = true;
    return true;
  }

  bool present() const { return present_; }

  /*
    Drain whatever the FIFO holds into the ring buffers.

    Called every loop rather than on a timer: the part fills at its own rate and
    the only failure that matters is letting it overflow, which silently drops
    samples and puts a step in the middle of a waveform being measured.
  */
  void poll() {
    if (!present_) return;

    uint8_t wr = readReg(REG_FIFO_WR_PTR);
    uint8_t rd = readReg(REG_FIFO_RD_PTR);
    int available = (wr - rd) & 0x1F;
    if (available == 0) return;

    for (int i = 0; i < available; i++) {
      Wire.beginTransmission(MAX30102_ADDR);
      Wire.write(REG_FIFO_DATA);
      if (Wire.endTransmission(false) != 0) return;
      if (Wire.requestFrom(MAX30102_ADDR, 6) != 6) return;

      uint32_t red = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      uint32_t ir  = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      red &= 0x03FFFF;  // 18 bits
      ir  &= 0x03FFFF;

      redBuf_[head_] = red;
      irBuf_[head_]  = ir;
      head_ = (head_ + 1) % PPG_WINDOW;
      if (filled_ < PPG_WINDOW) filled_++;
    }
  }

  bool ready() const { return filled_ >= PPG_WINDOW; }

  /*
    Is a finger — or a chest — actually against the sensor?

    An unloaded MAX30102 still returns numbers, just small ones. Without this
    check the device would compute a confident SpO2 out of ambient light and
    feed it straight into the baseline.
  */
  bool contact() const {
    // The threshold below has never been checked against a real reading. It
    // must be measured on the bench, through clothing, against a chest: too
    // high and the device decides nobody is wearing it, too low and it
    // computes a confident SpO2 out of ambient light.
    if (filled_ < PPG_WINDOW) return false;
    double sum = 0;
    for (int i = 0; i < PPG_WINDOW; i++) sum += irBuf_[i];
    return (sum / PPG_WINDOW) > CONTACT_IR_THRESHOLD;
  }

  /*
    SpO2 by ratio-of-ratios.

    R = (AC_red / DC_red) / (AC_ir / DC_ir), and SpO2 is an empirical curve on
    R. The coefficients below are Maxim's published approximation for this
    part. They are a factory curve, not a calibration against a real oximeter,
    so the absolute value carries real error — which is exactly why RespiGuard
    scores a *drop from the wearer's own baseline* rather than an absolute
    threshold. A consistent offset cancels out of a difference.
  */
  float spo2() {
    if (filled_ < PPG_WINDOW) return NAN;

    double dcRed = 0, dcIr = 0;
    for (int i = 0; i < PPG_WINDOW; i++) { dcRed += redBuf_[i]; dcIr += irBuf_[i]; }
    dcRed /= PPG_WINDOW; dcIr /= PPG_WINDOW;
    if (dcIr < 1000 || dcRed < 1000) return NAN;

    // RMS about the mean is a steadier AC estimate than peak-to-peak, which one
    // motion artefact can double.
    double acRed = 0, acIr = 0;
    for (int i = 0; i < PPG_WINDOW; i++) {
      double dr = redBuf_[i] - dcRed, di = irBuf_[i] - dcIr;
      acRed += dr * dr; acIr += di * di;
    }
    acRed = sqrt(acRed / PPG_WINDOW);
    acIr  = sqrt(acIr  / PPG_WINDOW);
    if (acIr < 1.0) return NAN;

    double R = (acRed / dcRed) / (acIr / dcIr);
    double s = 104.0 - 17.0 * R;
    if (s > 100.0) s = 100.0;
    if (s < 70.0)  return NAN;   // below this the curve is not trustworthy
    return (float)s;
  }

  /* Heart rate from peak spacing on the infrared pulse waveform. */
  float heartRate() {
    if (filled_ < PPG_WINDOW) return NAN;

    static float sig[PPG_WINDOW];
    detrend(irBuf_, sig);
    smooth(sig, 5);

    // A peak must clear half the signal's own standard deviation and cannot
    // arrive within 300 ms of the last one — 200 bpm is past anything this
    // wearer will produce, so a closer pair is the dicrotic notch, not a beat.
    float sd = stddev(sig);
    if (sd < 1.0f) return NAN;
    float thresh = sd * 0.5f;
    int   refractory = (int)(0.3f * PPG_HZ);

    int lastPeak = -refractory;
    int beats = 0;
    long spacingSum = 0;
    for (int i = 1; i < PPG_WINDOW - 1; i++) {
      if (sig[i] > thresh && sig[i] > sig[i - 1] && sig[i] >= sig[i + 1]
          && (i - lastPeak) >= refractory) {
        if (beats > 0) spacingSum += (i - lastPeak);
        lastPeak = i;
        beats++;
      }
    }
    if (beats < 4) return NAN;

    float meanSpacing = (float)spacingSum / (beats - 1);
    float bpm = 60.0f / (meanSpacing * PPG_DT);
    if (bpm < 35.0f || bpm > 200.0f) return NAN;
    return bpm;
  }

  /*
    Respiratory rate from the breathing modulation of the same signal.

    The pulse amplitude rises and falls with the breath. Taking the envelope of
    the pulsatile signal and counting its zero crossings gives breaths per
    minute without a second sensor. It is less reliable than a chest band and
    that is worth saying plainly — but RespiGuard scores a rise from baseline,
    and this is steady enough to see a 20 % rise.
  */
  float respiratoryRate() {
    if (filled_ < PPG_WINDOW) return NAN;

    static float sig[PPG_WINDOW];
    detrend(irBuf_, sig);

    // Rectify, then smooth hard. What survives is the breathing envelope; the
    // heartbeat itself is far above this cutoff and averages away.
    for (int i = 0; i < PPG_WINDOW; i++) sig[i] = fabsf(sig[i]);
    smooth(sig, PPG_HZ / 2);

    float mean = 0;
    for (int i = 0; i < PPG_WINDOW; i++) mean += sig[i];
    mean /= PPG_WINDOW;

    // Each breath crosses the mean once on the way up, so the spacing between
    // upward crossings is the breath period.
    //
    // Counting crossings and dividing by the window length looks equivalent
    // and is not: an integer count over a fixed 8 s window can only ever
    // produce multiples of 7.5 breaths per minute, so a normal resting rate of
    // 12 to 14 is not representable at all. Since the device scores a 20 %
    // rise over the wearer's own baseline, that quantisation turns a
    // continuous signal into a coarse on/off switch. Averaging the interval
    // gives a continuous estimate and makes the sanity range below mean
    // something.
    // Crossings are taken with hysteresis rather than against the bare mean.
    // The envelope still carries ripple, and a signal wandering either side of
    // a single threshold registers one breath several times - measured on
    // synthetic traces, that alone put the answer out by a factor of two at
    // low rates. Requiring the envelope to rise past mean + 0.35 sd, and to
    // fall back below mean - 0.35 sd before the next crossing can count,
    // brings the worst error across 12 to 32 breaths a minute from 18 down to
    // under 1.
    double var = 0;
    for (int i = 0; i < PPG_WINDOW; i++) { double d = sig[i] - mean; var += d * d; }
    float sd = (float)sqrt(var / PPG_WINDOW);
    if (sd < 1e-6f) return NAN;

    float hi = mean + sd * 0.35f, lo = mean - sd * 0.35f;
    int   minGap = PPG_HZ * 60 / 45;          // 45 breaths a minute, the ceiling
    bool  armed = sig[0] < lo;
    int   lastCross = -1, prevCross = -minGap, breaths = 0;
    long  gapSum = 0;

    for (int i = 1; i < PPG_WINDOW; i++) {
      if (armed && sig[i] > hi && (i - prevCross) >= minGap) {
        if (lastCross >= 0) { gapSum += (i - lastCross); breaths++; }
        lastCross = i;
        prevCross = i;
        armed = false;
      } else if (!armed && sig[i] < lo) {
        armed = true;
      }
    }
    if (breaths < 1) return NAN;

    float meanGap = (float)gapSum / breaths;           // samples per breath
    float rr = 60.0f / (meanGap * PPG_DT);
    if (rr < 5.0f || rr > 45.0f) return NAN;
    return rr;
  }

 private:
  uint32_t redBuf_[PPG_WINDOW] = {0};
  uint32_t irBuf_[PPG_WINDOW]  = {0};
  int  head_ = 0, filled_ = 0;
  bool present_ = false;

  /* Remove the slow baseline so only the pulsatile part is left. */
  void detrend(const uint32_t *src, float *dst) {
    double mean = 0;
    for (int i = 0; i < PPG_WINDOW; i++) mean += src[i];
    mean /= PPG_WINDOW;
    for (int i = 0; i < PPG_WINDOW; i++) dst[i] = (float)(src[i] - mean);
  }

  void smooth(float *x, int w) {
    if (w < 2) return;
    static float tmp[PPG_WINDOW];
    float acc = 0;
    for (int i = 0; i < PPG_WINDOW; i++) {
      acc += x[i];
      if (i >= w) acc -= x[i - w];
      tmp[i] = acc / min(i + 1, w);
    }
    memcpy(x, tmp, sizeof(float) * PPG_WINDOW);
  }

  float stddev(const float *x) {
    double m = 0;
    for (int i = 0; i < PPG_WINDOW; i++) m += x[i];
    m /= PPG_WINDOW;
    double v = 0;
    for (int i = 0; i < PPG_WINDOW; i++) { double d = x[i] - m; v += d * d; }
    return (float)sqrt(v / PPG_WINDOW);
  }

  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }

  uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(MAX30102_ADDR, 1) != 1) return 0;
    return Wire.read();
  }

  void clearFifo() {
    writeReg(REG_FIFO_WR_PTR, 0);
    writeReg(REG_OVF_COUNTER, 0);
    writeReg(REG_FIFO_RD_PTR, 0);
  }
};
