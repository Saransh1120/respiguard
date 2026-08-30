/*
  Wheeze detection — INMP441 to log-mel to the classifier.

  This file has one job that matters above all others: produce, on the ESP32,
  the *same* features the model was trained on in Python. A classifier fed
  features that differ from its training distribution does not degrade
  gracefully, it degrades silently — it keeps returning confident numbers that
  mean nothing. Every constant here is therefore pinned to the matching
  constant in prepare_icbhi.py and must be changed in both places or neither:

      sample rate  16000        n_fft   512      hop  256
      mel bands    40           fmin    50       fmax 8000
      window       4 s -> 251 frames

  The features are computed as a rolling stream rather than by buffering four
  seconds of audio and transforming it in one go. Four seconds at 16 kHz is
  64000 floats — a quarter of a megabyte, which this chip cannot spare next to
  the model arena. Holding 251 finished mel frames instead costs 40 KB, and the
  answer is identical because a mel frame never depends on audio outside its own
  512-sample window.

  The FFT is written out here rather than pulled from esp-dsp. It is a textbook
  radix-2 and this is the only place it is used; a dependency that has to be
  installed by hand is a worse trade for a sketch that someone else has to build.
*/

#pragma once
#include <driver/i2s.h>
#include <math.h>

// ---- geometry, mirrored from prepare_icbhi.py ----------------------------
static const int   WZ_SAMPLE_RATE = 16000;
static const int   WZ_N_FFT       = 512;
static const int   WZ_HOP         = 256;
static const int   WZ_N_MELS      = 40;
static const int   WZ_FRAMES      = 251;      // 4 s at this hop
static const float WZ_FMIN        = 50.0f;
static const float WZ_FMAX        = 8000.0f;

// ---- I2S pins, from the project pin map ----------------------------------
#define PIN_I2S_BCLK 4
#define PIN_I2S_WS   5
#define PIN_I2S_SD   6

class WheezeFrontEnd {
 public:
  bool begin() {
    buildMelFilters();
    buildHannWindow();
    buildTwiddles();

    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate = WZ_SAMPLE_RATE;
    // The INMP441 puts a 24-bit sample inside a 32-bit slot. Asking for 32 bits
    // and shifting down is correct; asking for 16 would take the wrong end of
    // the word and return something that looks like noise.
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 4;
    cfg.dma_buf_len = WZ_HOP;
    cfg.use_apll = false;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = PIN_I2S_BCLK;
    pins.ws_io_num = PIN_I2S_WS;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = PIN_I2S_SD;

    if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK) return false;
    if (i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) return false;
    present_ = true;
    return true;
  }

  bool present() const { return present_; }
  bool ready() const { return framesFilled_ >= WZ_FRAMES; }

  /*
    Read one hop of audio and turn it into one mel frame.

    Called from the main loop. At 16 kHz a hop is 16 ms, so this runs about
    sixty times a second and must stay cheap — which is why the FFT is 512-point
    and the mel filterbank is precomputed as a sparse triangle list rather than
    a 40x257 matrix multiply.
  */
  void poll() {
    if (!present_) return;

    static int32_t raw[WZ_HOP];
    size_t bytesRead = 0;
    if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, 0) != ESP_OK) return;
    int got = bytesRead / sizeof(int32_t);
    if (got < WZ_HOP) return;   // not a full hop yet

    // Slide the analysis window and append the new hop.
    memmove(audio_, audio_ + WZ_HOP, sizeof(float) * (WZ_N_FFT - WZ_HOP));
    for (int i = 0; i < WZ_HOP; i++) {
      // >> 11 brings the 24-bit sample down into a sane float range; the exact
      // scale does not matter because the patch is normalised at the end.
      audio_[WZ_N_FFT - WZ_HOP + i] = (float)(raw[i] >> 11) / 32768.0f;
    }

    melFrame(audio_, mel_[melHead_]);
    melHead_ = (melHead_ + 1) % WZ_FRAMES;
    if (framesFilled_ < WZ_FRAMES) framesFilled_++;
  }

  /*
    Copy the last four seconds of mel frames into `out` in training order,
    normalised the way prepare_icbhi.py normalises.

    `out` is [WZ_N_MELS][WZ_FRAMES], the layout the model expects.
  */
  bool patch(float *out) {
    if (framesFilled_ < WZ_FRAMES) return false;

    // The ring starts at melHead_, which is the oldest frame.
    float peak = -1e30f;
    for (int t = 0; t < WZ_FRAMES; t++) {
      const float *src = mel_[(melHead_ + t) % WZ_FRAMES];
      for (int m = 0; m < WZ_N_MELS; m++) if (src[m] > peak) peak = src[m];
    }

    // power_to_db(ref=max): decibels relative to the loudest bin in the patch,
    // floored 80 dB below it, exactly as librosa does.
    double sum = 0, sumsq = 0;
    for (int t = 0; t < WZ_FRAMES; t++) {
      const float *src = mel_[(melHead_ + t) % WZ_FRAMES];
      for (int m = 0; m < WZ_N_MELS; m++) {
        float db = 10.0f * log10f(fmaxf(src[m], 1e-10f) / fmaxf(peak, 1e-10f));
        if (db < -80.0f) db = -80.0f;
        out[m * WZ_FRAMES + t] = db;
        sum += db; sumsq += (double)db * db;
      }
    }

    // Per-patch standardisation, again matching the Python. This is what makes
    // the model indifferent to how loud the mic happens to be.
    int n = WZ_N_MELS * WZ_FRAMES;
    float mean = sum / n;
    float sd = sqrtf(fmaxf(sumsq / n - (double)mean * mean, 0.0)) + 1e-6f;
    for (int i = 0; i < n; i++) out[i] = (out[i] - mean) / sd;
    return true;
  }

 private:
  bool  present_ = false;
  float audio_[WZ_N_FFT] = {0};
  float mel_[WZ_FRAMES][WZ_N_MELS];
  int   melHead_ = 0, framesFilled_ = 0;

  float hann_[WZ_N_FFT];
  float cosTab_[WZ_N_FFT / 2], sinTab_[WZ_N_FFT / 2];

  // Sparse mel filterbank: each band is a triangle over a contiguous bin range,
  // so storing start, end and the weights is far smaller than a full matrix.
  int   melStart_[WZ_N_MELS], melEnd_[WZ_N_MELS];
  float melWeight_[WZ_N_MELS][40];   // widest triangle at this geometry

  static float hzToMel(float hz) { return 2595.0f * log10f(1.0f + hz / 700.0f); }
  static float melToHz(float m)  { return 700.0f * (powf(10.0f, m / 2595.0f) - 1.0f); }

  void buildHannWindow() {
    for (int i = 0; i < WZ_N_FFT; i++)
      hann_[i] = 0.5f - 0.5f * cosf(2.0f * PI * i / (WZ_N_FFT - 1));
  }

  void buildTwiddles() {
    for (int i = 0; i < WZ_N_FFT / 2; i++) {
      cosTab_[i] = cosf(-2.0f * PI * i / WZ_N_FFT);
      sinTab_[i] = sinf(-2.0f * PI * i / WZ_N_FFT);
    }
  }

  void buildMelFilters() {
    const int nBins = WZ_N_FFT / 2 + 1;
    float melLo = hzToMel(WZ_FMIN), melHi = hzToMel(WZ_FMAX);
    float points[WZ_N_MELS + 2];
    for (int i = 0; i < WZ_N_MELS + 2; i++) {
      float m = melLo + (melHi - melLo) * i / (WZ_N_MELS + 1);
      points[i] = melToHz(m) * WZ_N_FFT / WZ_SAMPLE_RATE;   // in FFT bins
    }
    for (int b = 0; b < WZ_N_MELS; b++) {
      int lo = (int)floorf(points[b]);
      int hi = (int)ceilf(points[b + 2]);
      if (lo < 0) lo = 0;
      if (hi > nBins - 1) hi = nBins - 1;
      melStart_[b] = lo;
      melEnd_[b] = hi;
      for (int k = lo; k <= hi && (k - lo) < 40; k++) {
        float w = 0.0f;
        if (k >= points[b] && k <= points[b + 1])
          w = (k - points[b]) / fmaxf(points[b + 1] - points[b], 1e-6f);
        else if (k > points[b + 1] && k <= points[b + 2])
          w = (points[b + 2] - k) / fmaxf(points[b + 2] - points[b + 1], 1e-6f);
        melWeight_[b][k - lo] = w;
      }
    }
  }

  /* In-place radix-2 complex FFT. Real input, imaginary part zeroed. */
  void fft(float *re, float *im) {
    const int n = WZ_N_FFT;
    for (int i = 1, j = 0; i < n; i++) {
      int bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { float t;
        t = re[i]; re[i] = re[j]; re[j] = t;
        t = im[i]; im[i] = im[j]; im[j] = t;
      }
    }
    for (int len = 2; len <= n; len <<= 1) {
      int step = n / len;
      for (int i = 0; i < n; i += len) {
        for (int k = 0; k < len / 2; k++) {
          float wr = cosTab_[k * step], wi = sinTab_[k * step];
          int a = i + k, b = i + k + len / 2;
          float xr = re[b] * wr - im[b] * wi;
          float xi = re[b] * wi + im[b] * wr;
          re[b] = re[a] - xr; im[b] = im[a] - xi;
          re[a] += xr;        im[a] += xi;
        }
      }
    }
  }

  /* One windowed FFT, folded down into WZ_N_MELS mel powers. */
  void melFrame(const float *frame, float *outMel) {
    static float re[WZ_N_FFT], im[WZ_N_FFT], power[WZ_N_FFT / 2 + 1];

    for (int i = 0; i < WZ_N_FFT; i++) { re[i] = frame[i] * hann_[i]; im[i] = 0.0f; }
    fft(re, im);
    for (int k = 0; k <= WZ_N_FFT / 2; k++) power[k] = re[k] * re[k] + im[k] * im[k];

    for (int b = 0; b < WZ_N_MELS; b++) {
      float acc = 0.0f;
      for (int k = melStart_[b]; k <= melEnd_[b] && (k - melStart_[b]) < 40; k++)
        acc += power[k] * melWeight_[b][k - melStart_[b]];
      outMel[b] = acc;
    }
  }
};
