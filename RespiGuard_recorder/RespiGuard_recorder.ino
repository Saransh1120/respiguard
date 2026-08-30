/*
  RespiGuard — recording sketch.

  Streams raw microphone audio over USB so that normal breathing can be captured
  on the finished device and used to fine-tune the wheeze classifier. This is the
  one piece of the machine-learning plan that cannot be done without hardware:
  the model was trained on a stethoscope pressed to the chest, and the INMP441
  sits under clothing. Recordings made here are what close that gap.

  Only the negative class is collected — ordinary breathing, from whoever is
  wearing it. That raises no ethics question and needs no consent process, which
  is exactly why it is the half of the problem worth solving first.

  The audio is sent at 4 kHz, matching prepare_icbhi.py, so the recordings drop
  straight into the training set without resampling. A 16-bit sample at 4 kHz is
  64 kbit/s, comfortably inside the serial link, so nothing has to be compressed
  or buffered to an SD card.

  Wiring is the same as the main firmware: BCLK 4, WS 5, SD 6, and the mic's L/R
  pin to ground.

  Use:
    1. Flash this sketch.
    2. Close the Arduino serial monitor — it will hold the port otherwise.
    3. Run  python record.py --seconds 30 --label rest
    4. Press the button, or just start the script; recording runs until it stops.
*/

#include <driver/i2s.h>

#define PIN_I2S_BCLK 4
#define PIN_I2S_WS   5
#define PIN_I2S_SD   6
#define PIN_LED_RED  16
#define PIN_BUTTON   18

// Matches prepare_icbhi.py. Recording at the training rate means the captured
// audio needs no resampling later, and resampling is a step where a quiet
// mismatch with the training features could creep in.
static const int SAMPLE_RATE = 4000;
static const int CHUNK = 256;

// A short header on every chunk. Without it, a byte dropped on the wire would
// shift every following sample by one and turn the recording into noise that
// still looks like audio.
static const uint8_t MAGIC[4] = {0x52, 0x47, 0x41, 0x31};   // "RGA1"

void setup() {
  // The bit rate here is for the USB CDC link, not the audio. It is set high so
  // the serial link is never the bottleneck.
  Serial.begin(921600);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = SAMPLE_RATE;
  // The INMP441 places a 24-bit sample inside a 32-bit slot. Reading 32 and
  // shifting down is correct; reading 16 takes the wrong end of the word and
  // returns something that looks like noise but is not.
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = CHUNK;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = PIN_I2S_BCLK;
  pins.ws_io_num = PIN_I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = PIN_I2S_SD;

  if (i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL) != ESP_OK ||
      i2s_set_pin(I2S_NUM_0, &pins) != ESP_OK) {
    // Fast blink forever: the host script will see no data and this says why.
    while (true) {
      digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
      delay(100);
    }
  }
}

void loop() {
  static int32_t raw[CHUNK];
  static int16_t out[CHUNK];

  size_t bytesRead = 0;
  if (i2s_read(I2S_NUM_0, raw, sizeof(raw), &bytesRead, portMAX_DELAY) != ESP_OK) return;
  int got = bytesRead / sizeof(int32_t);
  if (got <= 0) return;

  // 24-bit sample sitting in the top of a 32-bit word, brought down to 16 bits.
  // The shift is 16 rather than 8 because the low byte of the 24 is below the
  // mic's own noise floor and carries nothing.
  for (int i = 0; i < got; i++) out[i] = (int16_t)(raw[i] >> 16);

  Serial.write(MAGIC, 4);
  uint16_t n = got;
  Serial.write((uint8_t *)&n, 2);
  Serial.write((uint8_t *)out, got * sizeof(int16_t));

  // Slow heartbeat, so it is obvious at a glance that audio is still flowing.
  digitalWrite(PIN_LED_RED, (millis() / 500) % 2);
}
