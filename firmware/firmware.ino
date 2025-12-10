#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "driver/i2s.h"

// PCM5102A pins
#define I2S_BCK_IO 16
#define I2S_LRCK_IO 14
#define I2S_DATA_IO 15

// MSM261DGT-003 PDM mic pins
#define MIC_CLK 4
#define MIC_DATA 5

// microSD (SPI)
#define SD_CS   10
#define SD_MOSI 11
#define SD_CLK  12
#define SD_MISO 13

void allDevices(int addr) {
  Serial.printf("device 0x%02X\n", addr);

  for (int reg = 0; reg < 0xFF; reg++) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { continue; }
    if (Wire.requestFrom(addr, (int)1) != 1) { continue; }

    int val = Wire.read();
    Serial.printf("Reg 0x%02X = 0x%02X\n", reg, val);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // PCM5102A: I2S TX (I2S_NUM_0)
  i2s_config_t i2s_tx_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t i2s_tx_pins = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_LRCK_IO,
      .data_out_num = I2S_DATA_IO,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_tx_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &i2s_tx_pins);

  // PDM Microphone: I2S RX (I2S_NUM_1)
  i2s_config_t i2s_rx_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 128,
      .use_apll = false
  };

  i2s_pin_config_t i2s_rx_pins = {
      .mck_io_num = I2S_PIN_NO_CHANGE,
      .bck_io_num = MIC_CLK,         // PDM clock
      .ws_io_num = I2S_PIN_NO_CHANGE,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = MIC_DATA           // PDM data
  };

  i2s_driver_install(I2S_NUM_1, &i2s_rx_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &i2s_rx_pins);

  // Required for PDM
  i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  // microSD setup
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card mount failed!");
  } else {
    Serial.println("SD card OK.");
  }
}

void loop() {

  // I2C device scan
  Serial.println("\nI2C devices...");
  for (int addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      allDevices(addr);
    }
  }

  delay(500);

  // Sine wave to PCM5102A
  static const int amplitude = 3000;
  static const float freq = 440.0;  // A4
  static float phase = 0.0;

  int16_t samples[256 * 2]; // stereo

  for (int i = 0; i < 256; i++) {
    float value = amplitude * sin(phase);
    int16_t sample = (int16_t)value;

    samples[i * 2]     = sample; // Left
    samples[i * 2 + 1] = sample; // Right

    phase += (2.0f * PI * freq) / 44100.0f;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;
  }

  size_t bytes_written;
  i2s_write(I2S_NUM_0, samples, sizeof(samples), &bytes_written, portMAX_DELAY);

  // Mic read test (prints audio level)
  int16_t mic_buffer[256];
  size_t bytes_read;

  i2s_read(I2S_NUM_1, mic_buffer, sizeof(mic_buffer), &bytes_read, 10);

  long sum = 0;
  int count = bytes_read / 2;

  for (int i = 0; i < count; i++) {
    sum += abs(mic_buffer[i]);
  }

  int level = (count > 0) ? (sum / count) : 0;
  Serial.print("Mic Level: ");
  Serial.println(level);
}
