#include <Wire.h>
#include <SensirionGasIndexAlgorithm.h>
#include <VOCGasIndexAlgorithm.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define SGP40_ADDR 0x59

GasIndexAlgorithmParams vocParams;

// CRC8 (polynomial 0x31)
uint8_t sgp40_crc(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

uint16_t measureRawSGP40() {
  uint8_t rh[2] = {0x80, 0x00};   // 50% RH
  uint8_t t[2]  = {0x66, 0x66};   // 25°C

  Wire.beginTransmission(SGP40_ADDR);
  Wire.write(0x26);
  Wire.write(0x0F);

  Wire.write(rh, 2);
  Wire.write(sgp40_crc(rh, 2));

  Wire.write(t, 2);
  Wire.write(sgp40_crc(t, 2));

  Wire.endTransmission();
  delay(30);

  Wire.requestFrom(SGP40_ADDR, 3);
  if (Wire.available() != 3) return 0;

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  uint8_t crc = Wire.read();

  uint8_t data[2] = {msb, lsb};
  if (sgp40_crc(data, 2) != crc) return 0;

  return (msb << 8) | lsb;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // initialize algorithm in VOC mode
  GasIndexAlgorithm_init(&vocParams, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

  Serial.println("SGP40 VOC Index example started");
}

void loop() {
  uint16_t sraw = measureRawSGP40();

  if (sraw == 0) {
    Serial.println("SGP40 read error");
    delay(1000);
    return;
  }

  int32_t vocIndex = 0;
  GasIndexAlgorithm_process(&vocParams, sraw, &vocIndex);

  Serial.print("SRAW: ");
  Serial.print(sraw);
  Serial.print(" | VOC Index: ");
  Serial.println(vocIndex);

  delay(1000);   // must be 1Hz
}
