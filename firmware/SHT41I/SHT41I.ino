#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define SHT41_ADDR 0x44

// SHT4x commands
#define CMD_MEAS_HIGHREP 0xFD  // high repeatability (highest precision)
#define CMD_MEAS_MEDREP  0xF6  // medium repeatability
#define CMD_MEAS_LOWREP  0xE0  // low repeatability

// CRC8 (polynomial 0x31), same parameters as Sensirion datasheet
uint8_t sht_crc(uint8_t *data, uint8_t len) {
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

// Send a single-byte command to the sensor
bool sht_write_command(uint8_t cmd) {
  Wire.beginTransmission(SHT41_ADDR);
  Wire.write(cmd);
  if (Wire.endTransmission() != 0) return false;
  return true;
}

// Read one measurement: returns true if read+CRC OK and fills t_ticks/rh_ticks
bool measureRawSHT41(uint16_t &t_ticks, uint16_t &rh_ticks, uint8_t repeatability = CMD_MEAS_HIGHREP) {
  // send measurement command
  if (!sht_write_command(repeatability)) return false;

  // wait for measurement to complete
  // datasheet suggests ~7-9 ms for high, ~3.7-4.5 ms for med, ~1.3-1.6 ms for low
  // use safe delay of 10 ms for high, 5 ms for med, 2 ms for low
  if (repeatability == CMD_MEAS_HIGHREP) delay(10);
  else if (repeatability == CMD_MEAS_MEDREP) delay(6);
  else delay(3);

  // request 6 bytes: T_MSB, T_LSB, CRC, RH_MSB, RH_LSB, CRC
  Wire.requestFrom((uint8_t)SHT41_ADDR, (uint8_t)6);
  if (Wire.available() != 6) return false;

  uint8_t tb0 = Wire.read();
  uint8_t tb1 = Wire.read();
  uint8_t crc_t = Wire.read();
  uint8_t rb0 = Wire.read();
  uint8_t rb1 = Wire.read();
  uint8_t crc_r = Wire.read();

  uint8_t tmp_t[2] = { tb0, tb1 };
  uint8_t tmp_r[2] = { rb0, rb1 };

  // validate CRCs
  if (sht_crc(tmp_t, 2) != crc_t) return false;
  if (sht_crc(tmp_r, 2) != crc_r) return false;

  t_ticks = ((uint16_t)tb0 << 8) | tb1;
  rh_ticks = ((uint16_t)rb0 << 8) | rb1;
  return true;
}

// Read serial number (command 0x89) - returns true if OK and fills serial (32-bit)
bool readSerial(uint32_t &serial) {
  const uint8_t CMD_READ_SERIAL = 0x89;
  if (!sht_write_command(CMD_READ_SERIAL)) return false;
  delay(1); // tiny wait

  Wire.requestFrom((uint8_t)SHT41_ADDR, (uint8_t)6);
  if (Wire.available() != 6) return false;

  uint8_t s0 = Wire.read();
  uint8_t s1 = Wire.read();
  uint8_t crc0 = Wire.read();
  uint8_t s2 = Wire.read();
  uint8_t s3 = Wire.read();
  uint8_t crc1 = Wire.read();

  uint8_t part0[2] = { s0, s1 };
  uint8_t part1[2] = { s2, s3 };

  if (sht_crc(part0, 2) != crc0) return false;
  if (sht_crc(part1, 2) != crc1) return false;

  // Combine words into 32-bit serial (word0 MSB..LSB then word1 MSB..LSB)
  serial = ((uint32_t)s0 << 24) | ((uint32_t)s1 << 16) | ((uint32_t)s2 << 8) | (uint32_t)s3;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("SHT41I measurement example started");
}

void loop() {
  uint16_t t_ticks = 0, rh_ticks = 0;
  if (!measureRawSHT41(t_ticks, rh_ticks, CMD_MEAS_HIGHREP)) {
    Serial.println("SHT41 read error (measurement failed or CRC mismatch)");
    delay(1000);
    return;
  }

  // print raw ticks
  Serial.print("T ticks: 0x");
  Serial.print(t_ticks, HEX);
  Serial.print(" (");
  Serial.print(t_ticks);
  Serial.println(")");
  Serial.print("RH ticks: 0x");
  Serial.print(rh_ticks, HEX);
  Serial.print(" (");
  Serial.print(rh_ticks);
  Serial.println(")");

  // convert to physical values using datasheet formulas
  // Use 65535.0 (2^16 - 1) as denominator to match datasheet conversion
  float t_degC = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);
  float t_degF = -49.0f + 315.0f * ((float)t_ticks / 65535.0f); // datasheet formula for °F
  float rh_pct_uncropped = -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);
  float rh_pct = rh_pct_uncropped;
  if (rh_pct > 100.0f) rh_pct = 100.0f;
  if (rh_pct < 0.0f) rh_pct = 0.0f;
  float aw = rh_pct_uncropped / 100.0f; // water activity (aw) as per datasheet (1/100 factor)

  // print converted values
  Serial.print("Temperature: ");
  Serial.print(t_degC, 3);
  Serial.print(" °C | ");
  Serial.print(t_degF, 3);
  Serial.println(" °F");

  Serial.print("Relative Humidity (uncropped): ");
  Serial.print(rh_pct_uncropped, 3);
  Serial.println(" %RH");

  Serial.print("Relative Humidity (cropped 0..100): ");
  Serial.print(rh_pct, 3);
  Serial.println(" %RH");

  Serial.print("Water activity (aw): ");
  Serial.println(aw, 6);

  // optional: print serial number once
  static bool printedSerial = false;
  if (!printedSerial) {
    uint32_t serial = 0;
    if (readSerial(serial)) {
      Serial.print("SHT4x Serial: 0x");
      Serial.println(serial, HEX);
    } else {
      Serial.println("Failed to read serial number or CRC mismatch");
    }
    printedSerial = true;
  }

  Serial.println("----------------------------------------");
  delay(1000); // 1 Hz loop
}