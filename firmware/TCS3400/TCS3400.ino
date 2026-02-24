#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9

#define TCS3400_ADDR 0x39

// Registers
#define ENABLE_REG   0x80
#define ATIME_REG    0x81
#define CONTROL_REG  0x8F
#define STATUS_REG   0x93

#define CDATAL       0x94
#define RDATAL       0x96
#define GDATAL       0x98
#define BDATAL       0x9A
#define IR_REG       0xC0

// ENABLE bits
#define PON  0x01
#define AEN  0x02

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCS3400_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister8(uint8_t reg) {
  Wire.beginTransmission(TCS3400_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TCS3400_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(TCS3400_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TCS3400_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  return (high << 8) | low;
}

void initTCS3400() {
  writeRegister(ENABLE_REG, PON);
  delay(10);

  writeRegister(ATIME_REG, 0xF6);      // ~27.8ms integration
  writeRegister(CONTROL_REG, 0x02);    // 16x gain

  writeRegister(ENABLE_REG, PON | AEN);
  delay(50);
}

void readAllChannels(
  uint16_t &clear,
  uint16_t &red,
  uint16_t &green,
  uint16_t &blue,
  uint16_t &ir
) {
  clear = readRegister16(CDATAL);
  red   = readRegister16(RDATAL);
  green = readRegister16(GDATAL);
  blue  = readRegister16(BDATAL);

  // Enable IR mapping to clear channel
  writeRegister(IR_REG, 0x80);
  delay(5);
  ir = readRegister16(CDATAL);
  writeRegister(IR_REG, 0x00);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Started");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  initTCS3400();
  Serial.println("TCS34001FNM Color Sensor Started");
}

void loop() {
  uint16_t clear, red, green, blue, ir;
  readAllChannels(clear, red, green, blue, ir);

  if (clear == 0) {
    Serial.println("Read error or no light");
    delay(1000);
    return;
  }

  // Normalize RGB to 0–255 scale
  float rNorm = (float)red   / clear;
  float gNorm = (float)green / clear;
  float bNorm = (float)blue  / clear;

  uint8_t r8 = constrain(rNorm * 255, 0, 255);
  uint8_t g8 = constrain(gNorm * 255, 0, 255);
  uint8_t b8 = constrain(bNorm * 255, 0, 255);

  uint32_t visible = (clear > ir) ? (clear - ir) : 0;

  float lux = (0.136 * red) +
              (1.000 * green) +
              (-0.444 * blue);

  // Determine dominant color
  String dominant;
  if (r8 > g8 && r8 > b8) dominant = "Red";
  else if (g8 > r8 && g8 > b8) dominant = "Green";
  else if (b8 > r8 && b8 > g8) dominant = "Blue";
  else dominant = "Neutral / White";

  Serial.println("------ RAW DATA ------");
  Serial.print("Clear: "); Serial.println(clear);
  Serial.print("Red:   "); Serial.println(red);
  Serial.print("Green: "); Serial.println(green);
  Serial.print("Blue:  "); Serial.println(blue);
  Serial.print("IR:    "); Serial.println(ir);

  Serial.println("------ HUMAN READABLE ------");
  Serial.print("RGB (0-255): ");
  Serial.print(r8); Serial.print(", ");
  Serial.print(g8); Serial.print(", ");
  Serial.println(b8);

  Serial.print("Visible Counts: ");
  Serial.println(visible);

  Serial.print("Estimated Lux: ");
  Serial.println(lux);

  Serial.print("Dominant Color: ");
  Serial.println(dominant);

  delay(1000);
}