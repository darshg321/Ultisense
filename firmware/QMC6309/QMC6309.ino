#include <Wire.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define QMC6309_ADDR 0x7C   // 7-bit address from datasheet

// write register
void qmc6309_write(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(QMC6309_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// read multiple registers
void qmc6309_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(QMC6309_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC6309_ADDR, len);

  for (uint8_t i=0;i<len;i++)
    buf[i]=Wire.read();
}

// initialize QMC6309
void qmc6309_init()
{
  delay(10);

  // Soft reset
  qmc6309_write(0x0B,0x80);
  delay(2);
  qmc6309_write(0x0B,0x00);

  // Set Range ±32G, ODR 200Hz, Set/Reset ON
  qmc6309_write(0x0B,0x40);

  // OSR1=8 OSR2=8 NORMAL MODE
  qmc6309_write(0x0A,0x61);
}

// read magnetometer raw
bool readMagRaw(int16_t &x,int16_t &y,int16_t &z,uint8_t &status)
{
  qmc6309_read(0x09,&status,1);

  if(!(status & 0x01))
    return false;

  uint8_t buf[6];
  qmc6309_read(0x01,buf,6);

  x=(int16_t)(buf[1]<<8|buf[0]);
  y=(int16_t)(buf[3]<<8|buf[2]);
  z=(int16_t)(buf[5]<<8|buf[4]);

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(100000);

  qmc6309_init();

  Serial.println("QMC6309 Magnetometer Started");
}

void loop()
{
  int16_t x,y,z;
  uint8_t status;

  if(!readMagRaw(x,y,z,status))
  {
    Serial.println("No new data");
    delay(1000);
    return;
  }

  // sensitivity for ±32G = 1000 LSB/G
  float xG=x/1000.0;
  float yG=y/1000.0;
  float zG=z/1000.0;

  float heading=atan2(yG,xG)*180.0/PI;
  if(heading<0) heading+=360;

  Serial.print("RAW X:");
  Serial.print(x);
  Serial.print(" Y:");
  Serial.print(y);
  Serial.print(" Z:");
  Serial.print(z);

  Serial.print(" | Gauss X:");
  Serial.print(xG,3);
  Serial.print(" Y:");
  Serial.print(yG,3);
  Serial.print(" Z:");
  Serial.print(zG,3);

  Serial.print(" | Heading:");
  Serial.print(heading);

  Serial.print(" | DRDY:");
  Serial.print(status&0x01);
  Serial.print(" OVFL:");
  Serial.print((status>>1)&0x01);

  Serial.println();

  delay(1000);
}