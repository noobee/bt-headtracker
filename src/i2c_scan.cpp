#include <Arduino.h>
#include <Wire.h>

void scan_i2c() {
  delay(2000);
  Serial.printf("begin scanning I2C bus.\n");
  for (int i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.printf("=== found device at address 0x%02x\n", i);

      switch(i) {
        case 0x29:
          Serial.printf("likely bno055 with sd0=sd1=0\n");
          break;
        case 0x68:
          Serial.printf("possible bmi160 with sd0=0\n");
          Serial.printf("likely mpu6050 with ad0=0\n");
          Serial.printf("possible icm20948 with ad0=0\n");
          break;
        case 0x69:
          Serial.printf("likely bmi160 with sd0=1\n");
          Serial.printf("possible mpu6050 with ad1=0. mag hmc5883l local i2c = 0x0c?\n");
          Serial.printf("likely icm20948 with ad0=1. mag local i2c = 0x0c\n");
          break;
        case 0x76:
          Serial.printf("likely ms5611 with csb=1\n");
          break;
        case 0x77:
          Serial.printf("likely bmp280\n");
          Serial.printf("possible ms5611 with csb=0\n");
          break;
        default:
          Serial.printf("unknown device.\n");
          break;

      }
    }
  }
  Serial.printf("end scanning I2C bus.\n");
}
