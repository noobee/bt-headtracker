#include <Arduino.h>

#if ARDUINO_USB_MODE
#define Serial USBSerial
#endif

struct _sensor_data {
  float gyro[3]; // x, y, z
  float accel[3]; // x, y, z
  float mag[3]; // x, y, z
};

struct _orientation {
    float pitch; // -90 to 90 
    float roll; // -90 to 90
    float yaw; // 0 to 360
};

enum LED_STATE {INIT, IDLE, ERROR, BT, GCAL};
void update_led(enum LED_STATE new_led_state);

void scan_i2c();

void bmi160_setup();
void bmi160_get(struct _sensor_data *data);
void bmi160_test();

void bno055_setup();
void bno055_get(struct _orientation *data);
void bno055_test();

void icm20948_setup();
void icm20948_test();

void mpu6050_hmc5883l_setup();
void mpu6050_hmc5883l_get(struct _sensor_data *data);
void mpu6050_hmc5883l_test();
