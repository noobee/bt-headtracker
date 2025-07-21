#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "bt_headtracker.h"


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

void bno055_setup() {
    while (!bno.begin()) {
        update_led(ERROR);
        delay(500);
        update_led(IDLE);
        delay(500);
    }
  delay(100);
  bno.setExtCrystalUse(true);
}


void bno055_get(struct _orientation *data) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    imu::Quaternion quat = bno.getQuat();
    imu::Quaternion rotq;
    rotq.fromAxisAngle(imu::Vector<3>(0, 1, 0), -90 * M_PI / 180.0);
    quat = quat * rotq;
    imu::Vector<3> euler = quat.toEuler();

    data->pitch = euler.y() * 180.0 / M_PI;
    data->roll = euler.z() * 180.0 / M_PI;
    data->yaw = euler.x() * 180.0 / M_PI + 180;
}

void bno055_test() {

  while (1) {
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    imu::Quaternion quat = bno.getQuat();

    int8_t temp = bno.getTemp();
    uint8_t cal_sys, cal_gyro, cal_accel, cal_mag = 0;
    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    if (0) {
      Serial.printf("QUAT %6.3f %6.3f %6.3f %6.3f  ORIENT %6.1f %6.1f %6.1f  GYRO %6.3f %6.3f %6.3f  LINACCEL %6.3f %6.3f %6.3f  MAG %6.2f %6.2f %6.2f  ACCEL %6.3f %6.3f %6.3f  GVTY %6.3f %6.3f %6.3f  TEMP %2d  C_SGAM %d %d %d %d\n",
        quat.x(), quat.y(), quat.z(), quat.w(),

        orientationData.orientation.x, orientationData.orientation.y, orientationData.orientation.z,
        angVelocityData.gyro.x, angVelocityData.gyro.y, angVelocityData.gyro.z,
        linearAccelData.acceleration.x, linearAccelData.acceleration.y, linearAccelData.acceleration.z,
        magnetometerData.magnetic.x, magnetometerData.magnetic.y, magnetometerData.magnetic.z,
        accelerometerData.acceleration.x, accelerometerData.acceleration.y, accelerometerData.acceleration.z,
        gravityData.acceleration.x, gravityData.acceleration.y, gravityData.acceleration.z,
        temp,
        cal_sys, cal_gyro, cal_accel, cal_mag);
    }

    if (1) {
      imu::Quaternion rotq;
      rotq.fromAxisAngle(imu::Vector<3>(0, 1, 0), 90 * M_PI / 180.0);

      imu::Quaternion quat2 = quat * rotq;

      imu::Vector<3> euler = quat.toEuler();
      imu::Vector<3> euler2 = quat2.toEuler();

      Serial.printf("QUAT %6.3f %6.3f %6.3f %6.3f  EULER %6.1f %6.1f %6.1f  QUAT2 %6.3f %6.3f %6.3f %6.3f  EULER2 %6.1f %6.1f %6.1f  acc=%04x\n",
        quat.x(), quat.y(), quat.z(), quat.w(),
        euler.x() * 180.0 / M_PI, euler.y() * 180.0 / M_PI, euler.z() * 180.0 / M_PI,
        quat2.x(), quat2.y(), quat2.z(), quat2.w(),
        euler2.x() * 180.0 / M_PI, euler2.y() * 180.0 / M_PI, euler2.z() * 180.0 / M_PI,
        cal_sys << 12 |  cal_gyro << 8 | cal_accel << 4 | cal_mag); 
    }    

    delay(100);
  }
}