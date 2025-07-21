// BT Headtracker for ESP32-S3 Supermini
#include <Arduino.h>

#include <EEPROM.h>
#include <Wire.h>
#include "SensorFusion.h"
#include <BleGamepad.h>

#include "bt_headtracker.h"

BleGamepad *pble_gamepad;
float gyro_bias_offset[3] = {0.0}; // gyro bias offsets for calibration
const int batt_pin = 4; // via 2:1 voltage divider 
int batt_mv = 0; // battery voltage in mv
int batt_level = 0; // battery level in percent
long batt_update_interval = 10e6L; // 10s
const long bt_update_interval = 50e3L; // 50ms
bool show_stat = false;

enum LED_STATE led_state;
void update_led(enum LED_STATE new_led_state)
{
  if (new_led_state == led_state)
    return;

  led_state = new_led_state;
  switch (led_state) {
    case INIT:
      neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS); // white
      // alternative digitalWrite(RGB_BUILTIN, HIGH); // white
      break;
    case IDLE:
      neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS/8, RGB_BRIGHTNESS/8, 0); // yellow
      break;
    case ERROR:
      neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // red
      break;
    case BT:
      neopixelWrite(RGB_BUILTIN, 0, 0, RGB_BRIGHTNESS); // blue
      break;
    case GCAL:
      neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); // green
      break;
  }
}


int calibrate_gyro() {
  struct _sensor_data sensor_data;
  float cal_sum[3] = {0};
  float cal_min[3] = {__FLT_MAX__}, cal_max[3] = {-__FLT_MAX__};
  const int total = 500;

  update_led(GCAL);
  for (int i=0; i<total; i++) {

#if BMI160
    bmi160_get(&sensor_data);
#endif
    for (int j=0; j<3; j++) {
      cal_sum[j] += sensor_data.gyro[j];
      cal_min[j] = min(cal_min[j], sensor_data.gyro[j]);
      cal_max[j] = max(cal_max[j], sensor_data.gyro[j]);

      if (cal_max[j] - cal_min[j] > 50) {
        update_led(ERROR);
        delay(100);
        return 0;
      }
    }
  }

  for (int j=0; j<3; j++) {
    gyro_bias_offset[j] = (float) cal_sum[j] / total;
  }

  Serial.printf("\ncalibration avgxyz= %7.4f %7.4f %7.4f  minxyz= %4d %4d %4d  maxxyz= %4d %4d %4d\n", 
    gyro_bias_offset[0], gyro_bias_offset[1], gyro_bias_offset[2],
    cal_min[0], cal_min[1], cal_min[2],
    cal_max[0], cal_max[1], cal_max[2]);

  update_led(IDLE);
  return 1;
}

void serial_check() {
  static String cmd;
  
  if (!Serial.available())
    return;

  char c = Serial.read();
  //Serial.printf("received '%c' (%d)\n", c, c);
  if (c != '\r') {
    cmd += c;
    return;
  }

  cmd.trim();
  if (cmd == "restart") {
    Serial.printf("restarting esp32.\n");
    ESP.restart();
  } else if (cmd == "stat") {
    show_stat = not show_stat;
    batt_update_interval = show_stat ? 1e6L : 10e6L; // 1s or 10s
#if BMI160 || MPU6050_HMC5883L
  } else if (cmd == "cal") {
    while (!calibrate_gyro()) {
    }
#endif
  } else {
    Serial.printf("unknown command [%s]\n"
      "available commands:\n"
      "[restart] restart esp32\n"
      "[stat]    toggle showing status messages\n"
      "[cal]     calibrate gyro (for BMI160 or MPU6050_HMC5883L only)\n",
      cmd.c_str());
  }
  cmd.clear();
}


void setup() {
  Serial.begin(115200);

  update_led(INIT);
  Wire.begin();
#if IMU_TEST
  scan_i2c();
#endif

#if BMI160
  bmi160_setup();
#if IMU_TEST
  bmi160_test();
#endif
#endif

#if BNO055
  bno055_setup();
#if IMU_TEST
  bno055_test();
#endif
#endif

#if ICM20948
  icm20948_setup();
#if IMU_TEST
  icm20948_test();
#endif
#endif

#if MPU6050_HMC5883L
  mpu6050_hmc5883l_setup();
#if IMU_TEST
  mpu6050_hmc5883l_test();
#endif
#endif



  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false);
  ble_gamepad_config.setControllerType(CONTROLLER_TYPE_GAMEPAD); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  ble_gamepad_config.setButtonCount(4); // in case we want to use buttons later
  ble_gamepad_config.setWhichAxes(
    true,   // X for ROLL
    true,   // Y for PITCH
    true,   // Z for YAW
    false,  // RX
    false,  // RY
    false,  // RZ
    false,  // Slider1
    false); // Slider2    
  ble_gamepad_config.setAxesMin(0);
  ble_gamepad_config.setAxesMax(32767);
  ble_gamepad_config.setVid(0x045E);
  ble_gamepad_config.setPid(0x028E);

  unsigned long mac = ESP.getEfuseMac();
  char s[80];
  snprintf(s, sizeof(s), "BT Headtracker -%04x", mac & 0xffff);
  pble_gamepad = new BleGamepad(s);
  pble_gamepad->begin(&ble_gamepad_config);

#if BMI160 || MPU6050_HMC5883L  
  while (!calibrate_gyro()) {
  }
#endif

  update_led(IDLE);
}

void loop() {
  long t = micros();

  serial_check();

  float pitch, roll, yaw;
  struct _sensor_data sensor_data = {0.0};
  struct _orientation orientation = {0.0};

#if BMI160 || MPU6050_HMC5883L

#if BMI160
  bmi160_get(&sensor_data);
#endif
#if MPU6050_HMC5883L
  mpu6050_hmc5883l_get(&sensor_data);
#endif

  static SF sensor_fusion;
  static float sensor_fusion_dt;
  sensor_fusion_dt = sensor_fusion.deltatUpdate();
  //sensor_fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, sensor_fusion_dt);  //mahony is suggested if there isn't the mag
  //sensor_fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, sensor_fusion_dt);  //else use the magwick
  sensor_fusion.MadgwickUpdate(sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2], 
                             sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2], 
                             sensor_data.mag[0], sensor_data.mag[1], sensor_data.mag[2], // may be 0.0 for 6dof sensors
                             sensor_fusion_dt);

  pitch = sensor_fusion.getPitch(); // -90 to +90
  roll = sensor_fusion.getRoll(); // -90 to +90
  yaw = sensor_fusion.getYaw() - 180; // -180 to + 180 
#endif

#if BNO055
    bno055_get(&orientation);
    pitch = orientation.pitch;
    roll = orientation.roll;
    yaw = orientation.yaw;
#endif 

  static long last_batt_t = -batt_update_interval;
  if (t - last_batt_t > batt_update_interval) {
    last_batt_t = t;
    batt_mv = analogReadMilliVolts(batt_pin) * 2; // *2 due to 2:1 voltage divider
    batt_level = (batt_mv/1000.0 - 3.70) / (4.10 - 3.70) * 100; // [3.7-4.1V] maps to [0-100%]
    batt_level = constrain(batt_level, 0, 100);
    pble_gamepad->setBatteryLevel(batt_level);
  } 

  static long last_stat_t;
  if (show_stat && t - last_stat_t > 100000) {
    last_stat_t = t;
    Serial.printf("G=[%6.2f %6.2f %6.2f] A=[%6.2f %6.2f %6.2f] M=[%6.2f %6.2f %6.2f] PRY=[%7.2f %7.2f %7.2f] batt=[%4.2fV %d%%]\n", 
      sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
      sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2],
      sensor_data.mag[0], sensor_data.mag[1], sensor_data.mag[2],
      pitch, roll, yaw,
      batt_mv/1000.0, batt_level);
  }

  static long last_bt_update_t = -bt_update_interval;
  if (t - last_bt_update_t > bt_update_interval) {
    last_bt_update_t = t;
    if (pble_gamepad->isConnected()) {
      update_led(BT);
      pble_gamepad->setX((roll+90)/180*32767);
      pble_gamepad->setY((pitch+90)/180*32767);
      pble_gamepad->setZ(yaw/360*32767);
      pble_gamepad->sendReport();
    } else {
      update_led(IDLE);
    }

  }
}
