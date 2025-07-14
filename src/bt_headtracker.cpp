// BT Headtracker for ESP32-S3 Supermini
#include <Arduino.h>

#include <EEPROM.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "SensorFusion.h"
#include <BleGamepad.h>

#if ARDUINO_USB_MODE
#define Serial USBSerial
#endif


DFRobot_BMI160 bmi160;
BleGamepad *pble_gamepad;
int16_t gyro_bias_offset[3] = {0}; // gyro bias offsets for calibration
const int batt_pin = 4; // via 2:1 voltage divider 
int batt_mv = 0; // battery voltage in mv
int batt_level = 0; // battery level in percent
long batt_update_interval = 10e6L; // 10s
const long bt_update_interval = 50e3L; // 50ms
bool show_stat = false;

enum LED_STATE {INIT, IDLE, ERROR, BT, GCAL};
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

const int16_t max_still_gyro = 50; // max gyro value when still, in raw gyro units
const int calibrate_count = 200; // consecutive still samples to accept

void calibrate_gyro(int16_t gyro[]) {
  static int count;
  static int gyro_sum[3];

  if (abs(gyro[0]) > max_still_gyro || abs(gyro[1]) > max_still_gyro || abs(gyro[2]) > max_still_gyro) {
    count = 0;
    return;
  }

  if (count == 0) {
    for (int j=0; j<3; j++) {
      gyro_sum[j] = gyro[j];
    }
  } else {
    for (int j=0; j<3; j++) {
      gyro_sum[j] += gyro[j];
    }
  }

  count++;

  if (count == calibrate_count) {
    for (int j=0; j<3; j++) {
      gyro_bias_offset[j] = gyro_sum[j] / count;
    }
    count = 0;
    update_led(GCAL);
    delay(100);
    update_led(IDLE);
  }
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
  } else {
    Serial.printf("unknown command [%s]\n"
      "available commands:\n"
      "[restart] restart esp32\n"
      "[stat]    toggle showing status messages\n",
      cmd.c_str());
  }
  cmd.clear();
}

void setup() {
  Serial.begin(115200);

  update_led(INIT);

  Wire.begin();
  while (bmi160.I2cInit(0x69) != BMI160_OK) {
    Serial.printf("bmi160.I2cInit() failed.\n");
    update_led(ERROR);
    delay(500);
    update_led(IDLE);
    delay(500);
  }

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

  update_led(IDLE);
}

void loop() {
  static SF sensor_fusion;

  long t = micros();

  serial_check();

  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);

  calibrate_gyro(accelGyro);

  // apply calibration offsets
  float gx = accelGyro[0] - gyro_bias_offset[0];
  float gy = accelGyro[1] - gyro_bias_offset[1]; 
  float gz = accelGyro[2] - gyro_bias_offset[2];

  float ax = accelGyro[3];
  float ay = accelGyro[4];
  float az = accelGyro[5];

  // 32768 == 2000 deg per sec, convert to deg per sec
  gx *= 2000.0 / 32768;
  gy *= 2000.0 / 32768;
  gz *= 2000.0 / 32768;

  // convert to radians per sec
  gx *= 3.14159265359 / 180;
  gy *= 3.14159265359 / 180;
  gz *= 3.14159265359 / 180;

  // 32768 == 2G, convert to G
  ax *= 2.0 / 32768;
  ay *= 2.0 / 32768;
  az *= 2.0 / 32768;

  // convert to m per sec^2
  ax *= 9.80665;
  ay *= 9.80665;
  az *= 9.80665;

  static float delta_t;
  delta_t = sensor_fusion.deltatUpdate();
  // 9dof sensor GAM
  //sensor_fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, delta_t);  //mahony is suggested if there isn't the mag
  //sensor_fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, delta_t);  //else use the magwick
  
  // 6dof sensor GA
  //sensor_fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, delta_t);  // flat
  sensor_fusion.MadgwickUpdate(gz, gy, -gx, az, ay, -ax, delta_t);  // rotated

  float pitch, roll, yaw;
  pitch = sensor_fusion.getPitch(); // -90 to +90
  roll = sensor_fusion.getRoll(); // -90 to +90
  yaw = sensor_fusion.getYaw(); // 0 to 360
  
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
    Serial.printf("gxyz=[%6.2f %6.2f %6.2f] axyz=[%6.2f %6.2f %6.2f] PRY=[%7.2f %7.2f %7.2f] gbias=[%3d %3d %3d] batt=[%4.2fV %d%%]\n", 
      gx, gy, gz, ax, ay, az, 
      pitch, roll, yaw,
      gyro_bias_offset[0], gyro_bias_offset[1], gyro_bias_offset[2],
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
