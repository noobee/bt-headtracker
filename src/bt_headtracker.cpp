// BT Headtracker for ESP32-S3 Supermini
#include <Arduino.h>

#include <EEPROM.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include "SensorFusion.h"
#include <BleGamepad.h>

#ifdef ARDUINO_USB_MODE
#define Serial USBSerial
#endif

const int PIN_BATTERY = 4; // via 2:1 voltage divider 

DFRobot_BMI160 bmi160;
BleGamepad *pble_gamepad;
bool show_imu = false;
bool show_batt = false;
long bt_update_interval = 50e3L; // 50ms
long batt_update_interval = 10e6L; // 10s

struct {
  float gx_offset;
  float gy_offset;
  float gz_offset;
  unsigned long checksum;
} __attribute__((packed)) bt_headtracker_config;

unsigned long compute_bt_headtracker_config_checksum() {
  unsigned long sum = 0;
  for (unsigned char *p=(unsigned char *)&bt_headtracker_config; p<(unsigned char *)&bt_headtracker_config.checksum; p++)
    sum += *p;
  return sum + 0xa5a5a5a5;
}

void show_bt_headtracker_config() {
  Serial.printf("bt_headtracker_config:\n"
                "gxyz_offset=%7.4f %7.4f %7.4f  checksum=%xh\n", 
    bt_headtracker_config.gx_offset, 
    bt_headtracker_config.gy_offset, 
    bt_headtracker_config.gz_offset,
    bt_headtracker_config.checksum);
}

void write_bt_headtracker_config() {
  Serial.printf("saving calibration to eeprom.\n");
  bt_headtracker_config.checksum = compute_bt_headtracker_config_checksum();
  EEPROM.put(0, bt_headtracker_config);
  EEPROM.commit();
}

void clear_bt_headtracker_config() {
  Serial.printf("clearing bt_headtracker_config.\n");
  bt_headtracker_config.gx_offset = 0.0; 
  bt_headtracker_config.gy_offset = 0.0; 
  bt_headtracker_config.gz_offset = 0.0;

  write_bt_headtracker_config();
}

void calibrate_gyro() {

  int16_t accelGyro[6] = {0};
  float cal[3] = {0.0}, cal_min[3] = {1.0e6}, cal_max[3] = {-1.0e6};
  const int total = 500;

  for (int i=9; i>=0; i--) {
    Serial.printf("keep device still. starting gyro calibration in %d sec.\n", i);
    if (i > 0)
      delay(1000);
  }

  for (int i=0; i<total; i++) {
    if (i % 50 == 0) {
      Serial.printf(".");
      neopixelWrite(RGB_BUILTIN, 0, i/50 % 2 ? RGB_BRIGHTNESS : 0, 0); // green / black
    }

    int rslt = bmi160.getAccelGyroData(accelGyro);
    for (int j=0; j<3; j++) {
      cal[j] += (float)accelGyro[j];
      cal_min[j] = min(cal_min[j], (float)accelGyro[j]);
      cal_max[j] = max(cal_max[j], (float)accelGyro[j]);
    }
  }
  neopixelWrite(RGB_BUILTIN, 0, 0, 0);

  for (int j=0; j<3; j++) {
    cal[j] /= total;
  }

  Serial.printf("\ncalibration avgxyz= %7.4f, %7.4f, %7.4f  minxyz= %7.4f, %7.4f, %7.4f  maxxyz= %7.4f, %7.4f, %7.4f\n", 
    cal[0], cal[1], cal[2],
    cal_min[0], cal_min[1], cal_min[2],
    cal_max[0], cal_max[1], cal_max[2]);

  bt_headtracker_config.gx_offset = cal[0];
  bt_headtracker_config.gy_offset = cal[1];
  bt_headtracker_config.gz_offset = cal[2];  

  write_bt_headtracker_config();
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
  } else if (cmd == "batt") {
    show_batt = not show_batt;
    batt_update_interval = show_batt ? 1e6L : 10e6L; // 1s or 10s
  } else if (cmd == "imu") {
    show_imu = not show_imu;
  } else if (cmd == "config") {
    show_bt_headtracker_config();
  } else if (cmd == "clear") {
    clear_bt_headtracker_config();
  } else if (cmd == "calibrate") {
    calibrate_gyro();
  } else {
    Serial.printf("unknown command: %s\n", cmd.c_str());
  }
  cmd.clear();
}

void setup() {
  Serial.begin(115200);

  digitalWrite(RGB_BUILTIN, HIGH); // white

  EEPROM.begin(sizeof(bt_headtracker_config));
  EEPROM.get(0, bt_headtracker_config);
  if (compute_bt_headtracker_config_checksum() != bt_headtracker_config.checksum)
    clear_bt_headtracker_config();

  Wire.begin();
  while (bmi160.I2cInit(0x69) != BMI160_OK) {
    Serial.printf("bmi160.I2cInit() failed.\n");
    neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0, 0); // red
    delay(500);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
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
}

void loop() {
  static SF sensor_fusion;

  serial_check();

  int16_t accelGyro[6] = {0};
  int rslt = bmi160.getAccelGyroData(accelGyro);

  // apply calibration offsets
  float gx = accelGyro[0] - bt_headtracker_config.gx_offset;
  float gy = accelGyro[1] - bt_headtracker_config.gy_offset; 
  float gz = accelGyro[2] - bt_headtracker_config.gz_offset;

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
  
  static long last_imu_t;
  if (micros() - last_imu_t > 100000) {
    last_imu_t = micros();
    if (show_imu)
      Serial.printf("gxyz=[%6.2f %6.2f %6.2f] + axyz=[%6.2f %6.2f %6.2f] ==> PRY=[%6.2f %6.2f %6.2f]\n", 
        gx, gy, gz, ax, ay, az,
        pitch, roll, yaw);
  }
 
  static long last_batt_t = -batt_update_interval;
  if (micros() - last_batt_t > batt_update_interval) {
    last_batt_t = micros();
    int mv = analogReadMilliVolts(PIN_BATTERY) * 2; // *2 due to 2:1 voltage divider
    int level = (mv/1000.0 - 3.7) / (4.2-3.7) * 100; // [3.7-4.2V] maps to [0-100%]
    level = max(level, 0);
    level = min(level, 100);
    pble_gamepad->setBatteryLevel(level);

    if (show_batt)
      Serial.printf("batt=%5.2fV %d%%\n", mv/1000.0, level);
  } 

  static long last_bt_update_t = -bt_update_interval;
  if (micros() - last_bt_update_t > bt_update_interval) {
    last_bt_update_t = micros();
    if (pble_gamepad->isConnected()) {
      static int led;
      led = (led + 1) % 8;
      neopixelWrite(RGB_BUILTIN, 0, 0, led < 4 ? RGB_BRIGHTNESS : 0); // blue / black
      pble_gamepad->setX((roll+90)/180*32767);
      pble_gamepad->setY((pitch+90)/180*32767);
      pble_gamepad->setZ(yaw/360*32767);
      pble_gamepad->sendReport();
    }

  }
}
