#include <DFRobot_BMI160.h>

#include "bt_headtracker.h"

DFRobot_BMI160 bmi160;

void bmi160_setup() {
    while (bmi160.I2cInit(0x69) != BMI160_OK) {
        Serial.printf("bmi160.I2cInit() failed.\n");
        update_led(ERROR);
        delay(500);
        update_led(IDLE);
        delay(500);
    }
} 

void bmi160_get(struct _sensor_data *data) {
    int16_t accelGyro[6] = {0};

    bmi160.getAccelGyroData(accelGyro);

    // apply calibration offsets
    for (int j=0; j<3; j++) {
        data->gyro[j] = (float) accelGyro[j]      - 0.0; // no offset applied
        data->accel[j] = (float) accelGyro[j + 3] - 0.0; // no offset applied
    }

    for (int j=0; j<3; j++) {
        data->gyro[j] *= 2000.0 / 32768; // 32768 == 2000 deg per sec, convert to deg per sec
        data->gyro[j] *= 3.14159265359 / 180; // convert to radians per sec
        data->accel[j] *= 2.0 / 32768.0; // 32768 == 2G, convert to G
        data->accel[j] *= 9.80665; // convert to m per sec^2
    }

    // shuffle axes for sensor fusion
    // sensor (x, y, z) -> fusion (z, y, -x)
    float tmp;
    tmp = data->gyro[0];
    data->gyro[0] = data->gyro[2];
    data->gyro[2] = -tmp;
    tmp = data->accel[0];
    data->accel[0] = data->accel[2];
    data->accel[2] = -tmp;
    // no mag to shuffle
}

void bmi160_test() {

    while (1) {
        struct _sensor_data sensor_data = {0.0};
        bmi160_get(&sensor_data);

        Serial.printf("G %6.3f %6.3f %6.3f  A %6.3f %6.3f %6.3f\n",
            sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
            sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2]);

        delay(100);
    }
}