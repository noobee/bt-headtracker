#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#include "bt_headtracker.h"

MPU6050 mpu6050;
HMC5883L hmc5883l;

float gx, gy, gz, ax, ay, az, mx, my, mz;
bool mag_enabled = true;

void mpu6050_hmc5883l_setup() {

    bool failed = false;
    mpu6050.initialize(); 
    while (!mpu6050.testConnection()) {
        update_led(ERROR);
        delay(500);
        update_led(IDLE);    
        delay(500);
    }
    mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    // gyro int16 ==> +/- 2000 deg/s and accel int16 ==> +/- 2G
    
    mpu6050.setI2CBypassEnabled(true); // set mag bypass mode

    hmc5883l.initialize();
    if (!hmc5883l.testConnection()) {
        mag_enabled = false;
        Serial.println("HMC5883L not connected.");
    }

} 


void mpu6050_hmc5883l_get(struct _sensor_data *data) {

    int16_t g[3] = {0};
    int16_t a[3] = {0};
    int16_t m[3] = {0};

    mpu6050.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);
    if (mag_enabled) {
        hmc5883l.getHeading(&m[0], &m[1], &m[2]);
        int16_t tmp = m[0];
        m[0] = m[1];
        m[1] = tmp;
        m[2] = -m[2];
    }

    // apply calibration offsets
    for (int j=0; j<3; j++) {
        data->gyro[j] = (float) g[j]  - 0.0; // no offset applied
        data->accel[j] = (float) a[j] - 0.0; // no offset applied
        data->mag[j] = (float) m[j];
    }

    for (int j=0; j<3; j++) {
        data->gyro[j] *= 2000.0 / 32768; // 32768 == 2000 deg per sec, convert to deg per sec
        data->gyro[j] *= 3.14159265359 / 180; // convert to radians per sec
        data->accel[j] *= 2.0 / 32768.0; // 32768 == 2G, convert to G
        data->accel[j] *= 9.80665; // convert to m per sec^2
    }



    #if 0
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
    #endif
}

void mpu6050_hmc5883l_test() {

    while (1) {
        struct _sensor_data sensor_data = {0.0};
        mpu6050_hmc5883l_get(&sensor_data);
        
        Serial.printf("G %6.3f %6.3f %6.3f  A %6.3f %6.3f %6.3f  M %6.3f %6.3f %6.3f\n",
            sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
            sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2],
            sensor_data.mag[0], sensor_data.mag[1], sensor_data.mag[2]);

        delay(100);
    }
}


