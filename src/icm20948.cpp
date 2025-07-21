// ensure ICM_20948_USE_DMP is defined in platformio.ini or in util\ICM_20948_C.h

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <utility/imumaths.h> // from bno055 library
#include "bt_headtracker.h"

ICM_20948_I2C myICM;

void icm20948_setup() {
    //  myICM.enableDebugging();

    while (myICM.begin(Wire, 0) != ICM_20948_Stat_Ok) {
        update_led(ERROR);
        delay(500);
        update_led(IDLE);
        delay(500);
    }

    myICM.initializeDMP();
    myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION);

    myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 2 /*(55/11)-1*/); // (dmp_rate / odr) - 1
    
    myICM.enableFIFO();
    myICM.enableDMP();
    myICM.resetDMP();
    myICM.resetFIFO();
}

void icm20948_test() {

    while (1) {

        // essentially poll for most recent DMP data
        icm_20948_DMP_data_t data;
        do {
            myICM.readDMPdataFromFIFO(&data);
        } while (myICM.status != ICM_20948_Stat_Ok); // loop if ICM_20948_Stat_FIFONoDataAvail, ICM_20948_Stat_FIFOIncompleteData or ICM_20948_Stat_FIFOMoreDataAvail
                
        if (data.header & DMP_header_bitmap_Quat9) {

            double x = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double y = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double z = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            double w = sqrt(1.0 - ((x * x) + (y * y) + (z * z)));

            imu::Quaternion quat = imu::Quaternion(w, x, y, z);
            imu::Quaternion rotq;
            rotq.fromAxisAngle(imu::Vector<3>(0, 1, 0), 90 * M_PI / 180.0);
            imu::Quaternion quat2 = quat * rotq;
            imu::Vector<3> euler = quat.toEuler();
            imu::Vector<3> euler2 = quat2.toEuler();

            Serial.printf("QUAT %6.3f %6.3f %6.3f %6.3f  EULER %6.1f %6.1f %6.1f  QUAT2 %6.3f %6.3f %6.3f %6.3f  EULER2 %6.1f %6.1f %6.1f  acc %d\n",
                quat.x(), quat.y(), quat.z(), quat.w(),
                euler.x() * 180.0 / M_PI, euler.y() * 180.0 / M_PI, euler.z() * 180.0 / M_PI,
                quat2.x(), quat2.y(), quat2.z(), quat2.w(),
                euler2.x() * 180.0 / M_PI, euler2.y() * 180.0 / M_PI, euler2.z() * 180.0 / M_PI,
                data.Quat9.Data.Accuracy); 
        }

        delay(100);
    }
}