#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"

#define BNO08X_CS 12
#define BNO08X_INT 13
#define BNO08X_RESET 14

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void readIMU(bool printData) {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports(reportType, reportIntervalUs);
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) 
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  
      if (printData) {
          Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
          Serial.print(ypr.yaw);                Serial.print("\t");
          Serial.print(ypr.pitch);              Serial.print("\t");
          Serial.println(ypr.roll);
      }
}
  
void setupIMU() {
    //IMU Initialization
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("Adafruit BNO08x test!");

    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
    }
    Serial.println("BNO08x Found!");

    setReports(reportType, reportIntervalUs);

    Serial.println("Reading events");
    readIMU(true);
    delay(100);
}
