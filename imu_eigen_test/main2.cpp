#include <Arduino.h>
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#include <Eigen/Dense>
#include <SPI.h>
#include "ICM_20948.h"
#include <math.h>
#include "predict.h"
#include "update.h"
using namespace sym;

// ---- CONFIG ----
#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

ICM_20948_SPI imu;

// Eigen types
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec6 = Eigen::Matrix<float, 6, 1>;
using Mat77 = Eigen::Matrix<float, 7, 7>;
using Mat66 = Eigen::Matrix<float, 6, 6>;

// EKF globals
Vec7 x;     // stato [qw,qx,qy,qz,bx,by,bz]
Mat77 P;    // covarianza
Mat77 Q;    // rumore processo
Mat66 R;    // rumore misura

unsigned long last_time;

// Struct per calibrazione
struct {
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
  float magOffset[3] = {0};
  float magScale[3] = {1, 1, 1};
} calibration;

// ----------- CALIBRAZIONE ------------

// Calibrazione accelerometro/giroscopio: tieni il drone FERMO!
void calibrateAccelGyro(int samples = 500) {
  float accelSum[3] = {0};
  float gyroSum[3] = {0};
  Serial.println("\n*** Calibrazione accelerometro/giroscopio (NON MUOVERE IL DRONE) ***");
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      accelSum[0] += imu.accX();
      accelSum[1] += imu.accY();
      accelSum[2] += imu.accZ() - 1000.0; // sottrai 1g (1000 mg)
      gyroSum[0] += imu.gyrX();
      gyroSum[1] += imu.gyrY();
      gyroSum[2] += imu.gyrZ();
      delay(5);
    }
  }
  for(int i = 0; i < 3; i++) {
    calibration.accelOffset[i] = accelSum[i] / samples;
    calibration.gyroOffset[i] = gyroSum[i] / samples;
  }
  Serial.print("Accel Offset: ");
  Serial.print(calibration.accelOffset[0],2); Serial.print(" ");
  Serial.print(calibration.accelOffset[1],2); Serial.print(" ");
  Serial.println(calibration.accelOffset[2],2);
  Serial.print("Gyro Offset: ");
  Serial.print(calibration.gyroOffset[0],3); Serial.print(" ");
  Serial.print(calibration.gyroOffset[1],3); Serial.print(" ");
  Serial.println(calibration.gyroOffset[2],3);
}

// Calibrazione magnetometro: ruota il drone in TUTTE le direzioni (anche capovolto) per 30-60s!
void calibrateMagnetometer(int samples = 1000) {
  float magMin[3], magMax[3];
  Serial.println("\n*** Calibrazione magnetometro: ruota in tutte le direzioni ***");
  // Inizializza con prima lettura
  while(!imu.dataReady()) delay(1);
  imu.getAGMT();
  magMin[0] = magMax[0] = imu.magX();
  magMin[1] = magMax[1] = imu.magY();
  magMin[2] = magMax[2] = imu.magZ();
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      magMin[0] = min(magMin[0], imu.magX());
      magMax[0] = max(magMax[0], imu.magX());
      magMin[1] = min(magMin[1], imu.magY());
      magMax[1] = max(magMax[1], imu.magY());
      magMin[2] = min(magMin[2], imu.magZ());
      magMax[2] = max(magMax[2], imu.magZ());
      delay(5);
    }
  }
  // Offset e scale
  for(int i=0; i<3; i++)
    calibration.magOffset[i] = (magMax[i]+magMin[i])/2.0f;
  float avgDelta = ((magMax[0]-magMin[0])+(magMax[1]-magMin[1])+(magMax[2]-magMin[2]))/6.0f;
  for(int i=0; i<3; i++)
    calibration.magScale[i] = avgDelta / ((magMax[i]-magMin[i])/2.0f);

  Serial.print("Mag Offset: ");
  Serial.print(calibration.magOffset[0],2); Serial.print(" ");
  Serial.print(calibration.magOffset[1],2); Serial.print(" ");
  Serial.println(calibration.magOffset[2],2);
  Serial.print("Mag Scale: ");
  Serial.print(calibration.magScale[0],3); Serial.print(" ");
  Serial.print(calibration.magScale[1],3); Serial.print(" ");
  Serial.println(calibration.magScale[2],3);
}

// ----------- SETUP ------------

void setup() {
  Serial.begin(115200);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  while(true) {
    imu.begin(CS_PIN, SPI);
    if(imu.status == ICM_20948_Stat_Ok) {
      Serial.println("IMU connessa correttamente");
      break;
    }
    Serial.println("Connessione IMU fallita, riprovo...");
    delay(500);
  }

  imu.swReset(); delay(10);
  imu.sleep(false); imu.lowPower(false);
  imu.startupMagnetometer();
  delay(100);

  calibrateAccelGyro();
  calibrateMagnetometer();

  // EKF inizializzazione
  x << 1,0,0,0, 0,0,0; // [qw,qx,qy,qz, bias...]
  P.setIdentity();  P *= 0.01f;
  Q.setIdentity();  Q *= 0.1f;
  R.setIdentity();  R *= 1.0f;

  last_time = millis();
  Serial.println("Setup completo, ready!");
}

// ----------- LOOP ------------

void loop() {
  if (!imu.dataReady()) {
    delay(2);
    return;
  }

  // Calcola dt
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  imu.getAGMT();

  // Lettura e calibrazione
  float accX = imu.accX() - calibration.accelOffset[0];
  float accY = imu.accY() - calibration.accelOffset[1];
  float accZ = imu.accZ() - calibration.accelOffset[2];
  float gyrX = imu.gyrX() - calibration.gyroOffset[0];
  float gyrY = imu.gyrY() - calibration.gyroOffset[1];
  float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];
  float magX = (imu.magX() - calibration.magOffset[0]) * calibration.magScale[0];
  float magY = (imu.magY() - calibration.magOffset[1]) * calibration.magScale[1];
  float magZ = (imu.magZ() - calibration.magOffset[2]) * calibration.magScale[2];

  // Debug: stampa raw sensori (giroscopio e magnetometro calibrati)
  Serial.print("GYRO: ");
  Serial.print(gyrX, 3); Serial.print(" ");
  Serial.print(gyrY, 3); Serial.print(" ");
  Serial.println(gyrZ, 3);

  Serial.print("MAG: ");
  Serial.print(magX, 2); Serial.print(" ");
  Serial.print(magY, 2); Serial.print(" ");
  Serial.println(magZ, 2);

  // Costruisci vettore misura z = [gyro; mag]
  Vec6 z;
  z << gyrX, gyrY, gyrZ, magX, magY, magZ;

  // EKF predict/update (usa le tue funzioni generate da symforce)
  Vec7 x_pred; Mat77 P_pred;
  Predict(x, dt, P, Q, &x_pred, &P_pred);

  Vec7 x_upd; Mat77 P_upd;
  Update(x_pred, P_pred, z, R, &x_upd, &P_upd);

  // Normalizza il quaternione!
  x_upd.head<4>().normalize();
  x = x_upd;
  P = P_upd;

  // Estrai roll/pitch/yaw (in gradi)
  float qw = x(0), qx = x(1), qy = x(2), qz = x(3);
  float roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
  float pitch = asin (2*(qw*qy - qz*qx));
  float yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

  Serial.print("EKF RPY: ");
  Serial.print(roll  * 180.0/M_PI, 1); Serial.print(" ");
  Serial.print(pitch * 180.0/M_PI, 1); Serial.print(" ");
  Serial.println(yaw   * 180.0/M_PI, 1);

  delay(10); // loop veloce, puoi aumentare se vuoi
}
