#include <Arduino.h>
#undef B0
#undef B1
#undef B2
#undef B3
#undef C0
#undef C1
#undef C2
#undef C3

#include <SPI.h>
#include "ICM_20948.h"
#include <math.h>

// Assicuriamoci di avere std::result_of
#include <functional>

// Disabilitiamo il codice SIMD e le assert di allineamento
#define _GLIBCXX_USE_DEPRECATED_RESULT_OF 1
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
using namespace Eigen;


#include "predict.h"
#include "update.h"

using namespace sym;



#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

ICM_20948_SPI imu;

// Tipi Eigen
using Vec7 = Eigen::Matrix<float, 7, 1>;
using Vec6 = Eigen::Matrix<float, 6, 1>;
using Mat77 = Eigen::Matrix<float, 7, 7>;
using Mat66 = Eigen::Matrix<float, 6, 6>;

// EKF globals
Vec7 x;     // stato [qw,qx,qy,qz,bx,by,bz]
Mat77 P;    // covarianza
Mat77 Q;    // processo
Mat66 R;    // misura

unsigned long last_time;

// Struttura per calibrazione
struct {
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
  float magOffset[3] = {0};
  float magScale[3] = {1, 1, 1};
} calibration;

// Struttura per orientamento
struct {
  float roll = 0;
  float pitch = 0;
  float yaw = 0;
} orientation;

void calibrateMagnetometer(int samples = 500) {
  float magMin[3] = {0, 0, 0};
  float magMax[3] = {0, 0, 0};
  
  Serial.println("Calibrazione magnetometro: ruota il dispositivo in tutte le direzioni");
  
  // Prima lettura per inizializzare min/max
  if(imu.dataReady()) {
    imu.getAGMT();
    magMin[0] = magMax[0] = imu.magX();
    magMin[1] = magMax[1] = imu.magY();
    magMin[2] = magMax[2] = imu.magZ();
  }
  
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      
      // Aggiorna minimi e massimi
      magMin[0] = min(magMin[0], imu.magX());
      magMax[0] = max(magMax[0], imu.magX());
      magMin[1] = min(magMin[1], imu.magY());
      magMax[1] = max(magMax[1], imu.magY());
      magMin[2] = min(magMin[2], imu.magZ());
      magMax[2] = max(magMax[2], imu.magZ());
      
      delay(10);
    }
  }
  
  // Calcola offset e fattori di scala
  calibration.magOffset[0] = (magMax[0] + magMin[0]) / 2;
  calibration.magOffset[1] = (magMax[1] + magMin[1]) / 2;
  calibration.magOffset[2] = (magMax[2] + magMin[2]) / 2;
  
  float avgDelta = 0;
  avgDelta += (magMax[0] - magMin[0]) / 2;
  avgDelta += (magMax[1] - magMin[1]) / 2;
  avgDelta += (magMax[2] - magMin[2]) / 2;
  avgDelta /= 3;
  
  calibration.magScale[0] = avgDelta / ((magMax[0] - magMin[0]) / 2);
  calibration.magScale[1] = avgDelta / ((magMax[1] - magMin[1]) / 2);
  calibration.magScale[2] = avgDelta / ((magMax[2] - magMin[2]) / 2);
  
  Serial.println("Calibrazione magnetometro completata:");
  Serial.print("Offset X:"); Serial.print(calibration.magOffset[0]);
  Serial.print(" Y:"); Serial.print(calibration.magOffset[1]);
  Serial.print(" Z:"); Serial.println(calibration.magOffset[2]);
  Serial.print("Scale X:"); Serial.print(calibration.magScale[0]);
  Serial.print(" Y:"); Serial.print(calibration.magScale[1]);
  Serial.print(" Z:"); Serial.println(calibration.magScale[2]);
}

void calibrateAccelGyro(int samples = 500) {
  float accelSum[3] = {0};
  float gyroSum[3] = {0};

  Serial.println("Calibrazione accelerometro/giroscopio... (non muovere il sensore)");
  
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      
      accelSum[0] += imu.accX();
      accelSum[1] += imu.accY();
      accelSum[2] += imu.accZ() - 1000.0; // Sottrai 1g (1000 mg)
      
      gyroSum[0] += imu.gyrX();
      gyroSum[1] += imu.gyrY();
      gyroSum[2] += imu.gyrZ();
      
      delay(10);
    }
  }

  // Calcola offset medi
  for(int i = 0; i < 3; i++) {
    calibration.accelOffset[i] = accelSum[i] / samples;
    calibration.gyroOffset[i] = gyroSum[i] / samples;
  }

  Serial.println("Calibrazione accelerometro/giroscopio completata:");
  Serial.print("Accel Offset X:"); Serial.print(calibration.accelOffset[0]);
  Serial.print(" Y:"); Serial.print(calibration.accelOffset[1]);
  Serial.print(" Z:"); Serial.println(calibration.accelOffset[2]);
}

void calculateOrientation(float accX, float accY, float accZ, float magX, float magY, float magZ) {
  // Normalizza i valori dell'accelerometro
  float accNorm = sqrt(accX * accX + accY * accY + accZ * accZ);
  accX /= accNorm;
  accY /= accNorm;
  accZ /= accNorm;

  // Calcolo Roll e Pitch dagli accelerometri (in radianti)
  orientation.roll = atan2(accY, accZ);
  orientation.pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
  
  // Compensazione del magnetometro per Roll e Pitch
  float cosRoll = cos(orientation.roll);
  float sinRoll = sin(orientation.roll);
  float cosPitch = cos(orientation.pitch);
  float sinPitch = sin(orientation.pitch);
  
  // Tilt compensation
  float magXcomp = magX * cosPitch + magZ * sinPitch;
  float magYcomp = magX * sinRoll * sinPitch + magY * cosRoll - magZ * sinRoll * cosPitch;
  
  // Calcolo Yaw dal magnetometro (in radianti)
  orientation.yaw = atan2(-magYcomp, magXcomp);
  
  // Converti in gradi
  orientation.roll *= 180.0 / M_PI;
  orientation.pitch *= 180.0 / M_PI;
  orientation.yaw *= 180.0 / M_PI;
  
  // Normalizza Yaw tra 0 e 360 gradi
  if (orientation.yaw < 0) orientation.yaw += 360.0;
}

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

  // Configurazione di base IMU
  imu.swReset();
  delay(10);
  imu.sleep(false);
  imu.lowPower(false);
  
  // Configurazione magnetometro con metodo ufficiale
  imu.startupMagnetometer();
  
  calibrateAccelGyro();
  calibrateMagnetometer();

    // ——— Inizializza EKF ———
  x << 1,0,0,0,   // quaternion unitario
       0,0,0;    // bias gyro iniziali
  P.setIdentity();  P *= 0.01f;
  Q.setIdentity();  Q *= 0.001f;
  R.setIdentity();
  R.block<3,3>(0,0) *= 0.01f;   // gyro noise
  R.block<3,3>(3,3) *= 0.05f;   // mag noise

  last_time = millis();
}

void loop() {
  if (!imu.dataReady()) {
    delay(5);
    return;
  }

  // 1. Calcolo dt
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  imu.getAGMT();
  // 2. Leggi e calibra sensori
  float accX = imu.accX() - calibration.accelOffset[0];
  float accY = imu.accY() - calibration.accelOffset[1];
  float accZ = imu.accZ() - calibration.accelOffset[2];
  float gyrX = imu.gyrX() - calibration.gyroOffset[0];
  float gyrY = imu.gyrY() - calibration.gyroOffset[1];
  float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];
  float magX = (imu.magX() - calibration.magOffset[0]) * calibration.magScale[0];
  float magY = (imu.magY() - calibration.magOffset[1]) * calibration.magScale[1];
  float magZ = (imu.magZ() - calibration.magOffset[2]) * calibration.magScale[2];

  // 3. Costruisci vettore misura z = [gyro; mag]
  Vec6 z;
  z << gyrX, gyrY, gyrZ, magX, magY, magZ;

  // 4. EKF PREDICT
  Vec7 x_pred;
  Mat77 P_pred;
  Predict(x, dt, P, Q, &x_pred, &P_pred);

  // 5. EKF UPDATE
  Vec7 x_upd;
  Mat77 P_upd;
  Update(x_pred, P_pred, z, R, &x_upd, &P_upd);

  // 6. Normalizza quaternion e salva stato
  x_upd.head<4>().normalize();
  x = x_upd;
  P = P_upd;

  // 7. (opzionale) Stampa roll/pitch/yaw filtrati
  float qw = x(0), qx = x(1), qy = x(2), qz = x(3);
  float roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy));
  float pitch = asin (2*(qw*qy - qz*qx));
  float yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

  Serial.print("EKF RPY: ");
  Serial.print(roll  * 180.0/M_PI, 1); Serial.print(" ");
  Serial.print(pitch * 180.0/M_PI, 1); Serial.print(" ");
  Serial.println(yaw   * 180.0/M_PI, 1);
}
