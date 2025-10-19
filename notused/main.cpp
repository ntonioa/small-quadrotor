#include <Arduino.h>
#include <SPI.h>
#include "ICM_20948.h"
#include "predict_state.h"
#include "jacobian_F.h"
#include "predict_measurement.h"
#include "jacobian_H.h"
#include "predict.h"
#include "update.h"

#define CS_PIN 5    // D1 su Wemos D1 Mini32
#define SCK_PIN 18  // D5
#define MISO_PIN 19 // D6
#define MOSI_PIN 23 // D7

ICM_20948_SPI imu;

// Struttura per calibrazione
struct {
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
  float magOffset[3] = {0};
} calibration;

// Stato e matrici EKF
float state[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // [q_w, q_x, q_y, q_z, wx, wy, wz]
float dt = 0.05; // 50ms (20 Hz)
float covariance[49] = {
    0.01, 0, 0, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0, 0, 0,
    0, 0, 0.01, 0, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0, 0.1
};
float process_noise[49] = {
    0.0001, 0, 0, 0, 0, 0, 0,
    0, 0.0001, 0, 0, 0, 0, 0,
    0, 0, 0.0001, 0, 0, 0, 0,
    0, 0, 0, 0.0001, 0, 0, 0,
    0, 0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0, 0.01
};
float measurement[6]; // [wx, wy, wz, mx, my, mz]
float measurement_noise[36] = {
    0.05, 0, 0, 0, 0, 0,
    0, 0.05, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.1, 0, 0,
    0, 0, 0, 0, 0.1, 0,
    0, 0, 0, 0, 0, 0.1
};

void calibrateSensor(int samples = 500) {
  float accelSum[3] = {0};
  float gyroSum[3] = {0};
  float magSum[3] = {0};
  int validSamples = 0;

  Serial.println("Calibrazione in corso... (non muovere il sensore)");
  
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      
      accelSum[0] += imu.accX();
      accelSum[1] += imu.accY();
      accelSum[2] += imu.accZ() - 1000.0; // Sottrai 1g (1000 mg)
      
      gyroSum[0] += imu.gyrX();
      gyroSum[1] += imu.gyrY();
      gyroSum[2] += imu.gyrZ();
      
      magSum[0] += imu.magX();
      magSum[1] += imu.magY();
      magSum[2] += imu.magZ();
      
      validSamples++;
      delay(10);
    }
  }

  // Calcola offset medi
  if(validSamples > 0) {
    for(int i = 0; i < 3; i++) {
      calibration.accelOffset[i] = accelSum[i] / validSamples;
      calibration.gyroOffset[i] = gyroSum[i] / validSamples;
      calibration.magOffset[i] = magSum[i] / validSamples;
    }
  }

  Serial.println("Calibrazione completata:");
  Serial.printf("Accel Offset: %.2f, %.2f, %.2f\n", calibration.accelOffset[0], calibration.accelOffset[1], calibration.accelOffset[2]);
  Serial.printf("Gyro Offset: %.2f, %.2f, %.2f\n", calibration.gyroOffset[0], calibration.gyroOffset[1], calibration.gyroOffset[2]);
  Serial.printf("Mag Offset: %.2f, %.2f, %.2f\n", calibration.magOffset[0], calibration.magOffset[1], calibration.magOffset[2]);
}

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10); // Aspetta che la seriale sia pronta

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  while(true) {
    if(imu.begin(CS_PIN, SPI) == ICM_20948_Stat_Ok) {
      Serial.println("IMU connessa correttamente");
      break;
    }
    Serial.println("Connessione IMU fallita, riprovo...");
    delay(500);
  }

  // Configura magnetometro
  ICM_20948_fss_t fss;
  fss.a = ACCEL_SENSITIVITY_2G; // Sensibilità accelerometro
  fss.g = GYRO_SENSITIVITY_250DPS; // Sensibilità giroscopio
  imu.setFullScale(fss);
  imu.enableMagDataAccess(); // Abilita accesso al magnetometro

  calibrateSensor(); // Esegui calibrazione iniziale
}

void loop() {
  if(imu.dataReady()) {
    imu.getAGMT();
    
    // Applica correzione
    float accX = imu.accX() - calibration.accelOffset[0];
    float accY = imu.accY() - calibration.accelOffset[1];
    float accZ = imu.accZ() - calibration.accelOffset[2];
    
    float gyrX = imu.gyrX() - calibration.gyroOffset[0];
    float gyrY = imu.gyrY() - calibration.gyroOffset[1];
    float gyrZ = imu.gyrZ() - calibration.gyroOffset[2];
    
    float magX = imu.magX() - calibration.magOffset[0];
    float magY = imu.magY() - calibration.magOffset[1];
    float magZ = imu.magZ() - calibration.magOffset[2];

    // Filtraggio semplice (media mobile)
    static float smoothAcc[3] = {0};
    static float smoothGyr[3] = {0};
    static float smoothMag[3] = {0};
    const float filterFactor = 0.2;
    
    smoothAcc[0] = (1 - filterFactor) * smoothAcc[0] + filterFactor * accX;
    smoothAcc[1] = (1 - filterFactor) * smoothAcc[1] + filterFactor * accY;
    smoothAcc[2] = (1 - filterFactor) * smoothAcc[2] + filterFactor * accZ;
    
    smoothGyr[0] = (1 - filterFactor) * smoothGyr[0] + filterFactor * gyrX;
    smoothGyr[1] = (1 - filterFactor) * smoothGyr[1] + filterFactor * gyrY;
    smoothGyr[2] = (1 - filterFactor) * smoothGyr[2] + filterFactor * gyrZ;
    
    smoothMag[0] = (1 - filterFactor) * smoothMag[0] + filterFactor * magX;
    smoothMag[1] = (1 - filterFactor) * smoothMag[1] + filterFactor * magY;
    smoothMag[2] = (1 - filterFactor) * smoothMag[2] + filterFactor * magZ;

    // Popola vettore misure per EKF
    measurement[0] = smoothGyr[0] * (PI / 180.0); // Converti in rad/s
    measurement[1] = smoothGyr[1] * (PI / 180.0);
    measurement[2] = smoothGyr[2] * (PI / 180.0);
    measurement[3] = smoothMag[0];
    measurement[4] = smoothMag[1];
    measurement[5] = smoothMag[2];

    // Esegui passo di previsione
    float new_state[7];
    float new_covariance[49];
    ekf::predict(state, dt, covariance, process_noise, new_state, new_covariance);
    
    // Aggiorna stato e covarianza
    memcpy(state, new_state, sizeof(float) * 7);
    memcpy(covariance, new_covariance, sizeof(float) * 49);

    // Esegui passo di aggiornamento
    ekf::update(state, covariance, measurement, measurement_noise, new_state, new_covariance);
    
    // Aggiorna stato e covarianza
    memcpy(state, new_state, sizeof(float) * 7);
    memcpy(covariance, new_covariance, sizeof(float) * 49);

    // Stampa stato per debug
    Serial.printf("Quaternione: %.4f, %.4f, %.4f, %.4f | Gyro: %.4f, %.4f, %.4f\n",
                  state[0], state[1], state[2], state[3],
                  state[4], state[5], state[6]);
  }
  delay(50); // Frequenza ~20Hz
}