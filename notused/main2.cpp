// #include <Arduino.h>
// #include "icm20948.h"
// #include <SPI.h>

// #define CS_PIN 5  // Chip Select personalizzato
// #define SPI_CLK 18
// #define SPI_MISO 19
// #define SPI_MOSI 23

// ICM20948 imu(CS_PIN);  // Usa il costruttore SPI

// void setup() {
//   Serial.begin(115200);
//   delay(2000);
  
//   // Inizializza SPI
//   SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, CS_PIN);
//   SPI.setFrequency(1000000);  // 1 MHz clock
  
//   Serial.println("Inizializzazione IMU via SPI...");
//   if (!imu.beginSPI()) {  // Usa la funzione specifica per SPI
//     Serial.println("ERRORE: IMU non rilevata!");
//     while(1);
//   }
  
//   Serial.println("IMU inizializzata correttamente via SPI!");
// }

// void loop() {
//   int16_t accX, accY, accZ, gyrX, gyrY, gyrZ;
//   imu.readRawDataSPI(accX, accY, accZ, gyrX, gyrY, gyrZ);  // Funzione specifica SPI
  
//   Serial.printf("A: %6d %6d %6d | G: %6d %6d %6d\n",
//                accX, accY, accZ, gyrX, gyrY, gyrZ);
//   delay(100);
// }


// #include <Arduino.h>
// #include <SPI.h>
// #include <ICM_20948.h>

// // -------------------- pins --------------------
// #define CS_PIN     5       // nCS for the IMU  (VSPI default)
// #define SCK_PIN    18      // SCL  (VSPI default)
// #define MISO_PIN   19      // AD0 / SDO (VSPI default)
// #define MOSI_PIN   23      // SDA / SDI (VSPI default)

// // -------------------- objects -----------------
// ICM_20948_SPI imu;         // SPI-flavoured driver object

// void setup() {
//   Serial.begin(115200);

//   // Bring up the VSPI bus on the pins you wired
//   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

//   // Loop until the IMU answers
//   while (true) {
//     imu.begin(CS_PIN, SPI);           // CS + which SPI port to use
//     if (imu.status == ICM_20948_Stat_Ok) {
//       Serial.println("ICM-20948 connected over SPI!");
//       break;
//     }
//     Serial.println("IMU not detected – retrying...");
//     delay(500);
//   }
// }

// void loop() {
//   // ----------- read IMU when data ready -----------
//   if (imu.dataReady()) {
//     imu.getAGMT();                          // Accel, Gyro, Mag, Temp
//     Serial.print("Accel  X: "); Serial.print(imu.accX());
//     Serial.print("  Y: ");  Serial.print(imu.accY());
//     Serial.print("  Z: ");  Serial.print(imu.accZ());

//     Serial.print(" | Gyro  X: "); Serial.print(imu.gyrX());
//     Serial.print("  Y: ");        Serial.print(imu.gyrY());
//     Serial.print("  Z: ");        Serial.println(imu.gyrZ());
//   }
// }

#include <Arduino.h>
#include <SPI.h>
#include "ICM_20948.h"

#define CS_PIN 5
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

ICM_20948_SPI imu;

// Struttura per calibrazione
struct {
  float accelOffset[3] = {0};
  float gyroOffset[3] = {0};
} calibration;

void calibrateSensor(int samples = 500) {
  float accelSum[3] = {0};
  float gyroSum[3] = {0};

  Serial.println("Calibrazione in corso... (non muovere il sensore)");
  
  for(int i = 0; i < samples; i++) {
    if(imu.dataReady()) {
      imu.getAGMT();
      
      accelSum[0] += imu.accX();
      accelSum[1] += imu.accY();
      accelSum[2] += imu.accZ() - 2000.0; // Sottrai 1g (1000 mg)
      
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

  Serial.println("Calibrazione completata:");
  Serial.print("Accel Offset X:"); Serial.print(calibration.accelOffset[0]);
  Serial.print(" Y:"); Serial.print(calibration.accelOffset[1]);
  Serial.print(" Z:"); Serial.println(calibration.accelOffset[2]);
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

    // Filtraggio semplice (media mobile)
    static float smoothAcc[3] = {0};
    static float smoothGyr[3] = {0};
    const float filterFactor = 0.2;
    
    for(int i = 0; i < 3; i++) {
      smoothAcc[i] = (1-filterFactor)*smoothAcc[i] + filterFactor*((i==2) ? accZ : (i==1) ? accY : accX);
      smoothGyr[i] = (1-filterFactor)*smoothGyr[i] + filterFactor*((i==2) ? gyrZ : (i==1) ? gyrY : gyrX);
    }

    // Stampa dati corretti
    Serial.print("Accel X:"); Serial.print(smoothAcc[0], 2);
    Serial.print(" Y:"); Serial.print(smoothAcc[1], 2);
    Serial.print(" Z:"); Serial.print(smoothAcc[2], 2);
    
    Serial.print(" | Gyro X:"); Serial.print(smoothGyr[0], 2);
    Serial.print(" Y:"); Serial.print(smoothGyr[1], 2);
    Serial.print(" Z:"); Serial.println(smoothGyr[2], 2);
  }
  delay(50); // Frequenza ~20Hz
}