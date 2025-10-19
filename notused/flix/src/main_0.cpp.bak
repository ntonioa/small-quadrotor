// #include <Arduino.h>
// #include <SPI.h>
// #include <ICM_20948.h>

// // -------------------- pins --------------------
// #define MOTOR_PIN  13      // still on GPIO 13
// #define CS_PIN     5       // nCS for the IMU  (VSPI default)
// #define SCK_PIN    18      // SCL  (VSPI default)
// #define MISO_PIN   19      // AD0 / SDO (VSPI default)
// #define MOSI_PIN   23      // SDA / SDI (VSPI default)

// // -------------------- objects -----------------
// ICM_20948_SPI imu;         // SPI-flavoured driver object

// // -------------------- motor variables ---------
// int pwmValue = 0;
// int pwmStep = 10;
// unsigned long lastMotorUpdate = 0;
// const unsigned long motorUpdateInterval = 500; // 500ms

// void setup() {
//   Serial.begin(115200);

//   // Motor output with PWM
//   pinMode(MOTOR_PIN, OUTPUT);
//   ledcSetup(0, 1000, 8); // Channel 0, 1kHz frequency, 8-bit resolution
//   ledcAttachPin(MOTOR_PIN, 0);

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

//   Serial.println("Motor PWM test starting...");
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

//   // ---------------- motor PWM demo --------------------
//   if (millis() - lastMotorUpdate >= motorUpdateInterval) {
//     ledcWrite(0, pwmValue);
//     Serial.print("Motor PWM: ");
//     Serial.print(pwmValue);
//     Serial.print(" (");
//     Serial.print((pwmValue * 100) / 255);
//     Serial.println("%)");
    
//     pwmValue += pwmStep;
//     if (pwmValue > 255) {
//       pwmValue = 0;
//     }
    
//     lastMotorUpdate = millis();
//   }
// }


#include <Arduino.h>  // << essenziale

#define PWM_PIN 15
#define PWM_FREQUENCY 78000
#define PWM_RESOLUTION 10
#define PWM_STOP 0
#define PWM_MIN 0
#define PWM_MAX (1000000 / PWM_FREQUENCY)

// Function prototype for getDutyCycle
int getDutyCycle(float value);

void setup() {
  Serial.begin(115200);
  Serial.println("Inserisci valore (0.0 - 1.0) per duty cycle:");
  
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); // canale 0
  ledcAttachPin(PWM_PIN, 0); // collega il pin al canale
  ledcWrite(0, getDutyCycle(0)); // inizializza con 0%
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float val = input.toFloat();
    val = constrain(val, 0.0f, 1.0f);  // Usa 'f' per evitare ambiguità

    int duty = getDutyCycle(val);
    ledcWrite(0, duty);

    Serial.print("Duty cycle aggiornato: ");
    Serial.print(val * 100);
    Serial.println("%");
  }
}

int getDutyCycle(float value) {
  value = constrain(value, 0.0f, 1.0f);
  float pwm = value * (PWM_MAX - PWM_MIN) + PWM_MIN;
  if (value == 0) pwm = PWM_STOP;
  float duty = pwm * ((1 << PWM_RESOLUTION) - 1) / (1000000.0f / PWM_FREQUENCY);
  return round(duty);
}