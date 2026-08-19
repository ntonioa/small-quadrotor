#include <Arduino.h>
#include <Eigen/Dense>
#include <Eigen/Core>

// ==========================================
// LA TUA CONFIGURAZIONE HARDWARE (NON LA TOCCO)
// ==========================================
// m0 (Pin 12): Anteriore-Sinistro (FL)
// m1 (Pin 13): Posteriore-Sinistro (RL)
// m2 (Pin 14): Posteriore-Destro (RR)
// m3 (Pin 15): Anteriore-Destro (FR)
static const int MOTOR_PINS[4] = {12, 13, 14, 15};
static const int MOTOR_CHANNELS[4] = {0, 1, 2, 3};

// ==========================================
// IL TUO MIXER (LA MATRICE MATEMATICA)
// ==========================================
using Vec4 = Eigen::Matrix<float, 4, 1>;
using Mat44 = Eigen::Matrix<float, 4, 4>;

struct Mixer {
  float l = 0.062f;
  float km = 0.0041f;
  Mat44 A; 

  Mixer() {
    const float L = l / 1.41421356f;
    const float c_l = 1.0f / (4.0f * L);
    const float c_k = 1.0f / (4.0f * km);
    const float c_f = 0.25f;

    // LA MATRICE CHE C'ERA NEL TUO CODICE ORIGINALE
    A << 
       +c_l, +c_l, +c_k, c_f,  // F0 (FL - Pin 12)
       +c_l, -c_l, -c_k, c_f,  // F1 (RL - Pin 13)
       -c_l, -c_l, +c_k, c_f,  // F2 (RR - Pin 14)
       -c_l, +c_l, -c_k, c_f;  // F3 (FR - Pin 15)
  }
  Vec4 mix(const Vec4 &y) const { return A * y; }
} g_mixer;

void setup() {
  Serial.begin(115200);
  
  // Setup Motori
  for (int i = 0; i < 4; ++i) {
    ledcSetup(MOTOR_CHANNELS[i], 30000, 10);
    ledcAttachPin(MOTOR_PINS[i], MOTOR_CHANNELS[i]);
    ledcWrite(MOTOR_CHANNELS[i], 0);
  }

  Serial.println("\n\n=== TEST DIAGNOSTICA MIXER ===");
  Serial.println("Simulo un comando di ROLL A DESTRA (Ala destra giù).");
  Serial.println("I motori di SINISTRA (Pin 12 e 13) dovrebbero spingere di più.");
  Serial.println("I motori di DESTRA (Pin 14 e 15) dovrebbero fermarsi.");
  delay(2000);
}

void loop() {
  // 1. CREIAMO UN COMANDO FINTO: ROLL MASSIMO A DESTRA
  // y = [Torque_Roll, Torque_Pitch, Torque_Yaw, Thrust]
  // Roll positivo = 0.05 Nm (vogliamo girare a destra)
  // Thrust = 1.5 N (giusto per accendere i motori)
  Eigen::Vector4f y_fake;
  y_fake << 0.05f, 0.0f, 0.0f, 1.5f; 

  // 2. CALCOLIAMO COSA ESCE DALLA MATRICE
  Eigen::Vector4f f = g_mixer.mix(y_fake);

  // 3. STAMPIAMO I RISULTATI
  Serial.println("------------------------------------------------");
  Serial.print("INPUT MATRICE:  Roll=0.05, Pitch=0, Thrust=1.5\n");
  Serial.println("OUTPUT AI PIN (Teorico):");
  
  // FL (Pin 12)
  Serial.printf("PIN 12 (FL - Ant.SX):  %.3f N  -> %s\n", f(0), f(0) > 1.5 ? "ACCELERA (GIUSTO)" : "RALLENTA (ERRORE)");
  
  // RL (Pin 13)
  Serial.printf("PIN 13 (RL - Post.SX): %.3f N  -> %s\n", f(1), f(1) > 1.5 ? "ACCELERA (GIUSTO)" : "RALLENTA (ERRORE)");

  // RR (Pin 14)
  Serial.printf("PIN 14 (RR - Post.DX): %.3f N  -> %s\n", f(2), f(2) < 1.5 ? "RALLENTA (GIUSTO)" : "ACCELERA (ERRORE)");

  // FR (Pin 15)
  Serial.printf("PIN 15 (FR - Ant.DX):  %.3f N  -> %s\n", f(3), f(3) < 1.5 ? "RALLENTA (GIUSTO)" : "ACCELERA (ERRORE)");

  // 4. FACCIAMO GIRARE I MOTORI DAVVERO PER VEDERE CON GLI OCCHI
  // Scaliamo per PWM (molto grezzo, solo per vedere chi gira)
  for(int i=0; i<4; i++) {
    int pwm = (int)(f(i) * 50); // Moltiplicatore a caso per vederli girare piano
    if(pwm < 0) pwm = 0;
    if(pwm > 200) pwm = 200;
    ledcWrite(MOTOR_CHANNELS[i], pwm);
  }

  delay(1000);
}