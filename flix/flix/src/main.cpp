#include <Arduino.h>

#define PWM_FREQUENCY 78000
#define PWM_RESOLUTION 10
#define PWM_MIN 0
#define PWM_MAX (1000000 / PWM_FREQUENCY)
#define PWM_STOP 0

const int motorPins[4] = {12, 13, 14, 15};
const int motorChannels[4] = {0, 1, 2, 3};

int getDutyCycle(float value);

// Duty target in percentuale
const float dutyVal = 0.8f;  // 80%
const unsigned long interval = 3000;  // 3 secondi

bool motorsOn = true;
unsigned long lastToggleTime = 0;

// Calcola il valore PWM corrispondente a un duty normalizzato [0,1]
int getDutyCycle(float value) {
  value = constrain(value, 0.0f, 1.0f);
  float pwm = value * (PWM_MAX - PWM_MIN) + PWM_MIN;
  if (value == 0.0f) pwm = PWM_STOP;
  float duty = pwm * ((1 << PWM_RESOLUTION) - 1) / (1000000.0f / PWM_FREQUENCY);
  return round(duty);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Inizializzazione motori per yaw test ciclico...");

  // Configura PWM per tutti i motori
  for (int i = 0; i < 4; i++) {
    ledcSetup(motorChannels[i], PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(motorPins[i], motorChannels[i]);
    ledcWrite(motorChannels[i], getDutyCycle(0.0f));  // inizialmente spenti
  }

  // Accendi motori iniziali (12 e 14)
  ledcWrite(motorChannels[0], getDutyCycle(dutyVal));  // Pin 12
  ledcWrite(motorChannels[2], getDutyCycle(dutyVal));  // Pin 14
  lastToggleTime = millis();
  motorsOn = true;

  Serial.println("Motori 12 e 14 accesi all'80%");
}

void loop() {
  unsigned long now = millis();
  if (now - lastToggleTime >= interval) {
    lastToggleTime = now;
    motorsOn = !motorsOn;

    if (motorsOn) {
      ledcWrite(motorChannels[0], getDutyCycle(dutyVal));  // Pin 12
      ledcWrite(motorChannels[2], getDutyCycle(dutyVal));  // Pin 14
      Serial.println("Motori ACCESI all'80%");
    } else {
      ledcWrite(motorChannels[0], getDutyCycle(0.0f));
      ledcWrite(motorChannels[2], getDutyCycle(0.0f));
      Serial.println("Motori SPENTI");
    }
  }
}
