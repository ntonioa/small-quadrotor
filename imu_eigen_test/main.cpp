void setup() {
  Serial.begin(115200);
  motors_init();
  
  Serial.println("=== TEST IDENTIFICAZIONE MOTORI ===");
  
  // Test Pin 12 (che tu hai messo come primo nell'array)
  Serial.println("Sto girando il PIN 12... Chi si muove?");
  ledcWrite(0, duty_from_unit(0.15)); // Indice 0 -> Pin 12
  delay(1500);
  ledcWrite(0, 0);
  delay(2000);

  // Test Pin 13
  Serial.println("Sto girando il PIN 13... Chi si muove?");
  ledcWrite(1, duty_from_unit(0.15)); // Indice 1 -> Pin 13
  delay(1500);
  ledcWrite(1, 0);
  delay(2000);

  // Test Pin 14
  Serial.println("Sto girando il PIN 14... Chi si muove?");
  ledcWrite(2, duty_from_unit(0.15)); // Indice 2 -> Pin 14
  delay(1500);
  ledcWrite(2, 0);
  delay(2000);
  
  // Test Pin 15
  Serial.println("Sto girando il PIN 15... Chi si muove?");
  ledcWrite(3, duty_from_unit(0.15)); // Indice 3 -> Pin 15
  delay(1500);
  ledcWrite(3, 0);
  delay(2000);
}

void loop() {}