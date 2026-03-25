/*
 * TEST : Équilibre Basique (X/Z) - SENS INVERSÉ
 * Matériel: ESP32-S3 Self Balanced Robot
 */
#include <Wire.h>

// --- DÉFINITION DES PINS ---
// Moteur GAUCHE
const int ENA = 46; const int IN1 = 7; const int IN2 = 18;
const int ENC_L_A = 16; const int ENC_L_B = 17;

// Moteur DROIT
const int ENB = 45; const int IN3 = 10; const int IN4 = 11;
const int ENC_R_A = 9;  const int ENC_R_B = 47;

// I2C MPU6050
#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 4

// Variables
int16_t accX, accY, accZ;
float anglePitch = 0;

// Zone morte (angle minimal pour activer les moteurs)
const float DEADZONE = 2.0; 

void setup() {
  Serial.begin(115200);
  
  // Config Moteurs
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Init MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  Serial.println("--- TEST SENS INVERSÉ DÉMARRÉ ---");
  delay(1000);
}

void loop() {
  // --- 1. LECTURE CAPTEURS ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  accX = (Wire.read() << 8 | Wire.read());
  accY = (Wire.read() << 8 | Wire.read());
  accZ = (Wire.read() << 8 | Wire.read());

  // --- 2. CALCUL ANGLE (Axe X et Z) ---
  anglePitch = atan2(accX, accZ) * 180.0 / PI;

  // --- 3. COMMANDE MOTEURS ---
  int pwmSpeed = 0;
  
  // Calcul de la vitesse (map et constrain)
  int v = map(abs((int)anglePitch), 0, 45, 60, 255); 
  v = constrain(v, 0, 255); 

  if (anglePitch > DEADZONE) {
    // Si Angle Positif -> On appelle la fonction AVANCER (modifiée ci-dessous)
    moveForward(v);
  } 
  else if (anglePitch < -DEADZONE) {
    // Si Angle Négatif -> On appelle la fonction RECULER (modifiée ci-dessous)
    moveBackward(v);
  } 
  else {
    stopMotors();
  }

  // --- 4. AFFICHAGE ---
  Serial.print("AccX: "); Serial.print(accX);
  Serial.print(" | AccZ: "); Serial.print(accZ);
  Serial.print(" || Angle: "); Serial.println(anglePitch);

  delay(50);
}

// --- FONCTIONS MOTEURS INVERSÉES ---

void moveForward(int speed) {
  // J'ai inversé HIGH et LOW ici par rapport au code précédent
  
  // Moteur Gauche 
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH); 
  analogWrite(ENA, speed);
  
  // Moteur Droit
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW); 
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  // J'ai inversé HIGH et LOW ici aussi
  
  // Moteur Gauche
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  analogWrite(ENA, speed);
  
  // Moteur Droit
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
}