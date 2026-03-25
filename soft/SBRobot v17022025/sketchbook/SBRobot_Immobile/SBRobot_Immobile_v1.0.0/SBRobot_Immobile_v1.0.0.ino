#include <Wire.h>
#include <Preferences.h> //Bibliothèque pour la mémoire Flash

// --- PINS MOTEURS ---
const int ENA = 46; const int IN1 = 7; const int IN2 = 18;
const int ENC_L_A = 16; const int ENC_L_B = 17;

const int ENB = 45; const int IN3 = 10; const int IN4 = 11;
const int ENC_R_A = 9;  const int ENC_R_B = 47;

// --- I2C MPU6050 ---
#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 4

// --- VARIABLES GLOBALES ---
volatile long countLeft = 0;
volatile long countRight = 0;
int16_t accX, accY, accZ;

// Variables de calibration
float offAccX = 0; 
float offAccZ = 16384; 
int commonMinPWM = 0;
float ratioLeft = 1.0;
float ratioRight = 1.0;

// Variables de navigation et Securite
float anglePitch = 0;
const float DEADZONE = 2.0;
const float ANGLE_CHUTE = 60.0;
const float ANGLE_REPRISE = 5.0;
bool estTombe = false;

// Instance de la memoire Flash
Preferences preferences; 

// Interruptions Encodeurs
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countLeft++; else countLeft--;
}
void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countRight++; else countRight--;
}

void setup() {
  Serial.begin(115200);
  
  // Config Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderRight, CHANGE);

  // Init MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  
  delay(1000);

  // --- GESTION DE LA MEMOIRE FLASH ET DEMARRAGE ---
  readMPU();
  Serial.print("Check Z au demarrage: "); Serial.println(accZ);

  // On ouvre l'espace de stockage nomme "robot_calib" en mode Lecture/Ecriture (false)
  preferences.begin("robot_calib", false);

  if (accZ < -4000) {
    // ROBOT À L'ENVERS : On lance la calibration et on sauvegarde
    runCalibrationRoutine();
  } else {
    // ROBOT A L'ENDROIT : On essaie de lire la memoire
    bool isCalibrated = preferences.getBool("isCalibrated", false); // false par défaut
    
    if (isCalibrated) {
      Serial.println("--- CHARGEMENT DES DONNÉES DEPUIS LA FLASH ---");
      offAccX = preferences.getFloat("offAccX", 0);
      offAccZ = preferences.getFloat("offAccZ", 16384);
      commonMinPWM = preferences.getInt("minPWM", 0);
      ratioLeft = preferences.getFloat("ratioL", 1.0);
      ratioRight = preferences.getFloat("ratioR", 1.0);
      
      Serial.print("Offset X : "); Serial.println(offAccX);
      Serial.print("Zone Morte : "); Serial.println(commonMinPWM);
      Serial.print("Ratio Gauche : "); Serial.println(ratioLeft);
      Serial.print("Ratio Droit : "); Serial.println(ratioRight);
      Serial.println("--> Robot prêt pour l'équilibre !");
      delay(2000);
    } else {
      // Aucune donnée trouvee en memoire !
      Serial.println("! ERREUR : AUCUNE CALIBRATION EN MÉMOIRE !");
      Serial.println("Veuillez éteindre le robot, le retourner (roues en l'air) et le rallumer pour calibrer.");
      while(true) { delay(1000); } // On bloque le programme ici par securite
    }
  }
}

void loop() {
  // --- 1. LECTURE MPU6050 ---
  readMPU();

  // --- 2. CALCUL ANGLE ---
  float accX_corrige = accX - offAccX;
  float accZ_corrige = accZ - (offAccZ - 16384);

  anglePitch = atan2(accX_corrige, accZ_corrige) * 180.0 / PI;

  // --- 3. GESTION ANTI-CHUTE ---
  if (abs(anglePitch) > ANGLE_CHUTE) {
    if (!estTombe) Serial.println("!!! CHUTE DÉTECTÉE - Moteurs coupés !!!");
    estTombe = true;
  }

  if (estTombe && abs(anglePitch) < ANGLE_REPRISE) {
    Serial.println("Robot redressé - Reprise de l'équilibre");
    estTombe = false;
    delay(500); 
  }

  // --- 4. COMMANDE MOTEURS ---
  if (estTombe) {
    stopMotors();
  } 
  else {
    int plageUtile = 255 - commonMinPWM;
    int baseSpeed = map(abs((int)anglePitch), 0, (int)ANGLE_CHUTE, 0, plageUtile); 
    baseSpeed = constrain(baseSpeed, 0, plageUtile);

    if (anglePitch > DEADZONE) {
      moveForward(baseSpeed);
    } 
    else if (anglePitch < -DEADZONE) {
      moveBackward(baseSpeed);
    } 
    else {
      stopMotors();
    }
  }

  // --- 5. TÉLÉMÉTRIE ---
  Serial.print("Angle: "); Serial.print(anglePitch);
  Serial.print(" | Etat: "); Serial.println(estTombe ? "COUCHÉ" : "DEBOUT");

  delay(20); 
}

// ==========================================
//      FONCTION DE CALIBRATION AUTO
// ==========================================

void runCalibrationRoutine() {
  Serial.println("\n========================================");
  Serial.println("   MODE CALIBRATION DETECTÉ (Robot inversé)");
  Serial.println("========================================");
  Serial.println("Remettez le robot à l'endroit, ROUES EN L'AIR (sur un socle).");
  
  for (int i = 10; i > 0; i--) {
    Serial.print(i); Serial.print("s.. ");
    delay(1000);
  }
  Serial.println("\n");

  // --- ETAPE 1 : MPU6050 ---
  Serial.println("[1/3] Calibration MPU6050...");
  long sumAx=0, sumAz=0;
  for (int i = 0; i < 2000; i++) {
    readMPU();
    sumAx += accX; sumAz += accZ;
    delay(2);
  }
  offAccX = sumAx / 2000.0;
  offAccZ = sumAz / 2000.0; 

  // --- ETAPE 2 : ZONE MORTE ---
  Serial.println("\n[2/3] Calibration Zone Morte...");
  int minL = 0, minR = 0;
  countLeft = 0;
  for (int i = 0; i < 255; i++) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, i);
    delay(50); 
    if (abs(countLeft) > 5) { minL = i; analogWrite(ENA, 0); break; }
  }
  delay(1000);
  countRight = 0;
  for (int i = 0; i < 255; i++) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, i);
    delay(50);
    if (abs(countRight) > 5) { minR = i; analogWrite(ENB, 0); break; }
  }
  commonMinPWM = max(minL, minR);

  // --- ETAPE 3 : SYNCHRONISATION VITESSE ---
  Serial.println("\n[3/3] Calibration Vitesses...");
  countLeft = 0; countRight = 0;
  int testSpeed = 200;
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, testSpeed);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, testSpeed);
  
  delay(1000); 
  stopMotors();
  delay(500); 
  
  long ticksL = abs(countLeft);
  long ticksR = abs(countRight);
  
  if (ticksL > ticksR) {
    ratioLeft = (float)ticksR / (float)ticksL; 
    ratioRight = 1.0; 
  } else if (ticksR > ticksL) {
    ratioRight = (float)ticksL / (float)ticksR; 
    ratioLeft = 1.0; 
  }

  // --- SAUVEGARDE EN MÉMOIRE FLASH ---
  Serial.println("\n========================================");
  Serial.println(" SAUVEGARDE DANS LA MÉMOIRE FLASH...");
  
  preferences.putFloat("offAccX", offAccX);
  preferences.putFloat("offAccZ", offAccZ);
  preferences.putInt("minPWM", commonMinPWM);
  preferences.putFloat("ratioL", ratioLeft);
  preferences.putFloat("ratioR", ratioRight);
  preferences.putBool("isCalibrated", true); // On valide qu'une calibration a été faite

  Serial.println(" SAUVEGARDE RÉUSSIE !");
  Serial.println(" Mode Équilibre dans 3 sec... Lâchez le robot sur le sol !");
  Serial.println("========================================");
  
  delay(3000);
}

// ==========================================
//      FONCTIONS UTILITAIRES & MOTEURS
// ==========================================

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  accX = (Wire.read() << 8 | Wire.read());
  accY = (Wire.read() << 8 | Wire.read());
  accZ = (Wire.read() << 8 | Wire.read());
}

void moveForward(int baseSpeed) {
  int finalL = (baseSpeed * ratioLeft) + commonMinPWM;
  int finalR = (baseSpeed * ratioRight) + commonMinPWM;
  
  finalL = constrain(finalL, 0, 255);
  finalR = constrain(finalR, 0, 255);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, finalL);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, finalR);
}

void moveBackward(int baseSpeed) {
  int finalL = (baseSpeed * ratioLeft) + commonMinPWM;
  int finalR = (baseSpeed * ratioRight) + commonMinPWM;
  
  finalL = constrain(finalL, 0, 255);
  finalR = constrain(finalR, 0, 255);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, finalL);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, finalR);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}