#include <Wire.h>
#include <Preferences.h> //Bibliothèque pour la mémoire Flash
#include <Arduino_GFX_Library.h>
#include "I2Cdev.h"
#include <DFRobot_INA219.h>

#define INA219_I2C_ADDRESS  (0x40)
DFRobot_INA219_IIC power_monitor(&Wire,INA219_I2C_ADDRESS);
boolean power_monitor_ok = false;

// Display & Graphics
#include <Arduino_GFX_Library.h> /* Use v1.4.6, higher versions may conflict with SPI driver! */
#include "SD_MMC.h"
#include <FS.h>
#include "JpegFunc.h"

// --- PINS MOTEURS ---
const int ENA = 46; const int IN1 = 7; const int IN2 = 18;
const int ENC_L_A = 16; const int ENC_L_B = 17;

const int ENB = 45; const int IN3 = 10; const int IN4 = 11;
const int ENC_R_A = 9;  const int ENC_R_B = 47;

// --- I2C MPU6050 ---
#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 4

// SD card connections.
#define PIN_SD_CMD  2
#define PIN_SD_CLK  42
#define PIN_SD_D0  41

// TFT connctions
#define TFT_CS  15
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_SCLK  14
#define TFT_DC  21

Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC,TFT_CS,TFT_SCLK,TFT_MOSI,TFT_MISO,HSPI,true);

// Graphics defines
#define TFT_BLK  48
#define TFT_RES  -1
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RES, 0 /* rotation */, true /* IPS */);

//Define pictures 
#define MODE_CALIBRATION_START "/MODE_CALIBRATION_START.jpg"
#define MODE_CALIBRATION_MPU "/MODE_CALIBRATION_MPU.jpg"
#define MODE_CALIBRATION_DEADZONE "/MODE_CALIBRATION_DEADZONE.jpg"
#define MODE_CALIBRATION_SPEED_SYNC "/MODE_CALIBRATION_SPEED_SYNC.jpg"
#define MODE_CALIBRATION_SAVE "/MODE_CALIBRATION_SAVE.jpg"
#define MODE_CALIBRATION_END "/MODE_CALIBRATION_END.jpg"
#define OK_K  "/ok-horizontal.jpg"

uint8_t image_id = 0xff; // undefined.

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
const float DEADZONE = 1.5; // Zone morte réduite pour le PID
const float ANGLE_CHUTE = 60.0;
const float ANGLE_REPRISE = 5.0;
bool estTombe = false;

// --- VARIABLES PID ---
// À AJUSTER SELON TON ROBOT !
float Kp = 6;  // Force de réaction immédiate
float Ki = 0;   // Correction de l'erreur dans le temps
float Kd =  0;   // Anticipation de la vitesse de chute
float setpoint = 0.0; // Angle cible (équilibre parfait)
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

// Instance de la memoire Flash
Preferences preferences; 

// Interruptions Encodeurs
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countLeft++; else countLeft--;
}
void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countRight++; else countRight--;
}

// ==========================================
//                Affichage
// ==========================================

// pixel drawing callback
static int jpegDrawCallback(JPEGDRAW *pDraw) {
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

void image_show(char *p_filename) {
  jpegDraw(p_filename,jpegDrawCallback,true,0,0,gfx->width(),gfx->height()); // x, y, w, h
}

void sdcard_init(void) {
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
  if (!SD_MMC.begin("/sdcard", true, true)) {
    Serial.println("SD card not found");
    gfx->println(F("SD card not found"));
  }
}

void display_init(void) {
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, 1);
  gfx->begin();
  gfx->fillScreen(WHITE);
  gfx->setTextSize(1);
  gfx->setTextColor(RED);
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.flush();
  
  // Display & SD card
  display_init();
  sdcard_init();
  
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

  preferences.begin("robot_calib", false);

  if (accZ < -4000) {
    // ROBOT À L'ENVERS : On lance la calibration et on sauvegarde
    runCalibrationRoutine();
  } else {
    // ROBOT A L'ENDROIT : On essaie de lire la memoire
    bool isCalibrated = preferences.getBool("isCalibrated", false); 
    
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
      Serial.println("! ERREUR : AUCUNE CALIBRATION EN MÉMOIRE !");
      Serial.println("Veuillez éteindre le robot, le retourner (roues en l'air) et le rallumer pour calibrer.");
      while(true) { delay(1000); } 
    }
  }
  
  // Initialisation du chronomètre pour le PID
  previous_time = millis();
}

void loop() {
  // --- 1. LECTURE MPU6050 ---
  readMPU();

  // --- 2. CALCUL ANGLE ---
  float accX_corrige = accX - offAccX;
  float accZ_corrige = accZ - (offAccZ - 16384);
  anglePitch = atan2(accX_corrige, accZ_corrige) * 180.0 / PI;

  // --- CALCUL DU TEMPS (dt) POUR LE PID ---
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0; // Conversion en secondes
  if (dt <= 0.0) dt = 0.001; // Sécurité
  previous_time = current_time;

  // --- 3. GESTION ANTI-CHUTE ---
  if (abs(anglePitch) > ANGLE_CHUTE) {
    if (!estTombe) Serial.println("!!! CHUTE DÉTECTÉE - Moteurs coupés !!!");
    estTombe = true;
  }

  if (estTombe && abs(anglePitch) < ANGLE_REPRISE) {
    Serial.println("Robot redressé - Reprise de l'équilibre");
    estTombe = false;
    integral = 0; // Reset du PID
    delay(500); 
    previous_time = millis(); 
  }

  // --- CALCUL PID ---
  float error = setpoint - anglePitch;
  integral += error * dt;
  integral = constrain(integral, -100, 100); // Anti-windup
  
  float derivative = (error - previous_error) / dt;
  previous_error = error;

  float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // --- 4. COMMANDE MOTEURS ---
  if (estTombe) {
    stopMotors();
    integral = 0; 
  } 
  else {
    int plageUtile = 255 - commonMinPWM;
    
    // Vitesse basée sur le PID
    int baseSpeed = abs((int)pid_output);
    baseSpeed = constrain(baseSpeed, 0, plageUtile);

    if (abs(anglePitch) <= DEADZONE) {
      stopMotors();
      integral = 0; 
    }
    else if (pid_output < 0) { 
      moveForward(baseSpeed);
    } 
    else if (pid_output > 0) {
      moveBackward(baseSpeed);
    }
  }

  // --- 5. TÉLÉMÉTRIE ---
  Serial.print("Angle: "); Serial.print(anglePitch);
  Serial.print(" | PID: "); Serial.print(pid_output);
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

  gfx->setTextSize(2); // Texte plus gros pour qu'il soit bien visible
  gfx->setTextColor(WHITE);
  
  // Compte à rebours initial
  for (int i = 10; i >= 0; i--) {
    image_show((char*)MODE_CALIBRATION_START);
    gfx->setRotation(1);
    gfx->setCursor(212, 146);
    gfx->print(i);
    Serial.print(i); Serial.print("s.. ");
    gfx->setRotation(0);
    delay(1000);
  }
  Serial.println("\n");

  // --- ETAPE 1 : MPU6050 ---
  Serial.println("[1/3] Calibration MPU6050...");
  image_show((char*)MODE_CALIBRATION_MPU);
  gfx->setRotation(1);
  gfx->setRotation(0);
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
  image_show((char*)MODE_CALIBRATION_DEADZONE);
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
  image_show((char*)MODE_CALIBRATION_SPEED_SYNC);
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
  image_show((char*)MODE_CALIBRATION_SAVE);
  
  preferences.putFloat("offAccX", offAccX);
  preferences.putFloat("offAccZ", offAccZ);
  preferences.putInt("minPWM", commonMinPWM);
  preferences.putFloat("ratioL", ratioLeft);
  preferences.putFloat("ratioR", ratioRight);
  preferences.putBool("isCalibrated", true); 

  Serial.println(" SAUVEGARDE RÉUSSIE !");


  Serial.println("\n Mode Équilibre dans 3 sec... Lâchez le robot sur le sol !");
  Serial.println("========================================");
  
  // Troisième compte à rebours (Lâcher le robot)  
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE);
  for (int i = 3; i >= 0; i--) {
    image_show((char*)MODE_CALIBRATION_END);
    gfx->setRotation(1);
    gfx->setCursor(212, 146);
    gfx->print(i);
    Serial.print(i); Serial.print("s.. ");
    gfx->setRotation(0);
    delay(1000);
  }

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