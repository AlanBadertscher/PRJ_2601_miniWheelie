#include <Wire.h>
#include <Preferences.h> //Sauvegarde memoire interne
#include <Arduino_GFX_Library.h>
#include "I2Cdev.h"
#include <DFRobot_INA219.h>

#define INA219_I2C_ADDRESS  (0x40)
DFRobot_INA219_IIC power_monitor(&Wire,INA219_I2C_ADDRESS);
boolean power_monitor_ok = false;

//Ecran et carte SD
#include <Arduino_GFX_Library.h> //Version librairie specifique requise
#include "SD_MMC.h"
#include <FS.h>
#include "JpegFunc.h"

//Broches moteur gauche
const int ENA = 46; const int IN1 = 7; const int IN2 = 18;
const int ENC_L_A = 16; const int ENC_L_B = 17;

//Broches moteur droit
const int ENB = 45; const int IN3 = 10; const int IN4 = 11;
const int ENC_R_A = 9;  const int ENC_R_B = 47;

//Broches capteur inclinaison
#define MPU_ADDR 0x68
#define SDA_PIN 5
#define SCL_PIN 4

//Broches carte SD
#define PIN_SD_CMD  2
#define PIN_SD_CLK  42
#define PIN_SD_D0  41

//Broches ecran
#define TFT_CS  15
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_SCLK  14
#define TFT_DC  21

Arduino_ESP32SPI *bus = new Arduino_ESP32SPI(TFT_DC,TFT_CS,TFT_SCLK,TFT_MOSI,TFT_MISO,HSPI,true);

//Parametres ecran
#define TFT_BLK  48
#define TFT_RES  -1
Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RES, 0, true);

//Fichiers images
#define MODE_CALIBRATION_START "/MODE_CALIBRATION_START.jpg"
#define MODE_CALIBRATION_MPU "/MODE_CALIBRATION_MPU.jpg"
#define MODE_CALIBRATION_DEADZONE "/MODE_CALIBRATION_DEADZONE.jpg"
#define MODE_CALIBRATION_SPEED_SYNC "/MODE_CALIBRATION_SPEED_SYNC.jpg"
#define MODE_CALIBRATION_SAVE "/MODE_CALIBRATION_SAVE.jpg"
#define MODE_CALIBRATION_END "/MODE_CALIBRATION_END.jpg"
#define OK_K  "/ok-horizontal.jpg"

uint8_t image_id = 0xff;

//Valeurs globales capteurs
volatile long countLeft = 0;
volatile long countRight = 0;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

//Valeurs calibration
float offAccX = 0; 
float offAccZ = 16384; 
float offGyroY = 0;
int commonMinPWM = 0;
float ratioLeft = 1.0;
float ratioRight = 1.0;

//Securite et direction
float anglePitch = 0;
const float DEADZONE = 0; //Marge arret
const float ANGLE_CHUTE = 60.0;
const float ANGLE_REPRISE = 5;
bool estTombe = false;

//Reglage equilibre PID
float Kp = 15;  //Force reaction
float Ki = 0.3;   //Correction lente
float Kd =  0.5;   //Amortisseur
float setpoint = 0; //Point equilibre parfait
float integral = 0;
float previous_error = 0;
unsigned long previous_time = 0;

//Outil de sauvegarde
Preferences preferences; 

//Lecture compteurs moteurs
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countLeft++; else countLeft--;
}
void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countRight++; else countRight--;
}

//==========================================
//Affichage
//==========================================

//Dessin pixels image
static int jpegDrawCallback(JPEGDRAW *pDraw) {
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

void image_show(char *p_filename) {
  jpegDraw(p_filename,jpegDrawCallback,true,0,0,gfx->width(),gfx->height());
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
  //Demarrage communication pc
  Serial.begin(115200);
  Serial.flush();
  
  //Demarrage modules
  display_init();
  sdcard_init();
  
  //Configuration broches
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderRight, CHANGE);

  //Demarrage capteur inclinaison
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  
  delay(1000);

  //Gestion sauvegarde memoire
  readMPU();
  Serial.print("Check Z au demarrage: "); Serial.println(accZ);

  preferences.begin("robot_calib", false);

  if (accZ < -4000) {
    //Robot retourne on calibre
    runCalibrationRoutine();
  } else {
    //Robot debout on lit donnees
    bool isCalibrated = preferences.getBool("isCalibrated", false); 
    
    if (isCalibrated) {
      Serial.println("--- CHARGEMENT DES DONNEES DEPUIS LA FLASH ---");
      offAccX = preferences.getFloat("offAccX", 0);
      offAccZ = preferences.getFloat("offAccZ", 16384);
      offGyroY = preferences.getFloat("offGyroY", 0);
      commonMinPWM = preferences.getInt("minPWM", 0);
      if (commonMinPWM < 0) commonMinPWM = 0; //Securite pwm negatif
      ratioLeft = preferences.getFloat("ratioL", 1.0);
      ratioRight = preferences.getFloat("ratioR", 1.0);
      
      Serial.print("Offset X : "); Serial.println(offAccX);
      Serial.print("Zone Morte : "); Serial.println(commonMinPWM);
      Serial.print("Ratio Gauche : "); Serial.println(ratioLeft);
      Serial.print("Ratio Droit : "); Serial.println(ratioRight);
      Serial.println("--> Robot pret pour l equilibre !");
      delay(2000);
    } else {
      Serial.println("! ERREUR : AUCUNE CALIBRATION EN MEMOIRE !");
      Serial.println("Veuillez eteindre le robot, le retourner (roues en l air) et le rallumer pour calibrer.");
      while(true) { delay(1000); } 
    }
  }
  
  //Depart chrono PID
  previous_time = millis();
}

void loop() {
  //Chrono de boucle precise
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0; //Duree en secondes
  
//Attente si boucle trop rapide (10ms = 100Hz)
  if (dt < 0.01) return;

  previous_time = current_time;

  //Lecture donnees
  readMPU();

  //Calcul angle corrige
  float accX_corrige = accX - offAccX;
  float accZ_corrige = accZ - offAccZ;
  
  //Angle brut capteur
  float accPitch = atan2(accX_corrige, accZ_corrige) * 180.0 / PI;
  
  //Vitesse rotation corrige
  float gyroRate = -(gyroY - offGyroY) / 131.0;
  
  anglePitch = 0.98 * (anglePitch + gyroRate * dt) + 0.10 * accPitch;

  //Verification chute
  if (abs(anglePitch) > ANGLE_CHUTE) {
    if (!estTombe) Serial.println("!!! CHUTE DETECTEE - Moteurs coupes !!!");
    estTombe = true;
  }

  if (estTombe && abs(anglePitch) < ANGLE_REPRISE) {
    Serial.println("Robot redresse - Reprise equilibre");
    estTombe = false;
    integral = 0; //Remise a zero memoire
    delay(500); 
    previous_time = millis(); 
  }

  //Calcul correction PID
  float error = setpoint - anglePitch;
  integral += error * dt;
  integral = constrain(integral, -100, 100); //Securite depassement
  
  float derivative = (error - previous_error) / dt;
  previous_error = error;

  float pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  //Application moteurs
  if (estTombe) {
    stopMotors();
    integral = 0; 
  } 
  else {
    int plageUtile = 255 - commonMinPWM;
    
    //Puissance calculee
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

  //Envoi informations pc
  static unsigned long last_print = 0;
  if (millis() - last_print > 250) {
    Serial.print("Angle: "); Serial.print(anglePitch);
    Serial.print(" | PID: "); Serial.print(pid_output);
    Serial.print(" | Etat: "); Serial.println(estTombe ? "COUCHE" : "DEBOUT");
    last_print = millis();
  }

}

//==========================================
//Fonction calibration auto
//==========================================

void runCalibrationRoutine() {
  Serial.println("\n========================================");
  Serial.println("   MODE CALIBRATION DETECTE (Robot inverse)");
  Serial.println("========================================");
  Serial.println("Remettez le robot a l endroit, ROUES EN L AIR (sur un socle).");

  gfx->setTextSize(2); //Agrandir texte
  gfx->setTextColor(WHITE);
  
  //Premier compte a rebours
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

  //Etape1 capteur
  Serial.println("[1/3] Calibration MPU6050...");
  image_show((char*)MODE_CALIBRATION_MPU);
  long sumAx=0, sumAz=0, sumGy=0; //Ajout valeurs gyro
  for (int i = 0; i < 2000; i++) {
    readMPU();
    sumAx += accX; sumAz += accZ; sumGy += gyroY;
    delay(2);
  }
  offAccX = sumAx / 2000.0;
  offAccZ = (sumAz / 2000.0) - 16384.0;
  offGyroY = sumGy / 2000.0; //Sauvegarde decalage

  //Etape2 puissance minimum
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

  Serial.print("-> PWM Min Moteur Gauche : "); Serial.println(minL);
  Serial.print("-> PWM Min Moteur Droit  : "); Serial.println(minR);
  Serial.print("-> PWM Min Commun (retenu): "); Serial.println(commonMinPWM);

  //Etape3 roues synchronisees
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

  //Sauvegarde finale memoire
  Serial.println("\n========================================");
  Serial.println(" SAUVEGARDE DANS LA MEMOIRE FLASH...");
  image_show((char*)MODE_CALIBRATION_SAVE);
  
  preferences.putFloat("offAccX", offAccX);
  preferences.putFloat("offAccZ", offAccZ);
  preferences.putFloat("offGyroY", offGyroY);
  preferences.putInt("minPWM", commonMinPWM);
  preferences.putFloat("ratioL", ratioLeft);
  preferences.putFloat("ratioR", ratioRight);
  preferences.putBool("isCalibrated", true); 

  Serial.println(" SAUVEGARDE REUSSIE !");


  Serial.println("\n Mode Equilibre dans 3 sec... Lachez le robot sur le sol !");
  Serial.println("========================================");
  
  //Dernier compte a rebours
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

//==========================================
//Fonctions mouvements
//==========================================

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //Debut lecture
  Wire.endTransmission(false);
  
  //Lecture donnees completes
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);
  
  accX = (Wire.read() << 8 | Wire.read());
  accY = (Wire.read() << 8 | Wire.read());
  accZ = (Wire.read() << 8 | Wire.read());
  
  //Saut temperature
  Wire.read(); Wire.read(); 
  
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());
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