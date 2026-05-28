#include <Wire.h>
#include <Preferences.h> //Sauvegarde memoire interne
#include <Arduino_GFX_Library.h>
#include "I2Cdev.h"

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

//==========================================
// INTERRUPTIONS & CALLBACKS
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : readEncoderLeft  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Interruption incrémentant/décrémentant le compteur du moteur gauche
//-- démonstration : N/A
//-- aide - référence - lien : Encodeur en quadrature
//----------------------------------------------------------------------------------//
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) countLeft++; else countLeft--;
}

//----------------------------------------------------------------------------------//
//-- nom fct : readEncoderRight  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Interruption incrémentant/décrémentant le compteur du moteur droit
//-- démonstration : N/A
//-- aide - référence - lien : Encodeur en quadrature
//----------------------------------------------------------------------------------//
void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) countRight++; else countRight--;
}

//----------------------------------------------------------------------------------//
//-- nom fct : jpegDrawCallback  
//-- paramètre entrée : JPEGDRAW* - pDraw 
//-- paramètre sortie : int - status de réussite
//-- paramètre référence (IN-OUT) : aucun
//-- description : Callback utilisé par la librairie JPEG pour dessiner les pixels à l'écran
//-- démonstration : N/A
//-- aide - référence - lien : Arduino_GFX_Library / JPEGDEC
//----------------------------------------------------------------------------------//
static int jpegDrawCallback(JPEGDRAW *pDraw) {
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

//==========================================
// AFFICHAGE & SD
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : image_show  
//-- paramètre entrée : char* - p_filename (chemin de l'image)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Lance le décodage et l'affichage d'une image JPEG depuis la carte SD
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void image_show(char *p_filename) {
  jpegDraw(p_filename, jpegDrawCallback, true, 0, 0, gfx->width(), gfx->height());
}

//----------------------------------------------------------------------------------//
//-- nom fct : sdcard_init  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Initialise la communication SD_MMC sur les broches définies
//-- démonstration : N/A
//-- aide - référence - lien : ESP32 SD_MMC Library
//----------------------------------------------------------------------------------//
void sdcard_init(void) {
  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
  if (!SD_MMC.begin("/sdcard", true, true)) {
    Serial.println("SD card not found");
    gfx->println(F("SD card not found"));
  }
}

//----------------------------------------------------------------------------------//
//-- nom fct : display_init  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Configure et allume l'écran TFT
//-- démonstration : N/A
//-- aide - référence - lien : Arduino_GFX_Library
//----------------------------------------------------------------------------------//
void display_init(void) {
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, 1);
  gfx->begin();
  gfx->fillScreen(WHITE);
  gfx->setTextSize(1);
  gfx->setTextColor(RED);
}

//==========================================
// INITIALISATIONS MATÉRIELLES
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : initHardwarePins  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Configure l'état des I/O des moteurs et attache les interruptions
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void initHardwarePins() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC_L_A, INPUT); pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT); pinMode(ENC_R_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderRight, CHANGE);
}

//----------------------------------------------------------------------------------//
//-- nom fct : initMPU6050  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Démarre le bus I2C et réveille le capteur MPU6050
//-- démonstration : N/A
//-- aide - référence - lien : Registres MPU6050
//----------------------------------------------------------------------------------//
void initMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registre Power Management 1
  Wire.write(0);    // Reveil
  Wire.endTransmission(true);
  delay(1000);
}

//----------------------------------------------------------------------------------//
//-- nom fct : loadCalibrationOrRun  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Lit l'axe Z au démarrage. S'il est retourné (< -4000), lance la calibration, sinon charge la flash
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void loadCalibrationOrRun() {
  readMPU();
  Serial.print("Check Z au demarrage: "); Serial.println(accZ);
  preferences.begin("robot_calib", false);

  if (accZ < -4000) {
    runCalibrationRoutine();
  } else {
    bool isCalibrated = preferences.getBool("isCalibrated", false); 
    if (isCalibrated) {
      Serial.println("--- CHARGEMENT DES DONNEES DEPUIS LA FLASH ---");
      offAccX = preferences.getFloat("offAccX", 0);
      offAccZ = preferences.getFloat("offAccZ", 16384);
      offGyroY = preferences.getFloat("offGyroY", 0);
      commonMinPWM = preferences.getInt("minPWM", 0);
      if (commonMinPWM < 0) commonMinPWM = 0;
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
}

//==========================================
// LECTURE CAPTEURS & LOGIQUE DE CONTROLE
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : readMPU  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Lit les 6 axes (Accel/Gyro) en I2C depuis le MPU6050
//-- démonstration : N/A
//-- aide - référence - lien : I2C MPU6050 data reading
//----------------------------------------------------------------------------------//
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //Debut lecture
  Wire.endTransmission(false);
  
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);
  
  accX = (Wire.read() << 8 | Wire.read());
  accY = (Wire.read() << 8 | Wire.read());
  accZ = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); //Saut temperature
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());
}

//----------------------------------------------------------------------------------//
//-- nom fct : calculateRobotAngle  
//-- paramètre entrée : float - dt (temps écoulé depuis la dernière boucle)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Calcule l'angle exact du robot en fusionnant l'accéléromètre et le gyroscope (Filtre Complémentaire)
//-- démonstration : anglePitch = 0.98*(angle+gyro*dt) + 0.02*(accPitch)
//-- aide - référence - lien : Filtre complémentaire
//----------------------------------------------------------------------------------//
void calculateRobotAngle(float dt) {
  float accX_corrige = accX - offAccX;
  float accZ_corrige = accZ - offAccZ;
  
  float accPitch = atan2(accX_corrige, accZ_corrige) * 180.0 / PI;
  float gyroRate = -(gyroY - offGyroY) / 131.0;
  
  anglePitch = 0.98 * (anglePitch + gyroRate * dt) + 0.10 * accPitch;
}

//----------------------------------------------------------------------------------//
//-- nom fct : checkFallStatus  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Gère la sécurité en cas de chute et la reprise lorsque le robot est redressé
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void checkFallStatus() {
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
}

//----------------------------------------------------------------------------------//
//-- nom fct : computePID  
//-- paramètre entrée : float - dt (temps de cycle)
//-- paramètre sortie : float - valeur de correction PWM (pid_output)
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Applique la régulation PID pour stabiliser l'angle du robot
//-- démonstration : PID = (Kp * error) + (Ki * integral) + (Kd * derivative)
//-- aide - référence - lien : Régulation PID pour robot pendule inversé
//----------------------------------------------------------------------------------//
float computePID(float dt) {
  float error = setpoint - anglePitch;
  integral += error * dt;
  integral = constrain(integral, -100, 100); //Securite depassement
  
  float derivative = (error - previous_error) / dt;
  previous_error = error;

  return (Kp * error) + (Ki * integral) + (Kd * derivative);
}

//==========================================
// CONTROLE MOTEURS
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : moveForward  
//-- paramètre entrée : int - baseSpeed (Vitesse brute issue du PID)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Applique une commande de marche avant avec prise en compte du ratio et de la zone morte
//-- démonstration : final = (baseSpeed * ratio) + zoneMorte
//-- aide - référence - lien : Contrôle Pont en H PWM
//----------------------------------------------------------------------------------//
void moveForward(int baseSpeed) {
  int finalL = (baseSpeed * ratioLeft) + commonMinPWM;
  int finalR = (baseSpeed * ratioRight) + commonMinPWM;
  finalL = constrain(finalL, 0, 255);
  finalR = constrain(finalR, 0, 255);

  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, finalL);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, finalR);
}

//----------------------------------------------------------------------------------//
//-- nom fct : moveBackward  
//-- paramètre entrée : int - baseSpeed (Vitesse brute issue du PID)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Applique une commande de marche arrière avec prise en compte du ratio et de la zone morte
//-- démonstration : final = (baseSpeed * ratio) + zoneMorte
//-- aide - référence - lien : Contrôle Pont en H PWM
//----------------------------------------------------------------------------------//
void moveBackward(int baseSpeed) {
  int finalL = (baseSpeed * ratioLeft) + commonMinPWM;
  int finalR = (baseSpeed * ratioRight) + commonMinPWM;
  finalL = constrain(finalL, 0, 255);
  finalR = constrain(finalR, 0, 255);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, finalL);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, finalR);
}

//----------------------------------------------------------------------------------//
//-- nom fct : stopMotors  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Met la puissance PWM des deux moteurs à 0
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

//----------------------------------------------------------------------------------//
//-- nom fct : applyMotorControl  
//-- paramètre entrée : float - pid_output (commande calculée par le PID)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Décide si le robot doit avancer, reculer ou s'arrêter selon le PID et son statut de chute
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void applyMotorControl(float pid_output) {
  if (estTombe) {
    stopMotors();
    integral = 0; 
  } else {
    int plageUtile = 255 - commonMinPWM;
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
}

//==========================================
// UTILITAIRES DEBUG
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : printTelemetry  
//-- paramètre entrée : float - pid_output (valeur PID actuelle)
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Affiche les informations vitales sur le port série à intervalle régulier (250ms)
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
void printTelemetry(float pid_output) {
  static unsigned long last_print = 0;
  if (millis() - last_print > 250) {
    Serial.print("Angle: "); Serial.print(anglePitch);
    Serial.print(" | PID: "); Serial.print(pid_output);
    Serial.print(" | Etat: "); Serial.println(estTombe ? "COUCHE" : "DEBOUT");
    last_print = millis();
  }
}

//==========================================
// CALIBRATION (Reste globalement inchangé mais avec cartouche)
//==========================================

//----------------------------------------------------------------------------------//
//-- nom fct : runCalibrationRoutine  
//-- paramètre entrée : aucun
//-- paramètre sortie : aucun 
//-- paramètre référence (IN-OUT) : aucun 
//-- description : Routine bloquante qui calibre le MPU, la zone morte PWM, la symétrie des roues, puis sauvegarde
//-- démonstration : N/A
//-- aide - référence - lien : N/A
//----------------------------------------------------------------------------------//
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
  long sumAx=0, sumAz=0, sumGy=0; 
  for (int i = 0; i < 2000; i++) {
    readMPU();
    sumAx += accX; sumAz += accZ; sumGy += gyroY;
    delay(2);
  }
  offAccX = sumAx / 2000.0;
  offAccZ = (sumAz / 2000.0) - 16384.0;
  offGyroY = sumGy / 2000.0; 

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
// BOUCLES PRINCIPALES
//==========================================

void setup() {
  Serial.begin(115200);
  Serial.flush();
  
  display_init();
  sdcard_init();
  initHardwarePins();
  initMPU6050();
  loadCalibrationOrRun();
  
  previous_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;
  
  // Cadencement strict (100Hz max)
  if (dt < 0.01) return;
  previous_time = current_time;

  readMPU();
  calculateRobotAngle(dt);
  checkFallStatus();
  
  float pid_output = computePID(dt);
  applyMotorControl(pid_output);
  printTelemetry(pid_output);
}