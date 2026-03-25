/*
 * TEST SIMPLE : Rotation Moteurs (ON/OFF) + Lecture Encodeurs
 * Matériel: ESP32-S3 Self Balanced Robot
 */

// --- DÉFINITION DES PINS (D'après ton fichier PDF) ---
// Moteur GAUCHE (Motor 0)
const int ENA = 46;  // PWM (Vitesse)
const int IN1 = 7;   // Direction 1
const int IN2 = 18;  // Direction 2
const int ENC_L_A = 16; // Encodeur A
const int ENC_L_B = 17; // Encodeur B

// Moteur DROIT (Motor 1)
const int ENB = 45;  // PWM (Vitesse)
const int IN3 = 10;  // Direction 1
const int IN4 = 11;  // Direction 2
const int ENC_R_A = 9;  // Encodeur A
const int ENC_R_B = 47; // Encodeur B

volatile long countLeft = 0;
volatile long countRight = 0;

// Interruptions pour compter les ticks
void IRAM_ATTR readEncoderLeft() {
  if (digitalRead(ENC_L_A) == digitalRead(ENC_L_B)) { countLeft++; } 
  else { countLeft--; }
}

void IRAM_ATTR readEncoderRight() {
  if (digitalRead(ENC_R_A) == digitalRead(ENC_R_B)) { countRight++; } 
  else { countRight--; }
}

void setup() {
  Serial.begin(115200);
  
  // Configuration en SORTIE
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuration en ENTRÉE pour les encodeurs
  pinMode(ENC_L_A, INPUT); // Pas de INPUT_PULLUP car souvent géré par le circuit
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  // Attacher les interruptions
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderRight, CHANGE);

  Serial.println("--- TEST DÉMARRÉ ---");
  Serial.println("ATTENTION: Les moteurs vont tourner à pleine vitesse dans 2 secondes !");
  delay(2000);
}

void loop() {
  // --- COMMANDE MOTEUR (ON/OFF seulement) ---
  
  // Moteur GAUCHE : AVANCE
  // Sur ce driver, ENA doit être HIGH pour activer le moteur
  digitalWrite(ENA, HIGH); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Moteur DROIT : RECULE
  digitalWrite(ENB, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // --- AFFICHAGE SÉRIE ---
  Serial.print("Encodeur G: ");
  Serial.print(countLeft);
  Serial.print(" | Encodeur D: ");
  Serial.println(countRight);
  
  delay(200);
}