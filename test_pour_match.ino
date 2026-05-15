#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ArduinoOTA.h>


// RESTE A CODER
// TRAJECTOIRE DE LA COUPE
// EVENTUEL DELAI AU DEPART
// TELEVERSER EN WIFI


// ===================== PINS =====================
#define DIR_A 0
#define PWM_A 1

#define DIR_B 7
#define PWM_B 25

#define ENC_A_A 8 
#define ENC_A_B 12 

#define ENC_B_A 10 
#define ENC_B_B 9 

#define SDA_PIN 23
#define SCL_PIN 24

#define SERVO_PIN 3
#define TIRETTE_PIN 5
#define SELECT_MODE 5

unsigned long tempsDepart = 0;
// ===================== SETUP TELEVERSEMENT WIFI =====================
const char* ssid     = "NomWifi";
const char* password = "MotDePasse";


// ────────────────────────────────────────────────────────────
//  PARAMÈTRES DU CORRECTEUR PID + ANTI-WINDUP
// ────────────────────────────────────────────────────────────

const float DT = 0.010f;  // Période d'échantillonnage

const float Kp = 0.4f;
const float Ki = 0.1f;
const float Kd = 0.0f;
const float Kb = 1.00f;

const int COMMANDE_MAX = 200;
const int COMMANDE_MAX_G = 200; // PARAMETRE POUR PAMI 1 (F_A_1/Baignoire) : 200 (Roue Gauche) / PAMI 2 : 195
const int COMMANDE_MAX_D = 194; // PARAMETRE POUR PAMI 1 : 194 / PAMI 2 : 200

const float TOLERANCE_TICKS = 200.0f;    // Fenêtre de convergence : |erreur| < 20 ticks ≈ 1,1 mm - A AUGMENTER AU BESOIN
const float TOLERANCE_TICKS_ROTA = 50.0f;
const unsigned long TIMEOUT_MS = 8000;  // Timeout de sécurité


// ────────────────────────────────────────────────────────────
// STRUCTURE D'ÉTAT PID
// ────────────────────────────────────────────────────────────

struct PIDState {
  float target;   // Consigne de position en ticks
  float integ;    // Accumulateur de l'intégrateur
  float errPrev;  // Erreur au pas k-1
};

PIDState pidG;  // PID moteur gauche
PIDState pidD;  // PID moteur droit

// ===================== TOF & SERVO =====================
Adafruit_VL53L0X lox;
Servo monServo;

// ===================== ROBOT =====================
const float diametreRoueMM = 38.0;
const float tickParTour = 1170.0; 
const float mmParTick = (diametreRoueMM * PI) / tickParTour;
const float TICKS_PAR_MM = tickParTour / (diametreRoueMM * PI);  // ≈ 18,6  ticks/mm

const float ENTRE_ROUES_MM = 120.0 ;

// ===================== PWM =====================
const int pwmFrequence = 20000;
const int pwmResolution = 8;


// ===================== OBSTACLE =====================
const int distanceObstacleMM = 120;
const unsigned long tempsClearObstacleMS = 2000;

// ===================== CONSTANTE SELECTION MODE ET TIRETTE =====================
const float SEUIL_TIRETTE_V = 0.50f; // 0V (tirette retirée) < 0.5 < 1.65V (tirette en place) 
const float SEUIL_MODE_V    = 2.50f;
const int JAUNE = 1;
const int BLEU = -1;
int MODE = 0;


// ===================== ENCODEURS =====================
volatile long ticksGauche = 0;
volatile long ticksDroit = 0;

volatile uint8_t prevStateL = 0;
volatile uint8_t prevStateR = 0;

const int8_t qTable[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};


// ===================== ISR =====================
void IRAM_ATTR isrLeft() {
  uint8_t a = digitalRead(ENC_A_A);
  uint8_t b = digitalRead(ENC_A_B);

  uint8_t state = (a << 1) | b;
  uint8_t index = (prevStateL << 2) | state;

  ticksGauche -= qTable[index]; 
  prevStateL = state;
}

void IRAM_ATTR isrRight() {
  uint8_t a = digitalRead(ENC_B_A);
  uint8_t b = digitalRead(ENC_B_B);

  uint8_t state = (a << 1) | b;
  uint8_t index = (prevStateR << 2) | state;

  ticksDroit += qTable[index]; 
  prevStateR = state;
}

// ===================== MOTEURS =====================

void setMotor(int dirPin, int pwmPin, int speedVal) {
  speedVal = constrain(speedVal, -255, 255);

  if (speedVal == 0) {
    ledcWrite(pwmPin, 0);
    return;
  }

  if (speedVal > 0) {
    digitalWrite(dirPin, HIGH);
    ledcWrite(pwmPin, speedVal);
  } else {
    digitalWrite(dirPin, LOW);
    ledcWrite(pwmPin, -speedVal);
  }
}

void setMotors(int gauche, int droit) {
  setMotor(DIR_A, PWM_A, - gauche); 
  setMotor(DIR_B, PWM_B, - droit); 
}

void stopMotors() {
  setMotors(0, 0);
}
// ===================== SELECTEUR MODE ET TIRETTE =====================
// Interrupteur position HAUTE (COTE S1) pour BLEU (1.65V), BASSE (VERS L'ARRIERE DU PAMI) pour JAUNE (3.3V)

float lireTension_tirette() {
  // Moyenne sur 8 lectures pour stabiliser 
  long sum = 0;
  for (int i = 0; i < 8; i++) { sum += analogRead(TIRETTE_PIN); delay(1); }
  return (sum / 8.0f) * 3.3f / 4095.0f;
}

int select_mode() {
  float v = lireTension_tirette();
  int mode = (v > SEUIL_MODE_V) ? JAUNE : BLEU;
  Serial.println(mode == JAUNE ? "JAUNE" : "BLEU");
  return mode;
  }

void attendreTirette() {
  //Lire le mode pendant que la tirette est en place
  MODE = select_mode();

  // Attendre que la tirette soit retirée (tension < seuil)
  while (lireTension_tirette() >= SEUIL_TIRETTE_V) {
    stopMotors();
    delay(20);
  }
  }

void attendre_remet_Tirette() {
  while (lireTension_tirette() < SEUIL_TIRETTE_V) {
    delay(100);
  }
  }

// ===================== TOF =====================
uint16_t lireDistanceMM() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  }

  return 0;
}

bool obstacleDetecte() {
  uint16_t d = lireDistanceMM();

  if (d > 0 && d < distanceObstacleMM) {
    Serial.print("Obstacle : ");
    Serial.print(d);
    Serial.println(" mm");
    return true;
  }

  return false;
}

void attendreObstacleDisparu() {
  stopMotors();

  unsigned long debutSansObstacle = 0;

  while (true) {
    bool obstacle = obstacleDetecte();

    if (!obstacle) {
      if (debutSansObstacle == 0) {
        debutSansObstacle = millis();
      }

      if (millis() - debutSansObstacle >= tempsClearObstacleMS) {
        return;
      }
    } else {
      debutSansObstacle = 0;
    }

    delay(50);
  }
}


// ===================== SERVO =====================
void actionServoFin() {
  while (true) {

    for (int angle = 0; angle <= 90; angle++) {
      monServo.write(angle);
      delay(50);  // (90 × 50 ms ≈ 4500 ms = 4.5s)
    }

    delay(500);

    for (int angle = 90; angle >= 0; angle--) {
      monServo.write(angle);
      delay(50);
    }

    delay(500);
  }
}


// ────────────────────────────────────────────────────────────
//  UN PAS DE CORRECTEUR PID AVEC ANTI-WINDUP
// ────────────────────────────────────────────────────────────

float pidStep(PIDState &pid, long ticks) {

  float err = pid.target - (float)ticks;

  float terme_P = Kp * err;
  float terme_I = Ki * pid.integ;
  float terme_D = Kd * (err - pid.errPrev) / DT;

  float u_raw = terme_P + terme_I + terme_D;
  float u_sat = constrain(u_raw, -(float)COMMANDE_MAX, (float)COMMANDE_MAX);

  // Mise à jour intégrateur
  pid.integ += DT * (err - Kb * (u_raw - u_sat));

  // Mis à jour pour correcteur dérivé
  pid.errPrev = err;

  return u_sat;
}

bool isAtTarget(float tolerance) {
  long tG, tD;
  noInterrupts();
  tG = ticksGauche;
  tD = ticksDroit;
  interrupts();

  return (fabs(pidG.target - (float)tG) < tolerance)
         && (fabs(pidD.target - (float)tD) < tolerance);
}

void resetForMove() {
  noInterrupts();
  ticksGauche = 0;
  ticksDroit = 0;
  interrupts();

  pidG.target = 0;
  pidG.integ = 0;
  pidG.errPrev = 0;
  pidD.target = 0;
  pidD.integ = 0;
  pidD.errPrev = 0;
}

void runPIDUntilDone(float tolerance) {
  unsigned long tStart = millis();
  unsigned long nextStep = millis();  // Prochaine date d'exécution du PID

  while (!isAtTarget(tolerance)) {

    // Timeout de sécurité
    if (millis() - tStart > TIMEOUT_MS) {
      Serial.println("[PID] TIMEOUT — arrêt d'urgence");
      break;
    }

    // Attendre sans bloquer les ISR jusqu'au prochain pas de calcul
    if (millis() < nextStep) continue;
    nextStep += (unsigned long)(DT * 1000.0f);  // Programmer le pas suivant (+10 ms)
    
     if (obstacleDetecte()) {
      stopMotors();
      attendreObstacleDisparu(); 
    }

    // Lecture des positions actuelles
    long tG, tD;
    noInterrupts();
    tG = ticksGauche;
    tD = ticksDroit;
    interrupts();

    // Calcul PID pour chaque moteur
    float cmdG = constrain(pidStep(pidG, tG), -(float)COMMANDE_MAX_G, (float)COMMANDE_MAX_G);
    float cmdD = constrain(pidStep(pidD, tD), -(float)COMMANDE_MAX_D, (float)COMMANDE_MAX_D);

    // Application des commandes (avec correction de sens physique)
    setMotors((int)(cmdG), (int)(cmdD));
  }

  stopMotors();
}
// ===================== AVANCER EN LIGNE DROITE =====================

void moveForward(float dist_mm) {
  resetForMove();

  long ticks_cible = (long)(dist_mm * TICKS_PAR_MM);

  pidG.target = (float)ticks_cible;
  pidD.target = (float)ticks_cible;

  runPIDUntilDone(TOLERANCE_TICKS);
}

void turnAngle(float angle_deg) {
  resetForMove();

  float angle_rad = angle_deg * PI / 180.0f;
  float corr_emp = ENTRE_ROUES_MM * 90.0f /95.0f;
  float arc_mm = (corr_emp / 2.0f) * angle_rad;
  long ticks_arc = (long)(arc_mm * TICKS_PAR_MM);

  pidG.target = -(float)ticks_arc;
  pidD.target = (float)ticks_arc;

  runPIDUntilDone(TOLERANCE_TICKS_ROTA);
}

// ===================== SETUP =====================

void setup() {

  Serial.begin(115200);
  delay(1000);
  monServo.attach(SERVO_PIN);

  pinMode(DIR_A, OUTPUT);
  digitalWrite(DIR_A, HIGH);
  delay(50);
 
  pinMode(DIR_B, OUTPUT);

  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);

  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  pinMode(TIRETTE_PIN, INPUT); 


  ledcAttach(PWM_A, pwmFrequence, pwmResolution);
  ledcAttach(PWM_B, pwmFrequence, pwmResolution);

  stopMotors();

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lox.begin()) {
    Serial.println("Erreur TOF");
    while (1)
      ;
  }

  monServo.attach(SERVO_PIN);
  monServo.write(0);
  Serial.print(MODE);
  prevStateL = (digitalRead(ENC_A_A) << 1) | digitalRead(ENC_A_B);
  prevStateR = (digitalRead(ENC_B_A) << 1) | digitalRead(ENC_B_B);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_B), isrLeft, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_B_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_B), isrRight, CHANGE);
}

// ===================== LOOP =====================

void loop() {
  attendreTirette();
  delay(8000);
  tempsDepart = millis();
  Serial.println(MODE);
  // Code pami F_A_1 / Baignoire, va en case du fond / Départ aligné ligne rouge
  if (MODE == 1) { 
    // JAUNE
   moveForward(1050);
    turnAngle(45);
    moveForward(200);
    turnAngle(-45);
    moveForward(200);
  } else {
    // BLEU
    moveForward(1050);
    turnAngle(-45);
    moveForward(350);
    turnAngle(45);
    moveForward(150);
  }
  // Code PAMI SCOTCH TRANSPARENT / PEINTRE (va en case B de chaque côté) / DEPART DERRIERE PAMI 1
 /*if (MODE == 1) { 
    // JAUNE
    delay(2000);
    moveForward(600);
    turnAngle(-35);
    moveForward(540);
  } else {
    // BLEU
    delay(2000);
    moveForward(600);
    turnAngle(35);
    moveForward(540);*/
    actionServoFin();
  attendre_remet_Tirette();
}


