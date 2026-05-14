#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

// ===================== PINS =====================
#define DIR_A 0
#define PWM_A 1

#define DIR_B 7
#define PWM_B 25

#define ENC_A_A 10
#define ENC_A_B 9

#define ENC_B_A 8
#define ENC_B_B 12

#define SDA_PIN 23
#define SCL_PIN 24

#define SERVO_PIN 3
#define TIRETTE_PIN 5

// ────────────────────────────────────────────────────────────
//  PARAMÈTRES DU CORRECTEUR PID + ANTI-WINDUP
// ────────────────────────────────────────────────────────────

const float DT = 0.010f;  // Période d'échantillonnage

const float Kp = 0.80f;
const float Ki = 0;
const float Kd = 0;
const float Kb = 1.00f;

const int COMMANDE_MAX = 200;
const float TOLERANCE_TICKS = 20.0f;    // Fenêtre de convergence : |erreur| < 20 ticks ≈ 1,1 mm - A AUGMENTER AU BESOIN
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
const float tickParTour = 2340.0;
const float mmParTick = (diametreRoueMM * PI) / tickParTour;
const float TICKS_PAR_MM = tickParTour / (diametreRoueMM * PI);  // ≈ 18,6  ticks/mm

const float ENTRE_ROUES_MM = 120.0;

// ===================== PWM =====================
const int pwmFrequence = 20000;
const int pwmResolution = 8;

// ===================== OBSTACLE =====================
const int distanceObstacleMM = 120;
const unsigned long tempsClearObstacleMS = 2000;

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

  ticksGauche += qTable[index];
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

// ===================== ENCODEURS =====================
void resetEncodeurs() {
  noInterrupts();
  ticksGauche = 0;
  ticksDroit = 0;
  interrupts();
}

void lireTicks(long &gauche, long &droit) {
  noInterrupts();
  gauche = ticksGauche;
  droit = ticksDroit;
  interrupts();
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
  setMotor(DIR_A, PWM_A, gauche);
  setMotor(DIR_B, PWM_B, droit);
}

void stopMotors() {
  setMotors(0, 0);
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

// ===================== TIRETTE =====================
void attendreTirette() {
  Serial.println("En attente tirette...");

  while (digitalRead(TIRETTE_PIN) == HIGH) {
    stopMotors();
  }

  Serial.println("Depart !");
}

// ===================== SERVO =====================
void actionServoFin() {
  monServo.write(0);
  delay(300);

  monServo.write(90);
  delay(700);

  monServo.write(0);
  delay(300);
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

bool isAtTarget() {
  long tG, tD;
  noInterrupts();
  tG = ticksGauche;
  tD = ticksDroit;
  interrupts();

  return (fabs(pidG.target - (float)tG) < TOLERANCE_TICKS)
         && (fabs(pidD.target - (float)tD) < TOLERANCE_TICKS);
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

void runPIDUntilDone() {
  unsigned long tStart = millis();
  unsigned long nextStep = millis();  // Prochaine date d'exécution du PID

  while (!isAtTarget()) {

    if (obstacleDetecte()) {
      stopMotors();
      attendreObstacleDisparu();
    }
    // Timeout de sécurité
    if (millis() - tStart > TIMEOUT_MS) {
      Serial.println("[PID] TIMEOUT — arrêt d'urgence");
      break;
    }

    // Attendre sans bloquer les ISR jusqu'au prochain pas de calcul
    if (millis() < nextStep) continue;
    nextStep += (unsigned long)(DT * 1000.0f);  // Programmer le pas suivant (+10 ms)

    // Lecture des positions actuelles
    long tG, tD;
    noInterrupts();
    tG = ticksGauche;
    tD = ticksDroit;
    interrupts();

    // Calcul PID pour chaque moteur
    float cmdG = pidStep(pidG, tG);
    float cmdD = pidStep(pidD, tD);

    // Application des commandes (avec correction de sens physique)
    setMotors((int)(-cmdG), (int)(-cmdD));

    // AFFICHAGE DE DEBOGAGE
    Serial.print("G: err=");
    Serial.print(pidG.target - (float)tG, 0);
    Serial.print(" cmd=");
    Serial.print(cmdG, 0);
    Serial.print("  |  D: err=");
    Serial.print(pidD.target - (float)tD, 0);
    Serial.print(" cmd=");
    Serial.println(cmdD, 0);
  }

  stopMotors();
  Serial.println("[PID] Arrivé !");
}
// ===================== AVANCER EN LIGNE DROITE =====================

void moveForward(float dist_mm) {
  resetForMove();

  long ticks_cible = (long)(dist_mm * TICKS_PAR_MM);

  pidG.target = (float)ticks_cible;
  pidD.target = (float)ticks_cible;

  Serial.print("[moveForward] ");
  Serial.print(dist_mm);
  Serial.print(" mm → ");
  Serial.print(ticks_cible);
  Serial.println(" ticks");

  runPIDUntilDone();
}

void turnAngle(float angle_deg) {
  resetForMove();

  float angle_rad = angle_deg * PI / 180.0f;
  float arc_mm = (ENTRE_ROUES_MM / 2.0f) * angle_rad;
  long ticks_arc = (long)(arc_mm * TICKS_PAR_MM);

  pidG.target = -(float)ticks_arc;
  pidD.target = (float)ticks_arc;

  Serial.print("[turnAngle] ");
  Serial.print(angle_deg);
  Serial.print(" deg → arc=");
  Serial.print(arc_mm, 1);
  Serial.print(" mm → ");
  Serial.print(ticks_arc);
  Serial.println(" ticks");

  runPIDUntilDone();
}

// ===================== SETUP =====================

void setup() {

  Serial.begin(115200);
  delay(1000);
  monServo.attach(SERVO_PIN);

  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  pinMode(ENC_A_A, INPUT_PULLUP);
  pinMode(ENC_A_B, INPUT_PULLUP);

  pinMode(ENC_B_A, INPUT_PULLUP);
  pinMode(ENC_B_B, INPUT_PULLUP);

  pinMode(TIRETTE_PIN, INPUT_PULLUP);


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

  prevStateL = (digitalRead(ENC_A_A) << 1) | digitalRead(ENC_A_B);
  prevStateR = (digitalRead(ENC_B_A) << 1) | digitalRead(ENC_B_B);

  attachInterrupt(digitalPinToInterrupt(ENC_A_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_B), isrLeft, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC_B_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_B), isrRight, CHANGE);

  Serial.println("==================================");
  Serial.println("Tape une distance en mm puis Enter");
  Serial.println("Exemple : 300");
  Serial.println("==================================");
}

// ===================== LOOP =====================

void loop() {
  attendreTirette();
  moveForward(300);
  actionServoFin();
}