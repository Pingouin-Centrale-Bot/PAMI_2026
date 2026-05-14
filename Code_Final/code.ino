#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

// PINS

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

// ===================== OBSTACLE =====================
const int distanceObstacleMM = 120;
const unsigned long tempsClearObstacleMS = 2000;

// ===================== CONSTANTE SELECTION MODE ET TIRETTE =====================
const float SEUIL_TIRETTE_V = 0.50f; // 0V (tirette retirée) < 0.5 < 1.65V (tirette en place) 
const float SEUIL_MODE_V    = 2.50f;
const int JAUNE = 1;
const int BLEU = -1;
int MODE = 0;

// ===================== ROBOT =====================
const float diametreRoueMM = 38.0;
const float tickParTour = 2340.0;
const float mmParTick = (diametreRoueMM * PI) / tickParTour;
const float TICKS_PAR_MM = tickParTour / (diametreRoueMM * PI);  // ≈ 18,6  ticks/mm

const float ENTRE_ROUES_MM = 120.0 ;

// ===================== PWM =====================
const int pwmFrequence = 20000;
const int pwmResolution = 8;


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
  while   (lireTension_tirette() >= SEUIL_TIRETTE_V) {
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
Adafruit_VL53L0X lox;
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

// SERVO

Servo monServo;
void actionServoFin() {
  monServo.write(0);
  delay(300);

  monServo.write(90);
  delay(700);

  monServo.write(0);
  delay(300);
}

// MOTORS

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


// ISR
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

  ticksDroit -= qTable[index];
  prevStateR = state;
}

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


// Position control

const float Kp = 0.1;
const float Ki = 0.01;
const int DT = 1; // ms


const float error_acceptance = 200.0;

void control_loop(long r_left, long r_right, bool front){
  float error_right = r_right - ticksDroit;
  float error_left = r_left - ticksGauche;

  float integral_error_right = DT*error_right/1000;
  float integral_error_left = DT*error_left/1000;

  unsigned long previous = 0;

  while (abs(error_left) > error_acceptance || abs(error_right) > error_acceptance){
    if (millis() - previous >= DT){
      previous = millis();

      float u_right = -Kp*error_right - Ki*integral_error_right;
      u_right = constrain(u_right, -255, 255);

      float u_left = -Kp*error_left - Ki*integral_error_left;
      u_left = constrain(u_left, -253, 253);

      error_right = r_right - ticksDroit;
      error_left = r_left - ticksGauche;

      integral_error_right = DT*error_right/1000;
      integral_error_left = DT*error_left/1000;

      setMotor(DIR_B, PWM_B, u_left);
      setMotor(DIR_A, PWM_A, u_right);


      Serial.println("-----------------");
      Serial.println(u_left);
      Serial.println(u_right);
      Serial.println(error_left);
      Serial.println(error_right);
    }
  }
  stopMotors();
}

void angular_move_control_loop(float angle){
  int r_left = (100)*10*angle + ticksGauche;
  int r_right = (100 + ENTRE_ROUES_MM)*10*angle + ticksDroit;
  control_loop(r_left, r_right, false);
}

void angular_control_loop(float angle){
  int r_left = -(98)*7*angle + ticksGauche;
  int r_right = +(98)*7*angle + ticksDroit;
  control_loop(r_left, r_right, false);

}

void linear_control_loop(float distance){
  // 92.0 mm -> 10cm
  // distance -> ?
  int r_left = distance * 1000 / 920 + ticksGauche;
  int r_right = distance * 1000 / 920 + ticksDroit;
  control_loop(r_left, r_right, true);
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
}


void loop() {
  attendreTirette();
  Serial.println(MODE);


  if (MODE == 1){
    // JAUNE
    linear_control_loop(6000);
    angular_control_loop(-PI/3);
    linear_control_loop(10150);
    angular_control_loop(PI/3);
    linear_control_loop(2000);
  }
  else{
    // BLEU
    linear_control_loop(7000);
    angular_control_loop(5*PI/12);
    linear_control_loop(9500);
    angular_control_loop(-5*PI/12);
    linear_control_loop(3000);
  }


  actionServoFin();

  Serial.println("ok");
  attendre_remet_Tirette(); 
}
