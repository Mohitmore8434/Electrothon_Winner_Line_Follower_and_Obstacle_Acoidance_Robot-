// Line follower + obstacle avoiding robot
// Board  : Arduino Uno R3
// Made by:Team MHT
//
// WIRING
//   L293D ENA (left speed)  -> D5      L293D ENB (right speed) -> D6
//   L293D IN1 -> D2   IN2 -> D4        (left motor)
//   L293D IN3 -> D7   IN4 -> D8        (right motor)
//   IR left  OUT -> D10                IR right OUT -> D11
//   Ultrasonic TRIG -> D12             Ultrasonic ECHO -> D13
//   Servo signal -> D9
//   Servo, IR, ultrasonic VCC -> 5V    All GND common (battery, Uno, L293D)
//   2x18650 (7.4V) -> L293D motor supply + Uno VIN
//
// NOTE: on Uno the Servo library uses Timer1, so analogWrite() on D9/D10
// does not work. That is fine here, motor PWM is on D5 and D6.
//
// How it works (my plan):
//  - 2 IR sensors follow the black line
//  - ultrasonic sensor sits on the servo, it keeps scanning the front only
//  - if something comes close -> stop, go back a little, scan wide
//  - first 3 times we go back straight, after that we go back in a curve
//    so the robot gets a new view and can find a free way
//  - when a free way is found, turn to that side and continue

#include <Servo.h>

// ---------- pins ----------
#define ENA 5        // left motor speed (PWM)
#define IN1 2
#define IN2 4
#define ENB 6        // right motor speed (PWM)
#define IN3 7
#define IN4 8

#define IR_L 10      // left IR sensor
#define IR_R 11      // right IR sensor

#define TRIG 12      // ultrasonic
#define ECHO 13
#define SERVO_PIN 9

// ---------- settings (change these while testing) ----------
bool irBlackHigh = true;       // if robot runs away from the line, make this false
bool leftMotorRev = false;     // if left motor spins wrong way, make this true
bool rightMotorRev = false;    // same for right motor
bool servoLeftIsHigh = true;   // if robot turns TOWARDS the obstacle, change this

int baseSpeed = 140;
int slowSpeed = 100;
int steer = 90;                // how hard it corrects on the line
int turnSpeed = 150;
int backSpeed = 130;
int backFast = 150;            // for curve reverse
int backSlow = 60;

int stopDist = 20;             // cm, obstacle detected
int slowDist = 35;             // cm, start slowing down
int freeDist = 35;             // cm, need at least this much to go that way

int backStepTime = 350;        // ms
int backCurveTime = 650;       // ms
float msPerDeg = 6.0;          // time to turn 1 degree, calibrate this on the floor

// ---------- radar (front only) ----------
#define CENTER 90
#define FRONT_MIN 70
#define FRONT_MAX 110
#define FRONT_STEP 10

// ---------- wide scan ----------
#define WIDE_MIN 30
#define WIDE_STEP 15
#define NUM_POINTS 9           // 30,45,60,...,150

Servo radar;

int radarAngle = CENTER;
int radarDir = 1;
unsigned long lastRadarTime = 0;
int hits = 0;
int frontDist = 200;

int dist[NUM_POINTS];          // wide scan results
int bestAngle = CENTER;
int bestClear = 0;
int leftAvg = 0;
int rightAvg = 0;

// ================= motors =================
void setMotor(int en, int a, int b, int spd, bool rev) {
  if (rev) spd = -spd;
  spd = constrain(spd, -255, 255);

  if (spd > 0) {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  } else if (spd < 0) {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  } else {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);
  }
  analogWrite(en, abs(spd));
}

void drive(int left, int right) {
  setMotor(ENA, IN1, IN2, left, leftMotorRev);
  setMotor(ENB, IN3, IN4, right, rightMotorRev);
}

void stopCar() {
  drive(0, 0);
}

// ================= sensors =================
bool lineSeen(int pin) {
  if (irBlackHigh) return digitalRead(pin) == HIGH;
  return digitalRead(pin) == LOW;
}

int getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long t = pulseIn(ECHO, HIGH, 12000);   // timeout so it doesn't hang
  if (t == 0) return 200;                // nothing in front
  int cm = t / 58;
  if (cm < 2) cm = 2;
  if (cm > 200) cm = 200;
  return cm;
}

// take 3 readings and use the middle one, sensor gives random wrong values sometimes
int getDistanceMedian() {
  int a = getDistance();
  delay(5);
  int b = getDistance();
  delay(5);
  int c = getDistance();

  int t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}

bool angleIsLeft(int angle) {
  if (servoLeftIsHigh) return angle > CENTER;
  return angle < CENTER;
}

// ================= radar while driving =================
void resetRadar() {
  radarAngle = CENTER;
  radarDir = 1;
  radar.write(CENTER);
  delay(200);
  hits = 0;
  frontDist = 200;
  lastRadarTime = millis();
}

// one reading every 45 ms, no delay() so the line following keeps running
// returns true if obstacle is really there
bool checkRadar() {
  if (millis() - lastRadarTime < 45) return false;
  lastRadarTime = millis();

  frontDist = getDistance();

  // move servo to next angle
  radarAngle = radarAngle + radarDir * FRONT_STEP;
  if (radarAngle >= FRONT_MAX) {
    radarAngle = FRONT_MAX;
    radarDir = -1;
  }
  if (radarAngle <= FRONT_MIN) {
    radarAngle = FRONT_MIN;
    radarDir = 1;
  }
  radar.write(radarAngle);

  // need 2 hits in a row, otherwise one wrong reading stops the robot
  if (frontDist <= stopDist) {
    hits++;
    if (hits >= 2) {
      hits = 0;
      return true;
    }
  } else {
    hits = 0;
  }
  return false;
}

// ================= wide scan =================
void wideScan() {
  radar.write(WIDE_MIN);
  delay(350);   // servo has to travel far the first time

  for (int i = 0; i < NUM_POINTS; i++) {
    int angle = WIDE_MIN + i * WIDE_STEP;
    radar.write(angle);
    if (i > 0) delay(120);
    dist[i] = getDistanceMedian();

    Serial.print(angle);
    Serial.print(":");
    Serial.print(dist[i]);
    Serial.print("  ");
  }
  Serial.println();

  bestAngle = CENTER;
  bestClear = 0;
  float bestScore = -1000;
  long leftSum = 0, rightSum = 0;
  int leftCount = 0, rightCount = 0;

  for (int i = 0; i < NUM_POINTS; i++) {
    int angle = WIDE_MIN + i * WIDE_STEP;

    // robot has some width so check the neighbours also, take the smallest
    int clear = dist[i];
    if (i > 0 && dist[i - 1] < clear) clear = dist[i - 1];
    if (i < NUM_POINTS - 1 && dist[i + 1] < clear) clear = dist[i + 1];

    // straight ways get a small bonus (less turning)
    float score = clear - abs(angle - CENTER) / 3.0;
    if (score > bestScore) {
      bestScore = score;
      bestAngle = angle;
      bestClear = clear;
    }

    if (angle != CENTER) {
      if (angleIsLeft(angle)) {
        leftSum += dist[i];
        leftCount++;
      } else {
        rightSum += dist[i];
        rightCount++;
      }
    }
  }

  leftAvg = leftSum / leftCount;
  rightAvg = rightSum / rightCount;

  radar.write(CENTER);
}

// ================= avoid moves =================
void backStraight() {
  drive(-backSpeed, -backSpeed);
  delay(backStepTime);
  stopCar();
  delay(100);
}

// goLeft = true -> nose of the robot swings to the left while going back
void backCurve(bool goLeft) {
  if (goLeft) {
    drive(-backFast, -backSlow);
  } else {
    drive(-backSlow, -backFast);
  }
  delay(backCurveTime);
  stopCar();
  delay(100);
}

void turnDeg(int deg, bool toLeft) {
  if (deg <= 0) return;
  if (toLeft) {
    drive(-turnSpeed, turnSpeed);
  } else {
    drive(turnSpeed, -turnSpeed);
  }
  delay(deg * msPerDeg);
  stopCar();
  delay(100);
}

void avoidObstacle() {
  Serial.println(F("obstacle!"));
  stopCar();
  delay(120);

  bool curveLeft = false;   // which side to curve, updated after every scan

  for (int attempt = 1; attempt <= 7; attempt++) {

    if (attempt <= 3) {
      backStraight();            // first 3 tries straight back
    } else {
      backCurve(curveLeft);      // then curve back
    }

    wideScan();
    curveLeft = (leftAvg > rightAvg);   // go towards the side with more space

    Serial.print(F("try "));
    Serial.print(attempt);
    Serial.print(F("  best angle "));
    Serial.print(bestAngle);
    Serial.print(F("  clear "));
    Serial.println(bestClear);

    if (bestClear >= freeDist) {
      int turn = abs(bestAngle - CENTER);
      if (turn >= 10) {
        turnDeg(turn, angleIsLeft(bestAngle));
      }
      return;    // free way found
    }
  }

  // nothing worked, turn around
  Serial.println(F("stuck, u-turn"));
  turnDeg(180, curveLeft);
}

// ================= line following =================
// line is between the 2 sensors
void followLine(bool slow) {
  int spd = baseSpeed;
  if (slow) spd = slowSpeed;

  bool L = lineSeen(IR_L);
  bool R = lineSeen(IR_R);

  if (!L && !R) {
    drive(spd, spd);                          // line in the middle, go straight
  } else if (L && !R) {
    drive(spd - steer, spd + steer / 2);      // turn left
  } else if (!L && R) {
    drive(spd + steer / 2, spd - steer);      // turn right
  } else {
    drive(spd, spd);                          // both on black (junction), just go
  }
}

// ================= main =================
void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  stopCar();
  radar.attach(SERVO_PIN);
  resetRadar();

  delay(1500);   // time to put the robot on the track
}

void loop() {
  if (checkRadar()) {
    avoidObstacle();
    resetRadar();
    return;
  }

  followLine(frontDist <= slowDist);
}
