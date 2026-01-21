#include <Arduino.h>
#include "Chassis.h"
#include "Romi32U4Buttons.h"

#include <Wire.h>
#include <LSM6.h>

/* =========================================================
   CONSTANTS / HARDWARE CONFIG
   ========================================================= */

// encoder count targets (kept for turning)
#define NIGHTY_LEFT_TURN_COUNT  -716
#define NIGHTY_RIGHT_TURN_COUNT  712

// path string
char moves[200] = "2F R R R R";

// global timing targets
double targetTime = 6.0;
double endDist   = 41;
double startDist = -16;

// chassis params
Chassis chassis(7, 1440, 14);
Romi32U4ButtonA buttonA;

// IMU
LSM6 imu;

/* =========================================================
   ROBOT STATE
   ========================================================= */

enum ROBOT_STATE {
  ROBOT_IDLE,
  ROBOT_MOVE
};

ROBOT_STATE robotState = ROBOT_IDLE;

/* =========================================================
   GYRO / HEADING CONTROL
   ========================================================= */

float headingDeg = 0.0;
unsigned long lastGyroMicros = 0;

// PID (gyro)
float kP_gyro = 1.2;
float kD_gyro = 0.08;

float gyroBiasZ = 0.0;


void calibrateGyro() {
  const int samples = 500;
  long sum = 0;

  Serial.println("Calibrating gyro... keep robot still");

  for (int i = 0; i < samples; i++) {
    imu.read();
    sum += imu.g.z;
    delay(5);
  }

  gyroBiasZ = (sum / (float)samples) / 1000.0;
  Serial.println("Gyro calibration done");
}


float gyroError = 0;
float gyroPrevError = 0;

float targetHeading = 0;

/* =========================================================
   UTILITY
   ========================================================= */

void idle() {
  chassis.idle();
  robotState = ROBOT_IDLE;
}

/* =========================================================
   GYRO FUNCTIONS
   ========================================================= */

void resetHeading() {
  headingDeg = 0;
  gyroPrevError = 0;
  lastGyroMicros = micros();
}

void updateGyro() {
  imu.read();

  unsigned long now = micros();
  float dt = (now - lastGyroMicros) / 1000000.0;
  lastGyroMicros = now;

  float gyroZ = (imu.g.z / 1000.0) - gyroBiasZ;
  headingDeg += gyroZ * dt;
}


/* =========================================================
   STRAIGHT DRIVE USING GYRO (TIME-BASED)
   ========================================================= */

void driveStraightGyro(double distCm, double timeSec) {
  resetHeading();
  targetHeading = 0;

  unsigned long start = millis();
  unsigned long duration = timeSec * 1000;

  int baseSpeed = (distCm >= 0) ? 60 : -60;

  while (millis() - start < duration) {
    updateGyro();

    gyroError = targetHeading - headingDeg;
    float gyroDerivative = gyroError - gyroPrevError;
    gyroPrevError = gyroError;

    float correction = kP_gyro * gyroError + kD_gyro * gyroDerivative;


    int leftEffort  = baseSpeed - correction;
int rightEffort = baseSpeed + correction;


    // clamp for safety
    leftEffort  = constrain(leftEffort,  -100, 100);
    rightEffort = constrain(rightEffort, -100, 100);

    chassis.setMotorEfforts(leftEffort, rightEffort);

    delay(5);
  }

  chassis.idle();
  delay(40);
}


/* =========================================================
   TURNING (UNCHANGED)
   ========================================================= */

void left(double seconds) {
  chassis.turnWithTimePosPid(NIGHTY_LEFT_TURN_COUNT, seconds);
}

void right(double seconds) {
  chassis.turnWithTimePosPid(NIGHTY_RIGHT_TURN_COUNT, seconds);
}

/* =========================================================
   PATH PARSING HELPERS (UNCHANGED)
   ========================================================= */

int getMoveMultiplier(String st) {
  int i = 0;
  while (i < st.length() && isDigit(st[i])) i++;
  if (i == 0) return 1;
  return st.substring(0, i).toInt();
}

char getMoveCommand(String st) {
  for (int i = 0; i < st.length(); i++) {
    if (isAlpha(st[i])) return st[i];
  }
  return '?';
}

double getMoveDistance(String st, double defaultDist) {
  int i = 0;
  while (i < st.length() && !isAlpha(st[i])) i++;
  i++;
  if (i < st.length()) return st.substring(i).toDouble();
  return defaultDist;
}

/* =========================================================
   SETUP
   ========================================================= */

void setup() {
  Serial.begin(115200);

  chassis.init();
  chassis.setMotorPIDcoeffs(7, 1.0);

  Wire.begin();                 // 1️⃣ I2C FIRST

  if (!imu.init()) {            // 2️⃣ IMU INIT
    Serial.println("IMU INIT FAILED");
    while (1);
  }

  imu.enableDefault();          // 3️⃣ ENABLE SENSOR
  delay(500);

  calibrateGyro();              // 4️⃣ NOW IT IS SAFE
}


/* =========================================================
   MAIN LOOP
   ========================================================= */

void loop() {
char movesCopy[200];
strcpy(movesCopy, moves);

  if (buttonA.getSingleDebouncedPress()) {
    delay(300);
    robotState = ROBOT_MOVE;
  }

  if (robotState != ROBOT_MOVE) return;

  /* ===== TOKENIZE PATH ===== */

  int count = 1;
  for (int i = 0; i < strlen(moves); i++)
    if (isSpace(moves[i])) count++;

  char *movesList[count];
  char *ptr = strtok(movesCopy, " ");
  int idx = 0;

  while (ptr != NULL) {
    movesList[idx++] = ptr;
    ptr = strtok(NULL, " ");
  }

  /* ===== PRE-SCAN ===== */

  int numTurns = 0;
  double totalDist = 0;

  for (int i = 0; i < count; i++) {
    String st = movesList[i];
    char cmd = getMoveCommand(st);

    if (cmd == 'L' || cmd == 'R') {
      numTurns++;
    }
    else if (cmd == 'F' || cmd == 'B') {
      int mult = getMoveMultiplier(st);
      double dist = getMoveDistance(st, 50);
      totalDist += dist * mult;
    }
    else if (cmd == 'S') totalDist += abs(startDist);
    else if (cmd == 'E') totalDist += abs(endDist);
  }

  /* ===== TIME ALLOCATION ===== */

  double turnTime = 0.55;
  double totalTurnTime = 0.65 * numTurns;
  double totalDriveTime = targetTime - totalTurnTime - 0.0029 * totalDist;

  /* ===== EXECUTION ===== */

  for (int i = 0; i < count; i++) {

    String st = movesList[i];
    char cmd = getMoveCommand(st);
    int mult = getMoveMultiplier(st);
    double dist = getMoveDistance(st, 50) * mult;

    double timeFrac = (abs(dist) / totalDist) * totalDriveTime;
    if (timeFrac < 0.15) timeFrac = 0.15;

    if (cmd == 'L') {
      left(turnTime);
    }
    else if (cmd == 'R') {
      right(turnTime);
    }
    else if (cmd == 'F') {
      driveStraightGyro(dist, timeFrac);
    }
    else if (cmd == 'B') {
      driveStraightGyro(-dist, timeFrac);
    }
    else if (cmd == 'S') {
      driveStraightGyro(startDist, abs(startDist) / totalDist * totalDriveTime);
    }
    else if (cmd == 'E') {
      driveStraightGyro(endDist, abs(endDist) / totalDist * totalDriveTime);
    }
  }

  idle();
}
