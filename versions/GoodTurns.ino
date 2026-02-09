#include <Arduino.h>
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

/* ================= HARDWARE ================= */

Motors motors;
Encoders encoders;
ButtonA buttonA;
IMU imu;

/* ================= GEOMETRY ================= */

const float COUNTS_PER_REV = 576.0;
const float WHEEL_DIAM_CM = 3.2;

const float CM_PER_COUNT =
  (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;

/* ================= PATH ================= */

char moves[] = "F R F L F";

/* ================= GYRO ================= */

float headingDeg = 0;
float gyroBias = 0;
unsigned long lastMicros;

/* ===== CORRECT LSM6 SCALE (245 dps mode) ===== */
const float GYRO_DPS_PER_LSB = 0.00875;

/* ================= PID ================= */

float kP_drive = 1.0;
float kD_drive = 0.02;

float kP_turn  = 1.4;
float kD_turn  = 0.05;

/* ================= UTILS ================= */

int clampSpeed(int v)
{
  if (v > 300) return 300;
  if (v < -300) return -300;
  return v;
}

/* ================= IMU ================= */

void calibrateGyro()
{
  const int samples = 800;
  long sum = 0;

  for (int i = 0; i < samples; i++)
  {
    imu.read();
    sum += imu.g.z;
    delay(2);
  }

  gyroBias = sum / (float)samples;
}

void resetHeading()
{
  headingDeg = 0;
  lastMicros = micros();
}

void updateHeading()
{
  imu.read();

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  float rate =
    (imu.g.z - gyroBias) * GYRO_DPS_PER_LSB;

  headingDeg += rate * dt;
}

/* ================= ENCODERS ================= */

void resetEncoders()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

float getDistanceCM()
{
  long L = encoders.getCountsLeft();
  long R = encoders.getCountsRight();
  return (L + R) * 0.5f * CM_PER_COUNT;
}

/* ================= DRIVE STRAIGHT ================= */

void driveStraightCM(float distCM, int baseSpeed)
{
  resetEncoders();
  resetHeading();

  float prevErr = 0;
  unsigned long lastTime = micros();

  while (abs(getDistanceCM()) < abs(distCM))
  {
    updateHeading();

    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6;
    lastTime = now;

    float err = -headingDeg;
    float dErr = (err - prevErr) / dt;
    prevErr = err;

    float corr = kP_drive * err + kD_drive * dErr;
    corr = constrain(corr, -50, 50);

    motors.setSpeeds(
      clampSpeed(baseSpeed - corr),
      clampSpeed(baseSpeed + corr)
    );

    delay(5);
  }

  motors.setSpeeds(0,0);
  delay(150);
}

/* ================= TURN ================= */

void turnDeg(float targetDeg)
{
  resetHeading();

  float prevErr = targetDeg;
  unsigned long lastTime = micros();
  unsigned long startTime = millis();

  while (true)
  {
    updateHeading();

    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6;
    lastTime = now;

    float err = targetDeg - headingDeg;
    float dErr = (err - prevErr) / dt;
    prevErr = err;

    // exit condition
    if (abs(err) < 1.5 && abs(dErr) < 10)
      break;

    // timeout safety (3 seconds max)
    if (millis() - startTime > 3000)
      break;

    float u = kP_turn * err + kD_turn * dErr;
    u = constrain(u, -160, 160);

    motors.setSpeeds(-u, u);

    delay(5);
  }

  motors.setSpeeds(0, 0);
  delay(200);
}

/* ================= PATH ================= */

void executeMove(char c)
{
  if (c == 'F') driveStraightCM(50, 130);
  if (c == 'B') driveStraightCM(-50, -130);
  if (c == 'L') turnDeg(90);
  if (c == 'R') turnDeg(-90);
}

/* ================= SETUP ================= */

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  imu.init();
  imu.enableDefault();
  delay(2000);

  calibrateGyro();
  resetHeading();

  Serial.println("READY");
}

/* ================= LOOP ================= */

void loop()
{
  if (!buttonA.getSingleDebouncedPress())
    return;

  delay(300);

  for (int i = 0; i < strlen(moves); i++)
  {
    if (moves[i] == ' ') continue;
    executeMove(moves[i]);
  }

  motors.setSpeeds(0, 0);
}

