#include <Arduino.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

/* ======================================================
   HARDWARE
   ====================================================== */

Motors motors;
Encoders encoders;
ButtonA buttonA;

/* ======================================================
   CONSTANTS
   ====================================================== */

const float COUNTS_PER_REV = 576.0;   // verify gearbox
const float WHEEL_DIAM_CM = 3.2;

const float CM_PER_COUNT =
  (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;

/* ======================================================
   PATH
   ====================================================== */

char moves[200] = "F R F L F";

/* ======================================================
   UTILITY
   ====================================================== */

int clampSpeed(int v)
{
  if (v > 400) return 400;
  if (v < -400) return -400;
  return v;
}

/* ======================================================
   ENCODERS
   ====================================================== */

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

/* ======================================================
   DRIVE STRAIGHT (ENCODER ONLY)
   ====================================================== */

void driveStraightCM(float distCM, int speed)
{
  resetEncoders();

  while (abs(getDistanceCM()) < abs(distCM))
  {
    motors.setSpeeds(
      clampSpeed(speed),
      clampSpeed(speed)
    );
  }

  motors.setSpeeds(0, 0);
  delay(100);
}

/* ======================================================
   TURN IN PLACE (ENCODER APPROX)
   ====================================================== */

void turnDeg(float deg)
{
  resetEncoders();

  // rough constant: tune later
  // initially 420 --> multiplied by 0.4 --> 168
  const float COUNTS_PER_90 = 168.0;
  float target = abs(deg) / 90.0 * COUNTS_PER_90;

  int dir = (deg > 0) ? 1 : -1;

  while (abs(encoders.getCountsLeft()) < target)
  {
    motors.setSpeeds(
      clampSpeed(-dir * 120),
      clampSpeed(dir * 120)
    );
  }

  motors.setSpeeds(0, 0);
  delay(100);
}

/* ======================================================
   PATH EXECUTION
   ====================================================== */

void executeMove(char c)
{
  if (c == 'F') driveStraightCM(50, 120);
  if (c == 'B') driveStraightCM(-50, -120);
  if (c == 'L') turnDeg(90);
  if (c == 'R') turnDeg(-90);
}

/* ======================================================
   SETUP / LOOP
   ====================================================== */

void setup()
{
  Serial.begin(115200);
  Serial.println("READY (ENCODER MODE)");
}

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
}
