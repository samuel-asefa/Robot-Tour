#include <Arduino.h>
#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include <math.h>


using namespace Pololu3piPlus32U4;


/* ================= HARDWARE ================= */
Motors motors;
Encoders encoders;
ButtonA buttonA;
IMU imu;


/* ================= GLOBAL TIME (Robot Tour Requirement) ================= */
const unsigned long TOTAL_TARGET_TIME_MS = 61 * 1000UL;


/* ================= GEOMETRY ================= */
const float COUNTS_PER_REV = 576.0f;
const float WHEEL_DIAM_CM  = 3.2f;
float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;
const float TRACK_WIDTH_CM = 12.0f;


/* ================= IMU ================= */
float gyroBias = 0;
float headingDeg = 0;
unsigned long lastIMUTime = 0;
const float GYRO_DPS_PER_LSB = 0.00875f;


/* ================= STRAIGHT HEADING PID ================= */
const float Kp_heading = 9.0f;
const float Kd_heading = 0.40f;
const int MAX_CORR = 180;


/* ================= POSITION CONTROL ================= */
const float Kp_pos = 0.6f;
const float Kd_vel = 0.006f;
const int MAX_PWM = 170;


/* ================= TOKEN STORAGE ================= */
#define MAX_TOKENS 60
struct Token {
 char cmd;
 float value;
};
Token tokens[MAX_TOKENS];
int tokenCount = 0;


/* ================= DEFAULT MOVES ================= */
char moves[] = "F30 F R F L F F F L F L F L F R F L F R F L F R F R R F L F R F R F L F F L F";
// F30 F R F L F F F L F L F L F R F L F R F L F R F R R F L F R F R F L F F L F
// F30 F R F L F F R F L F
/* ================= UTIL ================= */
int clampPWM(int v)
{
 if (v > 300) return 300;
 if (v < -300) return -300;
 return v;
}


/* ================= IMU ================= */
void calibrateGyro()
{
 long sum = 0;
 for (int i = 0; i < 1000; i++)
 {
   imu.read();
   sum += imu.g.z;
   delay(2);
 }
 gyroBias = sum / 1000.0f;
}


void resetHeading()
{
 headingDeg = 0;
 lastIMUTime = micros();
}


float updateIMU()
{
 imu.read();
 unsigned long now = micros();
 float dt = (now - lastIMUTime) * 1e-6f;
 lastIMUTime = now;
 if (dt <= 0) dt = 1e-6f;


 float rate = (imu.g.z - gyroBias) * GYRO_DPS_PER_LSB;
 headingDeg += rate * dt;


 return rate;
}


/* ================= ENCODERS ================= */
void resetEncoders()
{
 encoders.getCountsAndResetLeft();
 encoders.getCountsAndResetRight();
}


float getAverageCounts()
{
 return (encoders.getCountsLeft() + encoders.getCountsRight()) * 0.5f;
}


float cmToCounts(float cm)
{
 return cm / CM_PER_COUNT;
}


/* ================= PARSER ================= */
void parseMoves(const char* s)
{
 tokenCount = 0;
 int i = 0;
 int n = strlen(s);


 while (i < n && tokenCount < MAX_TOKENS)
 {
   while (i < n && isspace(s[i])) i++;
   if (i >= n) break;


   char cmd = toupper(s[i++]);


   float value = 0;
   bool hasVal = false;


   while (i < n && isdigit(s[i]))
   {
     hasVal = true;
     value = value * 10 + (s[i] - '0');
     i++;
   }


   tokens[tokenCount].cmd = cmd;
   tokens[tokenCount].value = hasVal ? value : 0;
   tokenCount++;
 }
}


/* ================= STRAIGHT (TIMED + INERTIAL LOCK) ================= */
void driveStraightTimed(float distCM, unsigned long allocatedMs)
{
 float targetCounts = cmToCounts(distCM);


 resetEncoders();
 resetHeading();


 float desiredHeading = headingDeg;


 float prevCounts = 0;
 unsigned long start = millis();
 unsigned long lastTime = micros();


 while (true)
 {
   float gyroRate = updateIMU();


   unsigned long now = micros();
   float dt = (now - lastTime) * 1e-6f;
   lastTime = now;
   if (dt <= 0) dt = 1e-6f;


   float counts = getAverageCounts();
   float vel = (counts - prevCounts) / dt;
   prevCounts = counts;


   float elapsed = millis() - start;
   if (elapsed > allocatedMs) elapsed = allocatedMs;


   float t = elapsed / allocatedMs;
   float smooth = 0.5f * (1 - cosf(PI * t));
   float desiredCounts = targetCounts * smooth;


   float posErr = desiredCounts - counts;


   float basePWM = Kp_pos * posErr - Kd_vel * vel;
   basePWM = constrain(basePWM, -MAX_PWM, MAX_PWM);


   float headingErr = desiredHeading - headingDeg;
   float correction = Kp_heading * headingErr - Kd_heading * gyroRate;
   correction = constrain(correction, -MAX_CORR, MAX_CORR);


   int leftPWM  = clampPWM((int)(basePWM - correction));
   int rightPWM = clampPWM((int)(basePWM + correction));


   motors.setSpeeds(leftPWM, rightPWM);


   if (elapsed >= allocatedMs && fabs(posErr) < cmToCounts(1.0f))
     break;


   delay(4);
 }


 motors.setSpeeds(0,0);
}


/* ================= TURN ================= */
void turnTimed(float deg, unsigned long allocatedMs)
{
 resetHeading();
 unsigned long start = millis();


 while (true)
 {
   float gyroRate = updateIMU();
   float err = deg - headingDeg;


   float u = 2.2f * err - 0.45f * gyroRate;
   u = constrain(u, -180, 180);


   motors.setSpeeds(-u, u);


   if (fabs(err) < 1.5f && fabs(gyroRate) < 8)
     break;


   if (millis() - start > allocatedMs)
     break;


   delay(5);
 }


 motors.setSpeeds(0,0);
}


/* ================= PATH EXECUTION ================= */
float equivalentDistance(Token &t)
{
 if (t.cmd == 'F' || t.cmd == 'B')
   return (t.value > 0) ? t.value : 50.0f;


 if (t.cmd == 'L' || t.cmd == 'R')
 {
   float deg = (t.value > 0) ? t.value : 90.0f;
   return fabs(deg) * (PI * TRACK_WIDTH_CM / 360.0f);
 }
 return 0;
}


void executePath(const char* s)
{
 parseMoves(s);
 if (tokenCount == 0) return;


 float totalEq = 0;
 for (int i=0;i<tokenCount;i++)
   totalEq += equivalentDistance(tokens[i]);


 unsigned long startTotal = millis();


 for (int i=0;i<tokenCount;i++)
 {
   float portion = equivalentDistance(tokens[i]) / totalEq;
   unsigned long alloc = portion * TOTAL_TARGET_TIME_MS;


   if (tokens[i].cmd == 'F' || tokens[i].cmd == 'B')
   {
     float dist = (tokens[i].value > 0) ? tokens[i].value : 50.0f;
     if (tokens[i].cmd == 'B') dist = -dist;
     driveStraightTimed(dist, alloc);
   }
   else if (tokens[i].cmd == 'L' || tokens[i].cmd == 'R')
   {
     float deg = (tokens[i].value > 0) ? tokens[i].value : 90.0f;
     if (tokens[i].cmd == 'R') deg = -deg;
     turnTimed(deg, alloc);
   }
 }


 unsigned long totalElapsed = millis() - startTotal;
 if (totalElapsed < TOTAL_TARGET_TIME_MS)
   delay(TOTAL_TARGET_TIME_MS - totalElapsed);
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
 if (!buttonA.getSingleDebouncedPress()) return;


 delay(300);


 executePath(moves);


 Serial.println("DONE");
}