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


/* ================= USER / GEOMETRY CONFIG ================= */


// TOTAL time (ms) the entire path must take
const unsigned long TOTAL_TARGET_TIME_MS = 10000UL; // e.g., 10 seconds


// If your "F50" test actually moved 79 cm, set measured to 79.0
const float MEASURED_50CM = 50.0f;
const float DESIRED_50CM  = 79.0f;


// encoder geometry (will be scaled by calibration)
const float COUNTS_PER_REV = 576.0f;
const float WHEEL_DIAM_CM = 3.2f;
float CM_PER_COUNT = (PI * WHEEL_DIAM_CM) / COUNTS_PER_REV;


// physical track width (center-to-center wheels) used to convert degrees to arc-length
const float TRACK_WIDTH_CM = 12.0f;


/* ================= IMU / GYRO ================= */
float headingDeg = 0.0f;
float gyroBias = 0.0f;
unsigned long lastMicros = 0;
const float GYRO_DPS_PER_LSB = 0.00875f; // LSM6 245 dps mode


/* ================= ORIGINAL GAINS (kept same) ================= */
float kP_drive = 1.0f;
float kD_drive = 0.02f;


float kP_turn  = 1.4f;
float kD_turn  = 0.05f;


/* ================= TRAJECTORY / CONTROL GAINS (calm forward) ================= */
const float Kp_pos = 0.55f;     // position proportional (counts)
const float Kd_vel = 0.006f;    // derivative on velocity error (counts/sec)


const int MAX_PWM_LINEAR = 140; // cap linear PWM (calm)
const int MAX_PWM_TURN   = 160; // cap turn PWM


const int MAX_HEADING_CORR = 50; // how much heading correction is allowed (use original 50 clamp)


/* ================= PARSER STORAGE ================= */
#define MAX_TOKENS 60
struct Token {
 char cmd;    // F,B,L,R
 float value; // cm for F/B (if 0 => default 50), degrees for L/R (if 0 => default 90)
};
Token tokens[MAX_TOKENS];
int tokenCount = 0;


/* Default moves string */
char moves[] = "F R F L F";


/* ================= UTIL ================= */
int clampPWM(int v)
{
 if (v > 300) return 300;
 if (v < -300) return -300;
 return v;
}


/* ================= IMU HELPERS ================= */
void calibrateGyro()
{
 const int samples = 800;
 long sum = 0;
 for (int i = 0; i < samples; ++i)
 {
   imu.read();
   sum += imu.g.z;
   delay(2);
 }
 gyroBias = (float)sum / (float)samples;
}


void resetHeading()
{
 headingDeg = 0.0f;
 lastMicros = micros();
}


void updateHeading()
{
 imu.read();
 unsigned long now = micros();
 float dt = (now - lastMicros) * 1e-6f;
 if (dt <= 0) dt = 1e-6f;
 lastMicros = now;


 float rate = (imu.g.z - gyroBias) * GYRO_DPS_PER_LSB;
 headingDeg += rate * dt;
}


/* ================= ENCODER HELPERS ================= */
void resetEncoders()
{
 encoders.getCountsAndResetLeft();
 encoders.getCountsAndResetRight();
}


long getLeftCount() { return encoders.getCountsLeft(); }
long getRightCount(){ return encoders.getCountsRight(); }
float countsToCM(float counts) { return counts * CM_PER_COUNT; }
float cmToCounts(float cm) { return cm / CM_PER_COUNT; }


/* ================= CALIBRATION: fix measured F50 -> desired 50cm ================= */
void applyCalibrationScale()
{
 if (MEASURED_50CM > 0.01f)
 {
   float scale = DESIRED_50CM / MEASURED_50CM;
   CM_PER_COUNT *= scale;
 }
}


/* ================= PARSER: expand tokens without STL ================= */
void parseAndExpandMoves(const char* s)
{
 tokenCount = 0;
 int i = 0;
 int n = strlen(s);


 while (i < n && tokenCount < MAX_TOKENS)
 {
   // skip spaces
   while (i < n && isspace((unsigned char)s[i])) i++;
   if (i >= n) break;


   // optional repeat count before command
   int repeat = 0;
   bool hasRepeat = false;
   while (i < n && isdigit((unsigned char)s[i]))
   {
     hasRepeat = true;
     repeat = repeat * 10 + (s[i] - '0');
     i++;
   }
   if (!hasRepeat) repeat = 1;


   // skip spaces
   while (i < n && isspace((unsigned char)s[i])) i++;
   if (i >= n) break;


   char cmd = s[i++];
   cmd = toupper((unsigned char)cmd);


   // skip spaces
   while (i < n && isspace((unsigned char)s[i])) i++;


   // optional numeric after command
   bool hasVal = false;
   int val = 0;
   int sign = 1;
   if (i < n && s[i] == '-')
   {
     sign = -1; i++;
   }
   while (i < n && isdigit((unsigned char)s[i]))
   {
     hasVal = true;
     val = val * 10 + (s[i] - '0');
     i++;
   }


   float v = 0.0f;
   if (hasVal) v = (float)(val * sign);


   for (int r = 0; r < repeat && tokenCount < MAX_TOKENS; ++r)
   {
     tokens[tokenCount].cmd = cmd;
     tokens[tokenCount].value = v;
     ++tokenCount;
   }
 }
}


/* ================= UTILITY: token equivalent distance for time allocation ================= */
float rotationDegreesToArcCM(float deg)
{
 return fabs(deg) * (PI * TRACK_WIDTH_CM / 360.0f);
}


float tokenEquivalentDistance(const Token &t)
{
 if (t.cmd == 'F' || t.cmd == 'B')
 {
   float d = (t.value > 0.0001f) ? fabs(t.value) : 50.0f;
   return d;
 }
 else if (t.cmd == 'L' || t.cmd == 'R')
 {
   float deg = (t.value > 0.0001f) ? fabs(t.value) : 90.0f;
   return rotationDegreesToArcCM(deg);
 }
 return 0.0f;
}


/* ================= DRIVE: time-aware calm position follower WITH ORIGINAL HEADING PD ================= */
void driveStraightTimed(float distCM, unsigned long allocatedMs)
{
 if (allocatedMs < 20) allocatedMs = 20;


 float targetCounts = cmToCounts(distCM); // could be negative


 resetEncoders();
 resetHeading();


 long prevAvgCounts = 0;
 unsigned long lastT_counts = micros();
 unsigned long lastTime_heading = micros();
 float prevHeadingErr = 0.0f;


 unsigned long start = millis();


 while (true)
 {
   // First update heading via IMU
   updateHeading();


   // timing for encoder velocity
   unsigned long now_counts = micros();
   float dt_counts = (now_counts - lastT_counts) * 1e-6f;
   if (dt_counts <= 0) dt_counts = 1e-6f;
   lastT_counts = now_counts;


   // timing for position profile
   unsigned long elapsedMs = millis() - start;
   if (elapsedMs > allocatedMs) elapsedMs = allocatedMs;
   float tt = (float)elapsedMs / (float)allocatedMs; // 0..1


   // desired smooth fraction (cosine)
   float s = 0.5f * (1.0f - cosf(PI * tt));
   float desiredCounts = targetCounts * s;


   // desired counts/sec
   float ds_dt = 0.5f * (PI / (float)allocatedMs * 1000.0f) * sinf(PI * tt);
   float desiredCountsPerSec = targetCounts * ds_dt;


   // measure actual counts and rate
   long curL = getLeftCount();
   long curR = getRightCount();
   float actualCounts = ((float)curL + (float)curR) * 0.5f;


   float actualCountsPerSec = (actualCounts - (float)prevAvgCounts) / dt_counts;
   prevAvgCounts = (long)actualCounts;


   // position and velocity error
   float posErr = desiredCounts - actualCounts;
   float velErr = desiredCountsPerSec - actualCountsPerSec;


   // base pwm from position/velocity follower
   float pwmBase = Kp_pos * posErr + Kd_vel * velErr;


   // clamp base pwm for calmness
   if (pwmBase > MAX_PWM_LINEAR) pwmBase = MAX_PWM_LINEAR;
   if (pwmBase < -MAX_PWM_LINEAR) pwmBase = -MAX_PWM_LINEAR;


   // *** ORIGINAL-STYLE HEADING PD (fixing drift left/right) ***
   // compute dt for heading derivative (use same dt as counts loop to be stable)
   unsigned long now_heading = micros();
   float dt_heading = (now_heading - lastTime_heading) * 1e-6f;
   if (dt_heading <= 0) dt_heading = 1e-6f;
   lastTime_heading = now_heading;


   float headingErr = -headingDeg; // want heading 0
   float dHeadingErr = (headingErr - prevHeadingErr) / dt_heading;
   prevHeadingErr = headingErr;


   float corr = kP_drive * headingErr + kD_drive * dHeadingErr;
   corr = constrain(corr, -MAX_HEADING_CORR, MAX_HEADING_CORR);


   // apply correction in same sign as original: left = base - corr, right = base + corr
   int leftPWM  = clampPWM((int)round(pwmBase - corr));
   int rightPWM = clampPWM((int)round(pwmBase + corr));


   motors.setSpeeds(leftPWM, rightPWM);


   // stopping criteria: at end of allocated time and within tolerance
   float remainingCounts = fabs(targetCounts - actualCounts);
   float remainingCM = fabs(remainingCounts * CM_PER_COUNT);


   if (elapsedMs >= allocatedMs && remainingCM <= 1.5f) break;


   // safety: if passed target and time expired
   if ((targetCounts >= 0 && actualCounts >= targetCounts) ||
       (targetCounts < 0 && actualCounts <= targetCounts))
   {
     if (elapsedMs >= allocatedMs) break;
   }


   if (millis() - start > allocatedMs + 500) break;


   delay(4);
 }


 motors.setSpeeds(0, 0);
 delay(100);
}


/* ================= TURN: REVERTED ORIGINAL PD TURN but with maxTime enforcement ================= */
void turnDegTimed(float targetDeg, unsigned long maxTimeMs)
{
 resetHeading();


 float prevErr = targetDeg;
 unsigned long lastTime = micros();
 unsigned long startTime = millis();


 while (true)
 {
   updateHeading();


   unsigned long now = micros();
   float dt = (now - lastTime) * 1e-6f;
   lastTime = now;
   if (dt <= 0) dt = 1e-6f;


   float err = targetDeg - headingDeg;
   float dErr = (err - prevErr) / dt;
   prevErr = err;


   // exit condition (same as original)
   if (abs(err) < 1.5 && abs(dErr) < 10)
     break;


   // timeout safety (use provided max time)
   if (millis() - startTime > maxTimeMs)
     break;


   float u = kP_turn * err + kD_turn * dErr;
   u = constrain(u, -160, 160);


   motors.setSpeeds(- (int)round(u), (int)round(u));


   delay(5);
 }


 motors.setSpeeds(0, 0);
 delay(200);
}


/* ================= WHOLE PATH EXECUTION: allocate exact ms and run tokens ================= */
void executeWholePathTimed(const char* moves)
{
 parseAndExpandMoves(moves);
 if (tokenCount == 0) return;


 // compute total equivalent distance for weighting (linear cm and rotation arc-cm)
 float totalEquivalent = 0.0f;
 for (int i = 0; i < tokenCount; ++i)
   totalEquivalent += tokenEquivalentDistance(tokens[i]);


 unsigned long allocMs[MAX_TOKENS];


 if (totalEquivalent <= 1e-6f)
 {
   unsigned long base = TOTAL_TARGET_TIME_MS / tokenCount;
   for (int i = 0; i < tokenCount; ++i) allocMs[i] = base;
   unsigned long used = base * tokenCount;
   if (used < TOTAL_TARGET_TIME_MS) allocMs[tokenCount-1] += (TOTAL_TARGET_TIME_MS - used);
 }
 else
 {
   unsigned long used = 0;
   for (int i = 0; i < tokenCount; ++i)
   {
     float part = tokenEquivalentDistance(tokens[i]) / totalEquivalent;
     unsigned long a = (unsigned long)round(part * (float)TOTAL_TARGET_TIME_MS);
     if (a < 30) a = 30;
     allocMs[i] = a;
     used += a;
   }
   if (used != TOTAL_TARGET_TIME_MS)
   {
     long diff = (long)TOTAL_TARGET_TIME_MS - (long)used;
     long newLast = (long)allocMs[tokenCount-1] + diff;
     if (newLast < 20) newLast = 20;
     allocMs[tokenCount-1] = (unsigned long)newLast;
   }
 }


 unsigned long planStart = millis();


 for (int i = 0; i < tokenCount; ++i)
 {
   Token &tk = tokens[i];
   unsigned long tms = allocMs[i];


   Serial.print("Token ");
   Serial.print(i);
   Serial.print(": ");
   Serial.print(tk.cmd);
   Serial.print(" ");
   Serial.print(tk.value);
   Serial.print(" alloc=");
   Serial.println(tms);


   if (tk.cmd == 'F' || tk.cmd == 'B')
   {
     float dist = (tk.value > 0.0001f) ? tk.value : 50.0f;
     if (tk.cmd == 'B') dist = -dist;
     driveStraightTimed(dist, tms);
   }
   else if (tk.cmd == 'L' || tk.cmd == 'R')
   {
     float deg = (tk.value > 0.0001f) ? tk.value : 90.0f;
     if (tk.cmd == 'R') deg = -deg;
     // use original PD turn controller behavior but force max time
     turnDegTimed(deg, tms);
   }


   // enforce schedule: if finished early, wait remainder for this token
   unsigned long elapsedSinceStart = millis() - planStart;
   unsigned long expectedUsed = 0;
   for (int j = 0; j <= i; ++j) expectedUsed += allocMs[j];
   if (elapsedSinceStart < expectedUsed)
   {
     unsigned long wait = expectedUsed - elapsedSinceStart;
     if (wait > 10) delay(wait);
   }
 }


 // if path finished early, wait remainder
 unsigned long totalElapsed = millis() - planStart;
 if (totalElapsed < TOTAL_TARGET_TIME_MS)
   delay(TOTAL_TARGET_TIME_MS - totalElapsed);
}


/* ================= SETUP & LOOP ================= */
void setup()
{
 Serial.begin(115200);
 Wire.begin();


 imu.init();
 imu.enableDefault();
 delay(2000);


 calibrateGyro();
 resetHeading();


 applyCalibrationScale();


 Serial.println("READY");
 Serial.print("CM_PER_COUNT = ");
 Serial.println(CM_PER_COUNT, 9);
 Serial.print("TOTAL_TARGET_TIME_MS = ");
 Serial.println(TOTAL_TARGET_TIME_MS);
}


void loop()
{
 if (!buttonA.getSingleDebouncedPress()) return;


 delay(250);


 Serial.print("RUN PATH: ");
 Serial.println(moves);


 executeWholePathTimed(moves);


 motors.setSpeeds(0, 0);
 Serial.println("PATH COMPLETE");
}



