#include <Arduino.h>
#include "Chassis.h"
#include "Romi32U4Buttons.h"


// encoder count targets, tune by turning 16 times and changing numbers untill offset is 0
#define NIGHTY_LEFT_TURN_COUNT -716
#define NIGHTY_RIGHT_TURN_COUNT 712


// 716 712
// positive = boost right motor; negative = boost left motor

// F and B go forward/backwards 50 cm by default, but other distances can be easily specified by adding a number after the letter
// S and E go the start/end distance
// L and R are left and right
// targetTime is target time (duh)
//char moves[200] = "B67 R F L F30 R R F60 B30 R F L F R F L F L F80 B80 L F L F L F30 B30 L F L F L F R F30 B30 R E";

// R R R R
char moves[200] = "2F R R R R";
double targetTime = 6; //75
double endDist = 41;
double startDist = -16;

// parameters are wheel diam, encoder counts, wheel track (tune these to your own hardware)
Chassis chassis(7, 1440, 14);
Romi32U4ButtonA buttonA;


// define the states
enum ROBOT_STATE { ROBOT_IDLE,
                   ROBOT_MOVE,
                   MOVING };
ROBOT_STATE robotState = ROBOT_IDLE;


// a helper function to stop the motors
void idle(void) {
  Serial.println("idle()");
  chassis.idle();
  robotState = ROBOT_IDLE;
}

void setup() {
  Serial.begin(115200);
  chassis.init();
  chassis.setMotorPIDcoeffs(7, 1.0);
}

void turnLeft() {
  chassis.turnFor(89, 60);
  delay(100);
  chassis.turnFor(89, 60);
  delay(100);
}

void left() {
  chassis.turnFor(3, 60);
  delay(100);
}

// CORRECTIONS FOR TURNING STRAIGHT

void leftTurnCorrection(float correctionLeft) {
  chassis.turnFor(-correctionLeft, 60); // PRIOR TO THE MOVEMENT

   

  delay(100);
  
}
void rightTurnCorrection(float correctionRight) {
  chassis.turnFor(-correctionRight, 60);

   

  delay(100);
  
}

void turnRight() {
  chassis.turnFor(-87, 60);
  delay(100);
}

void left(float seconds) {
  chassis.turnWithTimePosPid(NIGHTY_LEFT_TURN_COUNT, seconds);
}

void right(float seconds) {
  chassis.turnWithTimePosPid(NIGHTY_RIGHT_TURN_COUNT, seconds);
}

// =================== New Helper Functions (2F-5F) ==================

// returns leading number (2F -> 2). Defaults to 1 if none.
int getMoveMultiplier(String st) {
  int i = 0;
  while (i < st.length() && isDigit(st[i])) i++;
  if (i == 0) return 1;
  return st.substring(0, i).toInt();
}

// returns command letter (F, B, L, R, S, E)
char getMoveCommand(String st) {
  for (int i = 0; i < st.length(); i++) {
    if (isAlpha(st[i])) return st[i];
  }
  return '?';
}

// returns distance after command (3F30 -> 30), otherwise default
double getMoveDistance(String st, double defaultDist) {
  int i = 0;
  while (i < st.length() && !isAlpha(st[i])) i++;
  i++;
  if (i < st.length()) return st.substring(i).toDouble();
  return defaultDist;
}

void loop() {
  if (buttonA.getSingleDebouncedPress()) {
    delay(300);
    robotState = ROBOT_MOVE;
  }

  if (robotState == ROBOT_MOVE) {

    int count = 1;
    for (int i = 0; i < strlen(moves); i++)
      if (isSpace(moves[i])) count++;

    char *movesList[count];
    char *ptr = NULL;

    byte index = 0;
    ptr = strtok(moves, " ");
    while (ptr != NULL) {
      movesList[index] = ptr;
      index++;
      ptr = strtok(NULL, " ");
    }

    int numTurns = 0;
    double totalDist = 0;
    char currentChar;
    String st;

    // ===== COUNT TURNS + DISTANCE =====
    for (int i = 0; i < count; i++) {
      st = movesList[i];
      currentChar = getMoveCommand(st);

      if (currentChar == 'R' || currentChar == 'L') {
        numTurns++;
      }
      else if (currentChar == 'F' || currentChar == 'B') {   
    int multiplier = getMoveMultiplier(st);         // 2F → 2
    double dist = getMoveDistance(st, 50);
    totalDist += dist * multiplier;                 // account for multiple Fs
}

      /*else if (currentChar == 'F' || currentChar == 'B') {
        
        totalDist += getMoveDistance(st, 50);
      }*/
      else if (currentChar == 'S') {
        totalDist += abs(startDist);
      }
      else if (currentChar == 'E') {
        totalDist += abs(endDist);
      }
    }

    double turnTime = 0.55;
    double totalTurnTime = 0.65 * numTurns;
    double totalDriveTime = targetTime - totalTurnTime - 0.0029 * totalDist;

    // ===== EXECUTE MOVES =====
    for (int i = 0; i < count; i++) {

      st = movesList[i];
      char cmd = getMoveCommand(st);
      int mode = getMoveMultiplier(st);
      double dist = getMoveDistance(st, 50) * mode;

      double timeFrac = (dist) / totalDist * totalDriveTime;
      // double timeFrac = (dist * mode) / totalDist * totalDriveTime;
      double minTimeFrac = 0.15; // safe minimum for Romi

      if (timeFrac < minTimeFrac) 
        timeFrac = minTimeFrac;

      if (cmd == 'R') {
        right(turnTime);
      }
      else if (cmd == 'L') {
        left(turnTime);
      }
      else if (cmd == 'F') {
        float afterCorrectionFactor = 0; 
        float beforeCorrectionFactor = 0;

          switch(mode) {
              case 1: 
                afterCorrectionFactor = 1.0; 
                beforeCorrectionFactor = 1;
                break;  // F
              case 2: 
                afterCorrectionFactor = 0.75; 
                beforeCorrectionFactor = 0.75; // SHOULD BE HIGHER SINCE MORE ERROR FAVORING LEFT SIDE WILL ACCUMULATE
                break;  // 2F
              case 3: 
                afterCorrectionFactor = 0.3; 
                beforeCorrectionFactor = 0;
                break;  // 3F
              case 4: 
                afterCorrectionFactor = 0.25; 
                beforeCorrectionFactor = 0;
                break;  // 4F
              case 5: 
                afterCorrectionFactor = 0.25; 
                beforeCorrectionFactor = 0;
                break; // 5F almost no correction
              default: 
                afterCorrectionFactor = 0.5;
                beforeCorrectionFactor = 0;
          }
          leftTurnCorrection(beforeCorrectionFactor);
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(afterCorrectionFactor);
          chassis.idle();
          delay(40);
/*
        // ================= F =================
        if (mode == 1) {
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(0.5);
          chassis.idle();
          delay(40);
        }

        // ================= 2F =================
        else if (mode == 2) {
          // CUSTOM 2F LOGIC HERE
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(0.4);
          chassis.idle();
          delay(40);

        }

        // ================= 3F =================
        else if (mode == 3) {
          // CUSTOM 3F LOGIC HERE
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(0.3);
          chassis.idle();
          delay(40);
        }

        // ================= 4F =================
        else if (mode == 4) {
          // CUSTOM 4F LOGIC HERE
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(0.2);
          chassis.idle();
          delay(40);
        }

        // ================= 5F =================
        else if (mode == 5) {
          // CUSTOM 5F LOGIC HERE
          chassis.driveWithTime(dist, timeFrac);
          rightTurnCorrection(0.15);
          chassis.idle();
          delay(40);
        }*/
      }
      else if (cmd == 'B') {
        chassis.driveWithTime(-dist, timeFrac);
      }
      else if (cmd == 'S') {
        chassis.driveWithTime(startDist, abs(startDist)/totalDist * totalDriveTime);
      }
      else if (cmd == 'E') {
        chassis.driveWithTime(endDist, abs(endDist)/totalDist * totalDriveTime);
      }
    }

    idle();
  }
}
