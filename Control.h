#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <Servo.h>
#include "Arduino.h"
#include "CMPU6050.h"

#define AIRSPEED 10
#define ELEV_REV 1
#define LAIL_REV -1
#define RAIL_REV -1
#define PITOT_APIN 0
#define PITOT_CENTER 519
#define SERVO_LOWER 1000
#define SERVO_UPPER 2000
#define SERVO_RANGE (SERVO_UPPER - SERVO_LOWER)

extern int raw_airspeed;
extern float airspeed;
extern float last_airspeed;

// Servo assignments and 
struct TServo {
  Servo servo;
  int center;
};
extern TServo elevator, r_aileron, l_aileron;
extern TServo* servo_ptr[];


// Values for control coefficients
struct TControlVals {
  float P;
  float I;
  float D;
  float set;
  float error;
  float last_error;
  float rate;
  float integral;
  int response;
  int last_response;
};
extern TControlVals pitch, roll, yaw, pitot;
extern TControlVals* ctrl_vals[];


void ServoInit();
void ControlResponse(int use_pitot);
void PID(TControlVals* val_struct, int integral_limit);

#endif
