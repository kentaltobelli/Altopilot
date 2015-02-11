#include "Control.h"

// Servo trim and control coefficients
const int elev_sub_trim = 100;
int servo_trim[] = {
  -78, 0, 0};
const float P_coef[] = {
  15, 20, -1, -5};
const float I_coef[] = {
  .02, .01, -.005, 0};
const float D_coef[] = {
  (-4 / GYRO_SENS), (-2 / GYRO_SENS), (2 / GYRO_SENS), 0};
const float set[] = {
  -5, 0, 0, AIRSPEED};


TServo elevator, r_aileron, l_aileron;
TServo* servo_ptr[] = {
  &elevator, &r_aileron, &l_aileron};

TControlVals pitch, roll, yaw, pitot;
TControlVals* ctrl_vals[] = {
  &pitch, &roll, &yaw, &pitot};




void ServoInit() {
  // Initialize control values
  for (int i = 0; i < 4; i++) {
    // Setup axis control data
    ctrl_vals[i]->P = P_coef[i];
    ctrl_vals[i]->I = I_coef[i];
    ctrl_vals[i]->D = D_coef[i];
    ctrl_vals[i]->set = set[i];
    ctrl_vals[i]->integral = 0;
  }
  ctrl_vals[3]->rate = 0;
  servo_trim[0] += elev_sub_trim;

  // Initialize servo values
  for (int i = 0; i < 3; i++) {
    // Setup servo
    servo_ptr[i]->center = 1500 + servo_trim[i];
    servo_ptr[i]->servo.attach(i+4, SERVO_LOWER, SERVO_UPPER);  // Attach servos to pins 5 - 7
    servo_ptr[i]->servo.writeMicroseconds(servo_ptr[i]->center);
  }
  delay(500);

  // Test servos
  for (int i = 0; i < 3; i++) {
    servo_ptr[i]->servo.writeMicroseconds(SERVO_LOWER + servo_trim[i]);
    delay(250);
    servo_ptr[i]->servo.writeMicroseconds(SERVO_UPPER + servo_trim[i]);
    delay(250);
    servo_ptr[i]->servo.writeMicroseconds(servo_ptr[i]->center);
  }
  delay(1000);
}

void ControlResponse(int use_pitot) {
  // Calculate angular error
  if (use_pitot) {
    if (airspeed > 4) {
      Serial.println("pitot detected");
      // Use PID function to generate set point for main PID to control to
      ctrl_vals[3]->error = ctrl_vals[3]->set - airspeed;
      PID(ctrl_vals[3], AIRSPEED / 10);
      //ctrl_vals[3]->response = (.2 * ctrl_vals[3]->response) + (.8 * ctrl_vals[3]->last_response);
      ctrl_vals[0]->set = ctrl_vals[3]->response;
    }
    else
      ctrl_vals[0]->set = set[0];

    // Find pitch response
    ctrl_vals[0]->error = ctrl_vals[0]->set - axis_ptr[0].angle;
    ctrl_vals[0]->rate = axis_ptr[0].gyro;
    //axis_ptr[i].last_angle = axis_ptr[i].angle;
    PID(ctrl_vals[0], SERVO_RANGE/4);
    ctrl_vals[0]->response = (.25 * ctrl_vals[0]->response) + (.75 * ctrl_vals[0]->last_response);
    ctrl_vals[0]->last_response = ctrl_vals[0]->response;
  }

  // Calculate error, feed to PID control, filter output
  for (int i = use_pitot; i < 3; i++) {
    ctrl_vals[i]->error = ctrl_vals[i]->set - axis_ptr[i].angle;
    ctrl_vals[i]->rate = axis_ptr[i].gyro;
    //axis_ptr[i].last_angle = axis_ptr[i].angle;
    PID(ctrl_vals[i], SERVO_RANGE/4);
    ctrl_vals[i]->response = (.25 * ctrl_vals[i]->response) + (.75 * ctrl_vals[i]->last_response);
    ctrl_vals[i]->last_response = ctrl_vals[i]->response;
  }
}

void PID(TControlVals* val_struct, int integral_limit) {
  val_struct->integral += (val_struct->I * val_struct->error);
  if (abs(val_struct->integral) > integral_limit) {
    if (val_struct->integral < 0)
      val_struct->integral = -1 * integral_limit;
    else
      val_struct->integral = integral_limit;
  }

  val_struct->response = (val_struct->P * val_struct->error) + (val_struct->D * val_struct->rate) + (val_struct->integral);
  //val_struct->last_error = val_struct->error;
}




