// MPU-6050 3-Axis Accelerometer & Gyroscope
// by Kent Altobelli
// May 2014

#include <Wire.h>  // Included for I2C communication with MPU6050
#include <Servo.h>  // Included to drive output servos/gimbals
#include "CMPU6050.h"  // Header file for MPU6050
#include "Control.h"

#define ISR_FREQ 250  // Interrupt service routine frequency (62 to 7800 Hz)

CMPU6050 MPU;  // Declare instance of the MPU6050 class

bool pitot_attached = 0;
int raw_airspeed;
float airspeed = 0;
float last_airspeed = 0;

// Loop variables
uint16_t dt;  // Stores time in micros for gyro integration
uint32_t start;
uint32_t finish;

volatile uint8_t ISR_count = 0;  // Count variable
volatile uint8_t orient_update = 0;  // Update orientation flag
volatile uint8_t servo_update = 0;  // Update servo positions
volatile uint8_t airspeed_update = 0;  // Update airspeed measurement

int error;                    // Error code returned when MPU6050 is read
int temp;
long tic;
long toc;

// Main functions
//void UpdateOrientation();
//void UpdateServos();
//void UpdateAirspeed();
//void CenterAll();


void setup()
{
  Serial.begin(115200);

  // MPU6050 setup
  Wire.begin();  // Initialize I2C bus
  ServoInit();  // Initialize servos
  MPU.Initialize();  // Initialize MPU6050
  ISR_init((float)ISR_FREQ);  // Initialize timer 2 at specified interrupt frequency

  // Check for pitot tube sensor
  raw_airspeed = analogRead(PITOT_APIN);
  if (raw_airspeed > 500 && raw_airspeed < 575) pitot_attached = 1;
  Serial.println(pitot_attached);

  finish = micros();
}

void loop()
{
  if (orient_update)  UpdateOrientation();
  if (servo_update)  UpdateServos();
  if (airspeed_update)  UpdateAirspeed();

  // Other loop stuff

}



void UpdateOrientation() {
  orient_update = 0;

  finish = micros();  // Mark loop time for gyro integration
  dt = finish - start;
  start = finish;

  do {
    error = MPU.UpdateAngle(dt);
    if (error)  CenterAll();
  } 
  while (error);
  /*
    for (int i = 0; i < 3; i++) {
   Serial.print(axis_ptr[i].angle);
   Serial.print(" ");
   }
   Serial.println();
   Serial.println();
   */
}

void UpdateServos() {
  servo_update = 0;

  ControlResponse(pitot_attached);
  Serial.println(ctrl_vals[0]->response);
  Serial.println(ctrl_vals[1]->response);
  Serial.println();
  // Elevator, LAileron, RAileron
  servo_ptr[0]->servo.writeMicroseconds(ELEV_REV * (ctrl_vals[0]->response) + servo_ptr[0]->center);
  servo_ptr[1]->servo.writeMicroseconds(LAIL_REV * (ctrl_vals[1]->response + ctrl_vals[2]->response) + servo_ptr[1]->center);
  servo_ptr[2]->servo.writeMicroseconds(RAIL_REV * (ctrl_vals[1]->response + ctrl_vals[2]->response) + servo_ptr[2]->center);
}

void UpdateAirspeed() {
  airspeed_update = 0;

  raw_airspeed = analogRead(PITOT_APIN) - PITOT_CENTER;
  if (raw_airspeed <= 0)
    airspeed = 0;
  else
    airspeed = sqrt((2 * (float)raw_airspeed) / 1.225);

  airspeed = .25 * airspeed + .75 * last_airspeed;
  last_airspeed = airspeed;
}

void CenterAll() {
  // Center all servos
  for (int i = 0; i < 3; i++) {
    servo_ptr[i]->servo.writeMicroseconds(servo_ptr[i]->center);
  }
}


ISR(TIMER2_COMPA_vect)  // Interrupt service routine @ 200Hz
{
  ISR_count++;
  orient_update = 1;  // Update orientation data
  if ((ISR_count % 5) == 0)  servo_update = 1;  // Update servo positions
  if ((ISR_count % 25) == 0)  airspeed_update = 1;  // Sample Airspeed
  if (ISR_count >= 250)  ISR_count = 0;  // Reset ISR_count

}


void ISR_init(float frequency)
{
  /*  pg. 162
  http://www.atmel.com/images/doc8161.pdf
   */

  cli();                    // Disable global interrupts

  TCCR2A = 0;               // Clear Timer A register
  TCCR2B = 0;               // Clear Timer B register
  TCNT2 = 0;                // Set counter value to 0
  OCR2A = (uint8_t)((16000000 / (frequency * 1024)) - 1);  // Reset value
  TCCR2A |= (1 << WGM21);   // Turn on CTC mode
  TCCR2B = TCCR2B | (1 << CS22) | (1 << CS21) | (1 << CS20);  // 1024 Prescaler
  TIMSK2 |= (1 << OCIE2A);  // Enable compare match interrupt

  sei();                    // Enable global interrupts
}

