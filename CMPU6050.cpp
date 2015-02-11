#include "CMPU6050.h"

TAxisData x_axis, y_axis, z_axis;
TAxisData axis_ptr[] = {x_axis, y_axis, z_axis};

// Instance constructor
CMPU6050::CMPU6050() {
  x_axis.gyro_offset = 87;
  y_axis.gyro_offset = 15;
  z_axis.gyro_offset = 24;
}

// Instance destructor
CMPU6050::~CMPU6050() {
}


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::Initialize
// Sets up the registers in the MPU-6050
//
// Parameters: N/A
// Returns:    N/A
//
int CMPU6050::Initialize() {
  // Begin and end transmission with MPU6050, see if it's on the I2C bus
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  if (Wire.endTransmission(1)) {
    return(1);
  }

  // Setup MPU6050
  WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_PWR_MGMT_1, 0b00000000);     // Clear the 'sleep' bit to start the sensor
  WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_GYRO_CONFIG, 0b00010000);    // Set gyro to +/-500 deg/sec sensitivity
  WriteRegister(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_CONFIG, 0b00001000);   // Set accelerometer to +/-4Gs sensitivity
  delay(50);  // Wait for MPU6050 to stabilize

  // Initialize axis angle values to accelerometer values
  if(GetData())
    return(1);
  // Set accel_angles to zero in case ComplementAccel() quits because accel_mag > 1.2Gs
  for (int i = 0; i < 3; i++) {
    axis_ptr[i].accel_angle = 0;
  }
  ComplementAccel();
  // Initialize all angles where z is still zero from above
  for (int i = 0; i < 3; i++) {
    axis_ptr[i].angle = axis_ptr[i].accel_angle;
  }

  return(0);
}

// Function: int CMPU6050::UpdateAngle(uint16_t dt)    (2500us)
int CMPU6050::UpdateAngle(uint16_t dt) {
  if(GetData())
    return(1);

  IntegrateGyro(dt);
  ComplementAccel();
  
  return(0);
}


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::GetData  (1800us)
// Reads data from MPU-6050
//
// Parameters: none
// Returns:    error
//
int CMPU6050::GetData() {
  // Read 14 bytes from MPU6050 starting at the x_acceleration_high byte
  if (ReadRegister(MPU6050_I2C_ADDRESS, MPU6050_ACCEL_XOUT_H, MPUdata.reg.buffer, 14))
    return(1);

  // Swap every other byte
  for (int i = 0; i < 7; i++) {
    SwapBytes(&MPUdata.reg.buffer[2*i], &MPUdata.reg.buffer[2*i + 1]);
  }

  // Set struct data members to values read (including offset for gyro values)
  for (int i = 0; i < 3; i++) {
    axis_ptr[i].gyro = MPUdata.raw.gyro[i] + axis_ptr[i].gyro_offset;
    axis_ptr[i].accel = MPUdata.raw.accel[i];
  }

  return(0);
}


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::IntegrateGyro    (190us)
// Updates gyro angle by integrating
//
// Parameters: dt - time interval for integration
// Returns:    none
//
void CMPU6050::IntegrateGyro(uint16_t dt) {
  for (int i = 0; i < 3; i++) {
    axis_ptr[i].angle += ((float)axis_ptr[i].gyro * dt * GYRO_SENS_1000000_INVERSE);
  }
  
  if (axis_ptr[2].angle < -180)
    axis_ptr[2].angle += 360;
  else if (axis_ptr[2].angle > 180)
    axis_ptr[2].angle -= 360;
}


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::ComplementAccel    (550us)
// Updates orientation estimate angle by factoring in accelerometer data
//
// Parameters: none
// Returns:    none
//
void CMPU6050::ComplementAccel() {
  // Calculate the magnitude of the acceleration
  long accel_sum = 0;
  for (int i = 0; i < 3; i++) {
    accel_sum += (axis_ptr[i].accel * axis_ptr[i].accel);
  }
  accel_mag = sqrt(accel_sum);

  // Complement the angle with accel data if accel_mag > .8G && < 1.2G
  if (((float)accel_mag/ACCEL_SENS) > .8 && ((float)accel_mag/ACCEL_SENS) < 1.2) {
    // Calculate angle of accel_mag
    axis_ptr[0].accel_angle = atan2(axis_ptr[1].accel, axis_ptr[2].accel) * RAD2DEG;
    axis_ptr[1].accel_angle = atan2(axis_ptr[0].accel, axis_ptr[2].accel) * -1 * RAD2DEG;

    // LP Complement current angle with accel_angle
    axis_ptr[0].angle = (FILTER_COEF * axis_ptr[0].angle) + ((float)(1.0-FILTER_COEF) * axis_ptr[0].accel_angle);
    axis_ptr[1].angle = (FILTER_COEF * axis_ptr[1].angle) + ((float)(1.0-FILTER_COEF) * axis_ptr[1].accel_angle);
  }
}


void CMPU6050::SwapBytes(uint8_t* x, uint8_t* y) {
  uint8_t temp = *x;
  *x = *y;
  *y = temp;
}
