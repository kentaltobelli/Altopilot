#ifndef _I2C_H_
#define _I2C_H_

#include "Arduino.h"
#include <Wire.h>


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::WriteRegister
// Writes a single byte to an I2C device
//
// Parameters: reg - address to write data to
//             data - byte to be written
// Returns:    error - nonzero if something goes wrong
//
static void WriteRegister(int address, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(address);  // Transmit to MPU6050
  Wire.write(reg);  // Queue desired register value
  Wire.write(data);  // Queue data to put in register
  Wire.endTransmission(1);  // Send data and release bus
}


// ----------------------------------------------------------------------------------------------------------------
// Function: MPU6050::ReadRegister
// Reads multiple bytes from an I2C device
//
// Parameters: start - address to begin reading at
//             buffer - pointer to store bytes read from device
//             length - amount of bytes to read after start
// Returns:    int - error
//
static int ReadRegister(int address, uint16_t start_reg, uint8_t* buffer, int length) {
  Wire.beginTransmission(address);  // Transmit to MPU6050
  Wire.write(start_reg);  // Queue desired start register value
  if (Wire.endTransmission(0))  // Send queued data but keep line open
    return(1);

  int i = 0;
  Wire.requestFrom(address, length, 1);  // Request bytes from MPU6050
  while (Wire.available() && i < length) {
    buffer[i++] = Wire.read();
  }

  // If buffer receives number of bytes request, return no error
  if (i == length)
    return(0);
  else
    return(1);
}

#endif
