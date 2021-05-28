#include <Wire.h>
//MPU Variables:
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;


void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2c address of the MPU
  Wire.write((byte)0x6B); //Accessing the register 6B - Power Management (See 4.28 in MPU data sheet https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
  Wire.write((byte)0b0000000); //Writing zero to every bit in the 0x6B register
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); //Accesssing the register 1C - Accelerometer Configuration (See 4.5  in Data Sheet)
  Wire.write(0b00001000); // Set to +-4g Full Scale Range, so the LSB Sensitivity is 8192 LSB/g
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);//Accessing the register 1B - Gyroscope Configuration
  Wire.write(0b00001000); //Set the Full Scale Range to +-1000 degress/s  
  Wire.endTransmission();
}

void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 8192.0;
  gForceY = accelY / 8192.0;
  gForceZ = accelZ / 8192.0;
}

void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();
  processGyroData();
}

void processGyroData(){
  rotX = gyroX/32.8;
  rotY = gyroY/32.8;
  rotZ = gyroZ/32.8;
}

void printData(){
  delay(500);
  Serial.println((String)"Gyro (deg) X: " +rotX + " " + "Y: " + rotY + " " + "Z: " + rotZ);
  Serial.println((String)" Accel (deg) X: " +gForceX + " " + "Y: " + gForceY + " " + "Z: " + gForceZ);
}
