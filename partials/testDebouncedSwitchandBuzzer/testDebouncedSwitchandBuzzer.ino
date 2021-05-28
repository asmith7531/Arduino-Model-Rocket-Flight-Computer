#include <Wire.h>

const int buttonPin = 12;
const int ledPin = 11;
const int buzzerPin = 10;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Variables will change:
int redLEDState = HIGH;         // the current state of the output pin
void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);

  // set initial LED state
  digitalWrite(ledPin, redLEDState);
}

void loop() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      tone(10,1000,100);
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        redLEDState = !redLEDState;
      }
    }
  }

  // set the LED:
  digitalWrite(ledPin, redLEDState);
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}
//void setupMPU(){
//  Wire.beginTransmission(0b1101000); //This is the I2c address of the MPU
//  Wire.write((byte)0x6B); //Accessing the register 6B - Power Management (See 4.28 in MPU data sheet https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
//  Wire.write((byte)0b0000000); //Writing zero to every bit in the 0x6B register
//  Wire.endTransmission();
//  
//  Wire.beginTransmission(0b1101000);
//  Wire.write(0x1C); //Accesssing the register 1C - Accelerometer Configuration (See 4.5  in Data Sheet)
//  Wire.write(0b00001000); // Set to +-4g Full Scale Range, so the LSB Sensitivity is 8192 LSB/g
//  Wire.endTransmission();
//  
//  Wire.beginTransmission(0b1101000);
//  Wire.write(0x1B);//Accessing the register 1B - Gyroscope Configuration
//  Wire.write(0b00001100); //Set the Full Scale Range to +-1000 degress/s  
//  Wire.endTransmission();
//}
//
//void recordAccelRegisters(){
//  Wire.beginTransmission(0b1101000);
//  Wire.write(0x3B); //Starting register for Accel Readings
//  Wire.endTransmission();
//  Wire.requestFrom(0b1101000,6);
//  while(Wire.available()<6);
//  accelX = Wire.read()<<8|Wire.read();
//  accelY = Wire.read()<<8|Wire.read();
//  accelZ = Wire.read()<<8|Wire.read();
//  processAccelData();
//}
//
//void processAccelData(){
//  gForceX = accelX / 8192.0;
//  gForceY = accelY / 8192.0;
//  gForceZ = accelZ / 8192.0;
//}
//
//void recordGyroRegisters(){
//  Wire.beginTransmission(0b1101000);
//  Wire.write(0x43);
//  Wire.endTransmission();
//  Wire.requestFrom(0b1101000,6);
//  while(Wire.available()<6);
//  gyroX = Wire.read()<<8|Wire.read();
//  gyroY = Wire.read()<<8|Wire.read();
//  gyroZ = Wire.read()<<8|Wire.read();
//  processGyroData();
//}
//
//void processGyroData(){
//  rotX = gyroX/131.0;
//  rotY = gyroY/131.0;
//  rotZ = gyroZ/131.0;
//}
//
//void printData(bool on){
//    delay(500);
//    Serial.println((String)"Gyro (deg) X: " +rotX + " " + "Y: " + rotY + " " + "Z: " + rotZ);
//    Serial.println((String)" Accel (g) X: " +gForceX + " " + "Y: " + gForceY + " " + "Z: " + gForceZ);
//}
