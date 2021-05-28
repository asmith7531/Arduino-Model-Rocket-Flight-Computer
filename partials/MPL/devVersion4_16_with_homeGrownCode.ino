#include <Adafruit_MPL3115A2.h>
#include <Wire.h>
//MPU Variables:
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

// arduino pin variables:
const int buttonPin = 12;
const int ledPin = 11;
const int buzzerPin = 10;
const int droguePin = 3;

int buttonState;     // the current buttonReading from the input pin
int lastbuttonState = LOW;    // the previous buttonReading from the input pin

unsigned long lastDebounceTime = 0;   // the last time the output pin was toggled
unsigned long debounceDelay = 15;    // the debounce time; increase if the buttons feel unresponsive

int lastState = 0;
int flightState = 0;    // sets the state so we know if we are sitting on the pad, or in flight, etc.
int redLEDState = HIGH;     // the current state of the output pin

//MPL Altimeter Variables 
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
float lastAltitude;
float currentAltitude;
float maxAltitude;

float accelThresh = 2.0;      //used to set gyro threshold for setting flightState to in flight (flightState=2);  

//this function runs once when the arduino is powered on
void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(droguePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, redLEDState);
  Serial.begin(9600);
  Wire.begin();
  setupMPU(); 
  // setupMPL();
    if (! baro.begin()) {
    Serial.println("Couldnt find barometer sensor");
    return;
  };
}

//this function runs continuously on the arduino after the setup function completes
void loop() {
  int systemTime = millis();
  int buttonReading = digitalRead(buttonPin);
  //we are constantly running this case statement to keep track of the flightstate which will control if the rocket can launch, deploy chutes, and log data. 
  switch(flightState){
    case 0:
      armButton(buttonReading);
      Serial.println((String)+systemTime+": [Flight State "+flightState+"] Rocket is NOT armed.");
      break;
    case 1:
      armButton(buttonReading);
      recordAccelRegisters();
      recordGyroRegisters();
      recordAltitude();
      printData();
      if (lastState != 1){
      Serial.println((String)"Flight State: "+ flightState +" Rocket is ARMED... Standing by to detect launch.");
      };
      lastState = 1;
      if (accelY>accelThresh){
        flightState = 2;
        break;
      }; 
      break;
    case 2:   //this is the in flight state, needs to constantly be checking and recording the altitude so that we know where apogee is and can deploy our parachute 
      recordAccelRegisters();
      recordGyroRegisters();
      recordAltitude();
      if (currentAltitude > lastAltitude){
        maxAltitude = currentAltitude;
      }else if (currentAltitude < maxAltitude){
        deployDrogue();
        Serial.println((String)systemTime+" [Max Altitude Reached] :" + maxAltitude + " meters")
      };
      if (lastState != 2){
        Serial.println((String)"Flight State: "+flightState+" In Flight.");
      };
      lastState = 2;
      break;
    // case 3:
    //   if (lastState != 3){
    //   Serial.println((String)"Past Apogee!!! Falling....");
    //   }
    //   lastState = 3;
    //   break;
  }
}

void deployDrogue(){
  digitalWrite(droguePin,HIGH);//firing the parachute deployment charge, this will be a small black powder charge above the electronics bay. 
  delay(5000);
  digitalWrite(droguePin,LOW);
}

void armButton(int buttonReading){
//This function is called when the red colored arming button is pressed. Toggles the LEDs and flightState.
    if (buttonReading == lastbuttonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // Serial.println(lastDebounceTime);
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      tone(10,1000,100); 
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        redLEDState = !redLEDState; //set state to the opposite of the last known state
      }
      flightState = redLEDState; //toggle the flight state with the led on button press
    }
    digitalWrite(ledPin, redLEDState);
    lastbuttonState = buttonReading;
  }
  return;
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU
  Wire.write((byte)0x6B); //Accessing the register 6B - Power Management (See 4.28 in MPU register map/data sheet https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf for more details)
  Wire.write((byte)0b0000000); //Writing zero to every bit in the 0x6B register
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); //Accesssing the register 1C - Accelerometer Configuration (See 4.5  in Data Sheet)
  Wire.write(0b00001000); // Set to +-4g Full Scale Range, so the LSB Sensitivity is 8192 LSB/g
  Wire.endTransmission();
  
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);//Accessing the register 1B - Gyroscope Configuration
  Wire.write(0b00001100); //Set the Full Scale Range to +-1000 degress/s  
  Wire.endTransmission();
}

void recordAltitude(){
  currentAltitude = baro.getAltitude();//I gave up and just used the library, may return to this to try and fix my code to get data of the sensor. 
  return;
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
  return;
}

void processAccelData(){
  gForceX = accelX / 8192.0;
  gForceY = accelY / 8192.0;
  gForceZ = accelZ / 8192.0;
  return;
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
  return;
}

void processGyroData(){
  rotX = gyroX/131.0;
  rotY = gyroY/131.0;
  rotZ = gyroZ/131.0;
  return;
}

void printData(){
    delay(500);
    Serial.println((String)"Gyro (deg) X: " +rotX + " " + "Y: " + rotY + " " + "Z: " + rotZ);
    Serial.println((String)"Accel (g) X: " +accelX + " " + "Y: " + accelY + " " + "Z: " + accelZ);
    Serial.println((String)"Current Altitude: "+currentAltitude);
    //in this function we may want to refactor to create a one line string that is easily readable by processing to grab all the data and remove the delay.  
}
