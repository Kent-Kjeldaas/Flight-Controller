 
#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
// IMU STUFF START
#include "I2Cdev.h"
#include "MPU9250.h"
MPU9250 accelgyro;
I2Cdev   I2C_M;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
// IMU STUFF END
 
SoftwareSerial bd(10 , 11); //RX, TX, DOUBLE CHECK
//servo
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
 
//DEFINE VARIABLES
int m1; //motor 1-2 = ROLL
int m2;
int m3; //motor 3-4 = PITCH
int m4;
int counter = 0;
int ledPin = 13; 
 
bool first = true; //is this the first time looping?
bool calibrated = false; //are sensored calibrated?
//Variables for ROLL
int X_eV;             // Angle error (for roll)
float X_gV;           // For storing accelerometer data
float X_gVArray [5];  // Put accelerometer data in array
float X_ACal;         // Accelerometer data offset calibration
float X_oVH;          // Desiered angular velocity
float X_gVH;          // For storing Gyro data
float X_gVHArray [5]; // Put gyro data in array
float X_GCal;         // Gyro data offset calibration
float X_eVH;          // Angualar velocity error
float X_dKraft;       // For storing thrust difference between propellers
float X_gVHmed;       // Average angular velcoity
float X_gVmed;        // Average angle
//Variables for PITCH
int Y_eV;             // Angle error (for pitch)
float Y_gV;
float Y_gVArray [5];
float Y_ACal;
float Y_oVH;
float Y_gVH;
float Y_gVHArray [5];
float Y_GCal;
float Y_eVH;
float Y_dKraft;
float Y_gVHmed;
float Y_gVmed;
 
//Settings
float kp1 = 0.6;    //Set P-term to tune desired angular velocity
float kp2 = 0.8;    //Set P-term to tune difference between m1 and m2
int thrust = 1390;  //Set thrust
int maxVal = 1590;
int minVal = 1190;
int X_oV = 0;       //Set desired angle for ROLL
int Y_oV = 0;       //Set desired angle for PITCH
 
 
void setup()
{
  bd.begin(9600); //We should change this
  bd.setTimeout(50); //50ms
  Serial.begin(250000);
  // IMU STUFF START
  Wire.begin();
  accelgyro.initialize();
  // IMU STUFF END
  motor1.attach(6); // ESC pin
  motor2.attach(9); // ESC pin
  motor3.attach(3); // ESC pin
  motor4.attach(5); // ESC pin
  //Leds
  pinMode(ledPin, OUTPUT);
}
 
 
void loop() {
  //CALIBRATE
  if(!calibrated){
    Serial.println("Break");
    delay(5000);
    Serial.println("Calibrating...");
    while(!calibrated){
      // IMU STUFF START
      if(counter%100 == 0){
        Serial.println(counter);
      }
      getAccel_Data();
      getGyro_Data();
      float pitchDeg = 180 * (atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]))) / PI; // degrees
      float rollDeg =  180 * (atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2]))) / PI;  // degrees
      // IMU STUFF END
      //Read data from accelerometer
      X_gV = rollDeg; // roll
      Y_gV = pitchDeg;  // Pitch
      //Save all accelerometer data in a array
      X_ACal = X_ACal + X_gV;
      Y_ACal = Y_ACal + Y_gV;
      //Read data from gyro
      X_gVH = Gxyz[1];   // Acc x
      Y_gVH = Gxyz[0];   // Acc y
      //Save all gyro data in a array
      X_GCal = X_GCal + X_gVH;
      Y_GCal = Y_GCal + Y_gVH;
     
      counter++;
     
      if (counter >= 1000) {
        //calcualte average
        X_ACal = X_ACal/1000;
        Y_ACal = Y_ACal/1000;
        X_GCal = X_GCal/1000;
        Y_GCal = Y_GCal/1000;

        Serial.println("Finished Calibrating");
        calibrated = true;
        //TURN ON LED
        digitalWrite(ledPin, HIGH); //It's calibrated
      }
    }
  }

  //ARM MOTORS
  if (first) {
    if (bd.available()) {
      char start = bd.read();
      if (start = '1') { //add start sign
        motor1.writeMicroseconds(1000);
        motor2.writeMicroseconds(1000);
        motor3.writeMicroseconds(1000);
        motor4.writeMicroseconds(1000);
        delay(3000);
        Serial.println("Armed");
        first = false;
      }
    }
  }
 
  while (!first and calibrated) {
    if (bd.available()) {
      char stopp = bd.read();
      if (stopp = '!') {
          motor1.writeMicroseconds(0);
          motor2.writeMicroseconds(0);
          motor3.writeMicroseconds(0);
          motor4.writeMicroseconds(0);
          first = true;
      }
    }
    // IMU STUFF START
    getAccel_Data();
    getGyro_Data();
    float pitchDeg = 180 * (atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]))) / PI; // degrees
    float rollDeg =  180 * (atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2]))) / PI;  // degrees
 
    float    pitchDegFiltered = pitchDegFiltered*0.945+0.05498*pitchDeg;     // First order Lowpass filter, Fc at 20 Hz.
    float   rollDegFiltered = rollDegFiltered*0.945+0.05498*rollDeg;        // First order Lowpass filter, Fc at 20 Hz.
   
    // IMU STUFF END
 
    //Read data from accelerometer and gyro
    float x_gVprev = X_gV;
    X_gV = rollDegFiltered - X_ACal; // Roll - offset
    X_gV = 0.5*X_gV + (0.5)*x_gVprev;           // exponentially weighed moving average filter. Try and fail with constant ,current_output  = α*current_input + (1-α)*previous_output
 
    float y_gVprev = Y_gV;
    Y_gV = pitchDegFiltered - Y_ACal;  // Pitch - offset
    Y_gV = 0.5*Y_gV + (0.5)*y_gVprev;           // exponentially weighed moving average. Try and fail with constant
   
    //Save all accelerometer data in a array
    X_gVArray[counter % 5] = X_gV;
    Y_gVArray[counter % 5] = Y_gV;
 
    X_gVH = Gxyz[1];   // Acc x
    Y_gVH = Gxyz[0];   // acc y
 
    //We might need this to limit extreme changes
    if (X_gVH > 50) {
      X_gVH = 50;
    }
    else if (X_gVH < -50) {
      X_gVH = -50;
    }
 
    if (Y_gVH > 50) {
      Y_gVH = 50;
    }
    else if (Y_gVH < -50) {
      Y_gVH = -50;
    }
 
    //Save all gyro data in a different array
    X_gVHArray[counter % 5] = X_gVH - X_GCal; //Actual X - offset
    Y_gVHArray[counter % 5] = Y_gVH - Y_GCal; //Actual Y - offset
 
    counter++;
 
    if (counter % 5 == 0) {
      //Compute for every five readings
      //Find average angle
      X_gVmed = (X_gVArray[0] + X_gVArray[1] + X_gVArray[2] + X_gVArray[3] + X_gVArray[4]) / 5;
      Y_gVmed = (Y_gVArray[0] + Y_gVArray[1] + Y_gVArray[2] + Y_gVArray[3] + Y_gVArray[4]) / 5;
      //Find average angular velocity
      X_gVHmed = (X_gVHArray[0] + X_gVHArray[1] + X_gVHArray[2] + X_gVHArray[3] + X_gVHArray[4] / 5);
      Y_gVHmed = (Y_gVHArray[0] + Y_gVHArray[1] + Y_gVHArray[2] + Y_gVHArray[3] + Y_gVHArray[4] / 5);
      /*
           //Find biggest number
           for (int i = 0; i < 4; i++) {
             if (X_gVHArray[i] > X_big) {
               X_big = X_gVHArray[i];
             }
             if (Y_gVHArray[i] > Y_big) {
               Y_big = Y_gVHArray[i];
             }
           }
           //remove biggest number (spikes)
           X_gVHmed = (X_gVHmed - X_big) / 4;
           Y_gVHmed = (Y_gVHmed - Y_big) / 4;
      */
      //Error angle
      X_eV = X_oV - X_gVmed;
      Y_eV = Y_oV - Y_gVmed;
      //Set desired angular velocity
      X_oVH = X_eV * kp1;
      Y_oVH = Y_eV * kp1;
 
      X_eVH = X_oVH - X_gVHmed;
      Y_eVH = Y_oVH - Y_gVHmed;
 
      //Thrust differential
      X_dKraft = X_eVH * kp2;
      Y_dKraft = Y_eVH * kp2;
      //Final thrust
      m1 = thrust + X_dKraft;
      m2 = thrust + Y_dKraft;
      m3 = thrust - X_dKraft;
      m4 = thrust - Y_dKraft;
 
      //Check if m1 exceeds the limit
      if (m1 > maxVal) {
        m1 = maxVal;
      }
      else if (m1 < minVal) {
        m1 = minVal;
      }
      //Check if m2 exceeds the limit
      if (m2 > maxVal) {
        m2 = maxVal;
      }
      else if (m2 < minVal) {
        m2 = minVal;
      }
      //Check if m3 exceeds the limit
      if (m3 > maxVal) {
        m3 = maxVal;
      }
      else if (m3 < minVal) {
        m3 = minVal;
      }
      //Check if m4 exceeds the limit
      if (m4 > maxVal) {
        m4 = maxVal;
      }
      else if (m4 < minVal) {
        m4 = minVal;
      }
 
      //Send PWM signals
      motor1.writeMicroseconds(m1);
      motor2.writeMicroseconds(m2);
      motor3.writeMicroseconds(m3);
      motor4.writeMicroseconds(m4);
 
    }
    if (counter % 10 == 0) {
      //Print pwm signals for motors, acc and gyro data for every 10 loop
      Serial.print("X : ");
      Serial.print(m1);
      Serial.print(" - ");
      Serial.print(m3);
      Serial.print(" - ");
      Serial.print(X_gVmed);
      Serial.print(" - ");
      Serial.print(X_gVHmed);
      Serial.print(" ");
      Serial.print("Y : ");
      Serial.print(m2);
      Serial.print(" - ");
      Serial.print(m4);
      Serial.print(" - ");
      Serial.print(Y_gVmed);
      Serial.print(" - ");
      Serial.print(Y_gVHmed);
      Serial.println(" ");
    }
  }
}
// IMU FUNCTIONS
void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = ((double) ax / 256) -  6;
  Axyz[1] = ((double) ay / 256) - 4;
  Axyz[2] = ((double) az / 256);
}
 
void getGyro_Data(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}