#include <Servo.h>                         // Include the Servo.h library so we can use servos to adjust pitch on propeller.
#include <SoftwareSerial.h>                // Include the SoftwareSerial.h library so we can use bluetooth from other pins than tx/rx.
#include <Wire.h>                          // Include the Wire.h library so we can communicate with the gyro.
#include "I2Cdev.h"                        // Include the I2Cdev.h library.
#include "MPU9250.h"                       // Include the MPU9250.h library so we can use the MPU9250 IMU.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETTINGS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor and Controller Values
float kp1 = 0.8;                            // Set P-term to tune desired angular velocity
float kp2 = 1;                              // Set P-term to tune difference between m1 and m2
int thrust = 1390;                          // Set thrust
int maxVal = 1590;                          // Max thrust
int minVal = 0;                             // Min thrust
int X_oV = 0;                               // Set desired angle for ROLL
int Y_oV = 0;                               // Set desired angle for PITCH
//Calibration Settings
const int num_samples_cal = 2000;           // Number of samples in calibration
const int num_samples_cycle = 20;           // Number of samples per cycle
const int num_samples_print = 40;           // Number of samples before we print
//Gyro Limits
const int maxGyroVal = 50;                  // Set a limit for max deg/sec gyro meassurement
const int minGyroVal = -50;                 // Set a limit for max deg/sec gyro meassurement
// Sort Settings
bool sort = false;                          // Do we want to remove x lowest and x highest values?
int remove_num_spikes = 5;                  // How many spikes do we want to remove, if any (sort must be true for this to work).


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MPU9250 accelgyro;
I2Cdev   I2C_M;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float pitchDeg;
float rollDeg;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SoftwareSerial bd(10 , 11);                 // RX, TX
// Servo
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
 
// Define Variables|
int m1 = 0;                                 // motor 1-3 = ROLL
int m2 = 0;
int m3 = 0;                                 // motor 2-4 = PITCH
int m4 = 0;
int counter = 0;
int ledPin = 13;
 
bool first = true;                          // Is this the first time looping?
bool calibrated = false;                    // Are sensor calibrated?
// Variables for ROLL
int X_eV;                                   // Angle error (for roll)
float X_gV;                                 // For storing accelerometer data
float X_gVArray [num_samples_cycle];        // Put accelerometer data in array
float X_ACal;                               // Accelerometer data offset calibration
float X_oVH;                                // Desiered angular velocity
float X_gVH;                                // For storing Gyro data
float X_gVHArray [num_samples_cycle];       // Put gyro data in array
float X_GCal;                               // Gyro data offset calibration
float X_eVH;                                // Angualar velocity error
float X_dKraft;                             // For storing thrust difference between propellers
float X_gVHmed;                             // Average angular velcoity
float X_gVmed;                              // Average angle
// Variables for PITCH
int Y_eV;                                   // Angle error (for pitch)
float Y_gV;                                 // For storing accelerometer data
float Y_gVArray [num_samples_cycle];        // Put accelerometer data in array
float Y_ACal;                               // Accelerometer data offset calibration
float Y_oVH;                                // Desiered angular velocity
float Y_gVH;                                // For storing Gyro data
float Y_gVHArray [num_samples_cycle];       // Put gyro data in array
float Y_GCal;                               // Gyro data offset calibration
float Y_eVH;                                // Angualar velocity error
float Y_dKraft;                             // For storing thrust difference between propellers
float Y_gVHmed;                             // Average angular velcoity
float Y_gVmed;                              // Average angle
 
String readString;                          // main captured String
String pitch;                               // data String
String roll;                                // data String
String throttle;                            // data String
 
int ind1;                                   // , locations
int ind2;                                   // , locations
int ind3;                                   // , locations
unsigned long timer;                        // Safety kill mechanism (when flying autonomous)
unsigned long timer2;                       // Safety kill mechanism (when flying autonomous)
char c;                                     // Loop through each character to find the correct pitch, roll and throttle value.

void getAccelGyro_Data();                   // Get IMU data

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  motor1.writeMicroseconds(0);              // Arm motors
  motor2.writeMicroseconds(0);              // Arm motors
  motor3.writeMicroseconds(0);              // Arm motors
  motor4.writeMicroseconds(0);              // Arm motors
  bd.begin(9600);                           // Initalize Bluetooth
  bd.setTimeout(100);                       // Define timeout in order to not lose data. Can be less than 100.
  Serial.begin(38400);                      
  // IMU Stuff start
  Wire.begin();
  accelgyro.initialize();                   // Initalize IMU
  // IMU Stuff end
  motor1.attach(6);                         // ESC pin
  motor2.attach(9);                         // ESC pin
  motor3.attach(3);                         // ESC pin
  motor4.attach(5);                         // ESC pin
  //Leds
  pinMode(ledPin, OUTPUT);                  // LED
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // Calibrate
  if (!calibrated) {                        // Check if sensor is calibrated
    motor1.writeMicroseconds(0);            // Arm motor
    motor2.writeMicroseconds(0);            // Arm motor
    motor3.writeMicroseconds(0);            // Arm motor
    motor4.writeMicroseconds(0);            // Arm motor
    Serial.println("Break");                
    delay(5000);                            // Wait a bit so IMU gives correct data
    Serial.println("Calibrating...");       // Some issues with the gyro was found the first seconds
    while (!calibrated) {                   // Start calibration
      getAccelGyro_Data();                  // Fetch IMU data
      // Read data from accelerometer
      X_gV = rollDeg;                       // Roll value acc
      Y_gV = pitchDeg;                      // Pitch value acc
      // Save all accelerometer data in a array
      X_ACal = X_ACal + X_gV;               // Add to sum
      Y_ACal = Y_ACal + Y_gV;               // Add to sum
      // Read data from gyro
      X_gVH = Gxyz[1];                      // Roll value gyro
      Y_gVH = Gxyz[0];                      // Pitch value gyro
      // Save all gyro data in a array
      X_GCal = X_GCal + X_gVH;              // Add to sum
      Y_GCal = Y_GCal + Y_gVH;              // Add to sum
 
      counter++;                            // Increment counter
 
      if (counter >= num_samples_cal) {     // If we've taken as many samples as specified, take average
        //calcualte average
        X_ACal = X_ACal / num_samples_cal;
        Y_ACal = Y_ACal / num_samples_cal;
        X_GCal = X_GCal / num_samples_cal;
        Y_GCal = Y_GCal / num_samples_cal;
 
        Serial.println("Finished Calibrating");
        calibrated = true;
        //TURN ON LED
        digitalWrite(ledPin, HIGH);         // It is calibrated
      }
    }
  }
  //ARM MOTORS
  if (first) {                              // If first, arm motors
    if (bd.available()) {
      char start = bd.read();
      if (start = '1') {                    // If received signal
        motor1.writeMicroseconds(0);
        motor2.writeMicroseconds(0);
        motor3.writeMicroseconds(0);
        motor4.writeMicroseconds(0);
        bd.write("r");                      // Confirm message
        delay(3000);
        Serial.println("Armed");
        first = false;
      }
    }
  }
  timer = millis();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//INNER LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (!first and calibrated) {         // Exit if time limit is exceeded.
    timer2 = millis();
    if (timer+10000 < timer2){
       thrust = 0;
       exit(0);
    }
    if (bd.available()) {                 // Check for new signals from ground station
      char c = bd.read();
      Serial.println(c);
      if (c == '*') {
        ind1 = readString.indexOf(',');                 //finds location of first,
        roll = readString.substring(0, ind1);           //captures first data String
        ind2 = readString.indexOf(',', ind1+1 );        //finds location of second,
        pitch = readString.substring(ind1+1, ind2);     //captures second data String
        ind3 = readString.indexOf(',', ind2+1 );
        throttle = readString.substring(ind2+1, ind3);  // ind3+1
        X_oV = roll.toInt();
        Y_oV = pitch.toInt();
        thrust = throttle.toInt();
       
        readString="";   //clears variable for new input
        pitch="";
        roll="";
        throttle="";
      }  
      else {    
        readString += c; //makes the string readString
      }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GET CLEAN DATA
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    getAccelGyro_Data();
    float   pitchDegFiltered = pitchDegFiltered *0.9813 +  0.01867 * pitchDeg;  // First order Lowpass filter, Fc at 20 Hz.
    float   rollDegFiltered = rollDegFiltered * 0.9813 +  0.01867 * rollDeg;    // First order Lowpass filter, Fc at 20 Hz.
    //Read data from accelerometer and gyro
    float x_gVprev = X_gV;
    X_gV = rollDegFiltered - X_ACal;                      // Roll - offset
    X_gV = 0.98 * X_gV + (0.02) * x_gVprev;               // exponentially weighed moving average filter. Try and fail with constant ,current_output  = α*current_input + (1-α)*previous_output
 
    float y_gVprev = Y_gV;
    Y_gV = pitchDegFiltered - Y_ACal;                     // Pitch - offset
    Y_gV = 0.98 * Y_gV + (0.02) * y_gVprev;               // exponentially weighed moving average. Try and fail with constant
    //Save all accelerometer data in a array
    X_gVArray[counter % num_samples_cycle] = X_gV;
    Y_gVArray[counter % num_samples_cycle] = Y_gV;
 
    X_gVH = Gxyz[1];   // Acc x
    Y_gVH = Gxyz[0];   // acc y
 
    //limit extreme speed
    if (X_gVH > maxGyroVal) {
      X_gVH = maxGyroVal;
    }
    else if (X_gVH < minGyroVal) {
      X_gVH = minGyroVal;
    }
 
    if (Y_gVH > maxGyroVal) {
      Y_gVH = maxGyroVal;
    }
    else if (Y_gVH < minGyroVal) {
      Y_gVH = minGyroVal;
    }
    //Save all gyro data in a different array
    X_gVHArray[counter % num_samples_cycle] = X_gVH - X_GCal; //Actual X - offset
    Y_gVHArray[counter % num_samples_cycle] = Y_gVH - Y_GCal; //Actual Y - offset
 
    counter++;
 
    if (counter % num_samples_cycle == 0) {
      //Compute for every x readings
      //Sort arrays - this might take too much time
      if (sort) {
        int temp;
        for (int i = 0; i < num_samples_cycle - 1; i++) {
          for (int j = 0; j < num_samples_cycle - 1; j++) {
            //Swapping element in if statement-
            if (X_gVArray[j] > X_gVArray[j + 1]) {
              temp = X_gVArray[j];
              X_gVArray[j] = X_gVArray[j + 1];
              X_gVArray[j + 1] = temp;
            }
            if (Y_gVArray[j] > Y_gVArray[j + 1]) {
              temp = Y_gVArray[j];
              Y_gVArray[j] = Y_gVArray[j + 1];
              Y_gVArray[j + 1] = temp;
            }
            if (X_gVHArray[j] > X_gVHArray[j + 1]) {
              temp = X_gVHArray[j];
              X_gVHArray[j] = X_gVHArray[j + 1];
              X_gVHArray[j + 1] = temp;
            }
            if (Y_gVHArray[j] > Y_gVHArray[j + 1]) {
              temp = Y_gVHArray[j];
              Y_gVHArray[j] = Y_gVHArray[j + 1];
              Y_gVHArray[j + 1] = temp;
            }
          }
        }
 
        //Add remaining values
        for (int i = remove_num_spikes; i < (num_samples_cycle - remove_num_spikes); i++) {
          X_gVmed = X_gVmed + X_gVArray[i];
          Y_gVmed = Y_gVmed + Y_gVArray[i];
          //Find average angular velocity
          X_gVHmed = X_gVHmed + X_gVHArray[i];
          Y_gVHmed = Y_gVHmed + Y_gVHArray[i];
        }
 
        //Find average angle
        X_gVmed = X_gVmed / (num_samples_cycle - remove_num_spikes * 2);
        Y_gVmed = Y_gVmed / (num_samples_cycle - remove_num_spikes * 2);
        //Find average angular velocity
        X_gVHmed = X_gVHmed / (num_samples_cycle - remove_num_spikes * 2);
        Y_gVHmed = Y_gVHmed / (num_samples_cycle - remove_num_spikes * 2);
      }
      else {
        for (int i = 0; i < num_samples_cycle; i++) {
          X_gVmed = X_gVmed + X_gVArray[i];
          Y_gVmed = Y_gVmed + Y_gVArray[i];
          //Find average angular velocity
          X_gVHmed = X_gVHmed + X_gVHArray[i];
          Y_gVHmed = Y_gVHmed + Y_gVHArray[i];
        }
 
        //Find average angle
        X_gVmed = X_gVmed / num_samples_cycle;
        Y_gVmed = Y_gVmed / num_samples_cycle;
        //Find average angular velocity
        X_gVHmed = X_gVHmed / num_samples_cycle;
        Y_gVHmed = Y_gVHmed / num_samples_cycle;
      }
      
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CONTROL SYSTEM
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEBUG
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (counter % num_samples_print == 0) {
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
      Serial.print(" ");
      Serial.print(X_ACal);
      Serial.print(" ");
      Serial.println(Y_ACal);
      counter = 0;
    }
  }
}
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU FUNCTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void getAccelGyro_Data(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Accel
  Axyz[0] = ((double) ax / 256) -  6;
  Axyz[1] = ((double) ay / 256) - 4;
  Axyz[2] = ((double) az / 256);
 
  // Gyro
  Gxyz[0] = (double) gx * 250 / 16384;
  Gxyz[1] = (double) gx * 250 / 16384;
  Gxyz[2] = (double) gz * 250 / 32768;
 
  pitchDeg = 180 * (atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]))) / PI;  // degrees
  rollDeg =  180 * (atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2]))) / PI;  // degrees
}
