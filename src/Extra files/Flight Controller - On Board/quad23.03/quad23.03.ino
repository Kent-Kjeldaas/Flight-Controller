//#include <I2Cdev.h>
#include <Servo.h>
#include <Wire.h>
// IMU stuff start
#include "I2Cdev.h"
#include "MPU9250.h"
MPU9250 accelgyro;
I2Cdev   I2C_M;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
float Gxyz[3];
float pitchDeg;
float rollDeg;
// IMU Stuff end
char c;
// Settings
float kp1 = 0.8;    // Set P-term to tune desired angular velocity
float kp2 = 1;    // Set P-term to tune difference between m1 and m2.
int thrust = 1390;  // Set thrust
int maxVal = 1590;  // Max thrust
int minVal = 0;  // Min thrust
int X_oV = 0;       // Set desired angle for ROLL
int Y_oV = 0;       // Set desired angle for PITCH
// Sample Settings
const int num_samples_cal = 2000; // Number of samples in calibration
const int num_samples_cycle = 20; // Number of samples per cycle
const int num_samples_print = 40;
// Set a limit for max deg/sec gyro meassurement
const int maxGyroVal = 50;
const int minGyroVal = -50;
// Sort settings
bool sort = false; // Do we want to remove x lowest and x highest values?
int remove_num_spikes = 5;
 
// Servo
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
 
// Define Variables
int m1= 0 ; // motor 1-3 = ROLL
int m2 = 0;
int m3 = 0; // motor 2-4 = PITCH
int m4= 0;
int counter = 0;
int ledPin = 13;
int number = 0;
 
bool first = true; // Is this the first time looping?
bool calibrated = false; // Are sensored calibrated?
// Variables for ROLL
int X_eV;             // Angle error (for roll)
float X_gV;           // For storing accelerometer data
float X_gVArray [num_samples_cycle];  // Put accelerometer data in array
float X_ACal;         // Accelerometer data offset calibration
float X_oVH;          // Desiered angular velocity
float X_gVH;          // For storing Gyro data
float X_gVHArray [num_samples_cycle]; // Put gyro data in array
float X_GCal;         // Gyro data offset calibration
float X_eVH;          // Angualar velocity error
float X_dKraft;       // For storing thrust difference between propellers
float X_gVHmed;       // Average angular velcoity
float X_gVmed;        // Average angle
// Variables for PITCH
int Y_eV;             // Angle error (for pitch)
float Y_gV;
float Y_gVArray [num_samples_cycle];
float Y_ACal;
float Y_oVH;
float Y_gVH;
float Y_gVHArray [num_samples_cycle];
float Y_GCal;
float Y_eVH;
float Y_dKraft;
float Y_gVHmed;
float Y_gVmed;
 
String readString; //main captured String
String pitch; //data String
String roll;
String throttle;
 
int ind1; // , locations
int ind2;
int ind3;
unsigned long timer;
unsigned long timer2;
 
void getAccelGyro_Data();
 
void setup()
{
  motor1.writeMicroseconds(0);
  motor2.writeMicroseconds(0);
  motor3.writeMicroseconds(0);
  motor4.writeMicroseconds(0);
  Serial.begin(38400);
  // IMU Stuff start
  Wire.begin();
  accelgyro.initialize();
  // IMU Stuff end
  motor1.attach(6); // ESC pin
  motor2.attach(9); // ESC pin
  motor3.attach(3); // ESC pin
  motor4.attach(5); // ESC pin
  //Leds
  pinMode(ledPin, OUTPUT);
}
 
void loop() {
  // Calibrate
  if (!calibrated) {
    motor1.writeMicroseconds(0);
    motor2.writeMicroseconds(0);
    motor3.writeMicroseconds(0);
    motor4.writeMicroseconds(0);
    Serial.println("Break");
    delay(3000);
    Serial.println("Calibrating...");
    while (!calibrated) {
      // IMU Stuff start
      getAccelGyro_Data();
      // IMU Stuff end
      // Read data from accelerometer
      X_gV = rollDeg; // roll
      Y_gV = pitchDeg;  // Pitch
      // Save all accelerometer data in a array
      X_ACal = X_ACal + X_gV;
      Y_ACal = Y_ACal + Y_gV;
      // Read data from gyro
      X_gVH = Gxyz[1];   // Acc x
      Y_gVH = Gxyz[0];   // Acc y
      // Save all gyro data in a array
      X_GCal = X_GCal + X_gVH;
      Y_GCal = Y_GCal + Y_gVH;
 
      counter++;
 
      if (counter >= num_samples_cal) {
        //calcualte average
        X_ACal = X_ACal / num_samples_cal;
        Y_ACal = Y_ACal / num_samples_cal;
        X_GCal = X_GCal / num_samples_cal;
        Y_GCal = Y_GCal / num_samples_cal;
 
        Serial.println("Finished Calibrating");
        calibrated = true;
        //TURN ON LED
        digitalWrite(ledPin, HIGH); //It's calibrated
 
        motor1.writeMicroseconds(1000);
        motor2.writeMicroseconds(1000);
        motor3.writeMicroseconds(1000);
        motor4.writeMicroseconds(1000);
        delay(3000);
      }
    }
  }
  //ARM MOTORS
  if (first) {
        motor1.writeMicroseconds(0);
        motor2.writeMicroseconds(0);
        motor3.writeMicroseconds(0);
        motor4.writeMicroseconds(0);
        Serial.println("Armed");
        first = false;
      }
  while (!first and calibrated) {
    // IMU STUFF START
    getAccelGyro_Data();
    float   pitchDegFiltered = pitchDegFiltered *0.9813 +  0.01867 * pitchDeg; // First order Lowpass filter, Fc at 20 Hz.
    float   rollDegFiltered = rollDegFiltered * 0.9813 +  0.01867 * rollDeg;  // First order Lowpass filter, Fc at 20 Hz.
    //Read data from accelerometer and gyro
    float x_gVprev = X_gV;
    X_gV = rollDegFiltered - X_ACal;   // Roll - offset
    X_gV = 0.98 * X_gV + (0.02) * x_gVprev; // exponentially weighed moving average filter. Try and fail with constant ,current_output  = α*current_input + (1-α)*previous_output
 
    float y_gVprev = Y_gV;
    Y_gV = pitchDegFiltered - Y_ACal;  // Pitch - offset
    Y_gV = 0.98 * Y_gV + (0.02) * y_gVprev; // exponentially weighed moving average. Try and fail with constant
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
 
    }
      //Print pwm signals for motors, acc and gyro data for every 10 loop
      Serial.print(number);
      Serial.print(" ");
      number++;
      Serial.print(X_gVHmed);
      Serial.print(" ");
      Serial.println(Y_gVHmed);
  }
}
 
// IMU FUNCTIONS
void getAccelGyro_Data(void)
{
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Accel
  Axyz[0] = ((double) ax / 256) -  6;
  Axyz[1] = ((double) ay / 256) - 4;
  Axyz[2] = ((double) az / 256);
 
  // Gyro
  Gxyz[0] = (double) gx * 250 / 16384;
  Gxyz[1] = (double) gy * 250 / 16384;
  Gxyz[2] = (double) gz * 250 / 32768;
 
  pitchDeg = 180 * (atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]))) / PI; // degrees
  rollDeg =  180 * (atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2]))) / PI;  // degrees
}
