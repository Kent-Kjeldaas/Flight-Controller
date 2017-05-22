#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_L3GD20.h>
 
//communication
SoftwareSerial bluetooth(10 ,11); //RX, TX, DOUBLE CHECK 
//servo
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

#define USE_I2C
#ifdef USE_I2C
// The default constructor uses I2C
Adafruit_L3GD20 gyro;
#else
// To use SPI, you have to define the pins
#define GYRO_CS 4 // labeled CS
#define GYRO_DO 5 // labeled SA0
#define GYRO_DI 6  // labeled SDA
#define GYRO_CLK 7 // labeled SCL
Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
#endif 


//DEFINE VARIABLES
int m1; //motor 1-2 = ROLL
int m2;
int m3; //motor 3-4 = PITCH
int m4;
int counter =0; 
const int xPin = A1;
const int yPin = A3; 
bool first = true; //is this the first time looping?
bool contact = false; //have we established contact yet?
String readString; //main captured String
String pitch; //data String
String roll;
String sigThrottle;
int ind1; // , locations
int ind2;
int ind3;
//Variables for ROLL
int X_eV;             // Angle error (for roll)
float X_gV;           // For storing accelerometer data
float X_gVArray [5];  // Put accelerometer data in array 
float X_oVH;          // Desiered angular velocity
float X_gVH;          // For storing Gyro data 
float X_gVHArray [5]; // Put gyro data in array
float X_eVH;          // Angualar velocity error
float X_dKraft;       // For storing thrust difference between propellers
float X_gVHmed;       // Average angular velcoity
float X_gVmed;        // Average angle
//Variables for PITCH
int Y_eV;             // Angle error (for pitch)
float Y_gV;
float Y_gVArray [5];
float Y_oVH;
float Y_gVH;
float Y_gVHArray [5];
float Y_eVH;
float Y_dKraft;
float Y_gVHmed;
float Y_gVmed;
 
//Settings
float kp1 = 0.6;    //Set P-term to limit desired angular velocity
float kp2 = 0.6;    //Set P-term to limit difference between m1 and m2
int thrust = 1400;  //Set thrust
int maxVal = 1650;
int minVal = 1250;
int X_oV = 0;       //Set desired angle for ROLL
int Y_oV = 0;       //Set desired angle for PITCH


void setup()
{
  Serial.begin(250000);

  bluetooth.begin(250000); //can we set this to 250k? 
  bluetooth.setTimeout(50); //50ms
  // Try to initialise and warn if we couldn't detect the chip
  if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
    //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
    //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }
  // pin A0 (pin14) is VCC and pin A4 (pin18) in GND to activate the GY-61-module
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(14, HIGH);
  digitalWrite(18, LOW);

  motor1.attach(3); // ESC pin
  motor2.attach(5); // ESC pin
  motor3.attach(6); // ESC pin !!!!WILL PROBABLY NEED TO CHANGE!!!
  motor4.attach(10); // ESC pin      <-- !!!!HERE TO!!!!
}

void loop() {
  if(bluetooth.available()){
    if(!contact){ //on receive?
      //receive:
      String val = bluetooth.readString();
      if (val = "init"){
        //init stuff here
        // ...
        //send:
        bluetooth.write("r");
        Serial.print("Contact Achieved");
        contact = true;
      }
      else{
      Serial.print("Did not recieve signal");
      }
    }
    else{
    //add starting frequence
    //init (current start freq)
      if(first){
          for (int i = 1000; i <  1700; i += 5) {
            motor1.writeMicroseconds(i);
            motor2.writeMicroseconds(i);
            Serial.println(i);
            delay(25);
          }
          for (int j = 1700; j > 1390 ; j -= 5) {
            motor1.writeMicroseconds(j);
            motor2.writeMicroseconds(j);
            Serial.println(j);
            delay(25);
          }
          first = false;
        }
        else{
          while(true){
            //Read data for pitch, roll and throttle
            if(bluetooth.available()){
              char c = bluetooth.read();  //gets one byte from serial buffer
              if (c == '*') {
                //do stuff
                ind1 = readString.indexOf(',');  //finds location of first ,
                pitch = readString.substring(0, ind1);   //captures first data String
                ind2 = readString.indexOf(',', ind1+1 );   //finds location of second ,
                roll = readString.substring(ind1+1, ind2);   //captures second data String
                ind3 = readString.indexOf(',', ind2+1 );
                sigThrottle = readString.substring(ind2+1); // ind3+1
               
                Y_oV = pitch.toInt(); // Casts the string back to int or float, whatever we choose to use.
                X_oV = roll.toInt();
                thrust = sigThrottle.toInt();
               
				Serial.print(pitch);
				Serial.print(roll);
				Serial.print(sigThrottle);
			   
                readString=""; //clears variable for new input
                pitch="";
                roll="";
                sigThrottle="";
              }  
              else {    
                readString += c; //makes the string readString
              }

              //Read data from accelerometer and gyro
              X_gV = constrain(map(analogRead(xPin), 349, 281, 0, 180), -90, 90); //gives a value from -90 to 90 depending on angle
              Y_gV = constrain(map(analogRead(yPin), 349, 281, 0, 180), -90, 90); //double check this
              //Save all accelerometer data in a array
              X_gVArray[counter%5] = X_gV;
              Y_gVArray[counter%5] = Y_gV;
              gyro.read();
              X_gVH = (int)abs(gyro.data.x);
              Y_gVH = (int)abs(gyro.data.y);
              //Save all gyro data in a different array
              X_gVHArray[counter%5] = X_gVH;
              Y_gVHArray[counter%5] = Y_gVH;
              counter++;
             
              if (counter%5 == 0) {
                //Compute for every five readings
                //Find average angle
                X_gVmed = (X_gVArray[0] + X_gVArray[1] + X_gVArray[2] + X_gVArray[3] + X_gVArray[4]) / 5;
                Y_gVmed = (Y_gVArray[0] + Y_gVArray[1] + Y_gVArray[2] + Y_gVArray[3] + Y_gVArray[4]) / 5;
                //Find avarage angular velocity
                X_gVHmed = (X_gVHArray[0] + X_gVHArray[1] + X_gVHArray[2] + X_gVHArray[3] + X_gVHArray[4]) / 5;
                Y_gVHmed = (Y_gVHArray[0] + Y_gVHArray[1] + Y_gVHArray[2] + Y_gVHArray[3] + Y_gVHArray[4]) / 5;
                //Error angle
                X_eV = X_oV - X_gVmed;
                Y_eV = Y_oV - Y_gVmed;
                //Set desired angular velocity
                X_oVH = X_eV * kp1;
                Y_oVH = Y_eV * kp1;

                //To avoid sign error: 
                if(X_oVH >= 0){
                  X_eVH = X_oVH - X_gVHmed;
                }
                else{
                  X_eVH = X_oVH + X_gVHmed;
                }
                if(Y_oVH >= 0){
                  Y_eVH = Y_oVH - Y_gVHmed;
                }
                else{
                  Y_eVH = Y_oVH + Y_gVHmed;
                }

                //Thrust differential
                X_dKraft = X_eVH * kp2;
                Y_dKraft = Y_eVH * kp2;
                //Final thrust
                m1 = thrust + X_dKraft;
                m2 = thrust - X_dKraft;
                m3 = thrust + Y_dKraft;
                m4 = thrust - Y_dKraft;

                //Check if m1 exceeds the limit
                if(m1 > maxVal){
                  m1 = maxVal; 
                }
                else if(m1 < minVal){
                  m1 = minVal;
                }
                //Check if m2 exceeds the limit
                if(m2 > maxVal){
                  m2 = maxVal;
                }
                else if(m2 < minVal) {
                  m2 = minVal;
                }
             
                //Send PWM signals
                motor1.writeMicroseconds(m1);
                motor2.writeMicroseconds(m2);
                motor3.writeMicroseconds(m3);
                motor4.writeMicroseconds(m4);
             
              }
              if (counter%20 == 0){
                //Print pwm signals for motors, acc and gyro data for every 20 loop
                Serial.print("X : ");
                Serial.print(m1);
                Serial.print(" - ");
                Serial.print(m2);
                Serial.print(" - ");
                Serial.print(X_gV);
                Serial.print(" - ");
                Serial.print(X_gVH);
                Serial.println(" ");
                Serial.print("Y : ");
                Serial.print(m3);
                Serial.print(" - ");
                Serial.print(m4);
                Serial.print(" - ");
                Serial.print(Y_gV);
                Serial.print(" - ");
                Serial.print(Y_gVH);
                Serial.println(" ");
              }
            }
			else{
                motor1.writeMicroseconds(0);
                motor2.writeMicroseconds(0);
                motor3.writeMicroseconds(0);
                motor4.writeMicroseconds(0);
			}
          }
        }
      }
    }
  }
