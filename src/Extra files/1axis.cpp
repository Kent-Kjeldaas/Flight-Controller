#include <Servo.h>
#include <Wire.h>
#include <Adafruit_L3GD20.h>
 
//servo
Servo motor1;
Servo motor2;
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
 
 
//Other variables
int m1;
int m2;
int eV;
float gV;
float gVArray [5];
float oVH;
float gVH;
float gVHArray [5];
float eVH;
float dKraft;
float gVHmed;
float gVmed;
const int xPin = A1;
int counter =0;
bool first = true;
 
//Settings
float kp1 = 0.6; //to limit desired angular velocity
float kp2 = 0.6; //to limit difference between m1 and m2
int thrust = 1400;
int minVal = 1200;
int maxVal = 1600;
int oV = 0;
 
 
void setup()
{
  Serial.begin(250000);
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
 
  motor1.attach(4); // ESC pin
  motor2.attach(2); // ESC pin
}
 
void loop() {
  //init
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
  while(true){
    //Read data
    gV = constrain(map(analogRead(xPin), 349, 281, 0, 180), -90, 90); //gives a value from -90 to 90 depending on angle
    //lagre all acc data i et array
    gVArray[counter%5] = gV;
    gyro.read();
    gVH = (int)abs(gyro.data.x);
    //lagre all gyro data i et annet array
    if(gVH > 50){ //for extreme speeds, limit it.
      gVH = 50;
    }
    gVHArray[counter%5] = gVH;
    counter++;
   
    if (counter%5 == 0) {
      //Compute
      gVmed = (gVArray[0] + gVArray[1] + gVArray[2] + gVArray[3] + gVArray[4]) / 5;
      gVHmed = (gVHArray[0] + gVHArray[1] + gVHArray[2] + gVHArray[3] + gVHArray[4]) / 5;
      eV = oV - gVmed;
      oVH = eV * kp1;
      if(oVH >= 0){
        eVH = oVH - gVHmed;
      }
      else{
        eVH = oVH + gVHmed;
      }
      dKraft = eVH * kp2;
      m1 = thrust + dKraft;
      m2 = thrust - dKraft;
      
      //LIMIT M1
      if(m1 > maxVal){
        m1 = maxVal; 
      }
      else if(m1 < minVal){
        m1 = minVal;
      }
      //LIMIT M2
      if(m2 > maxVal){
        m2 = maxVal;
      }
      else if(m2 < minVal) {
        m2 = minVal;
      }
   
      //Send PWM signals
      motor1.writeMicroseconds(m1);
      motor2.writeMicroseconds(m2);
   
    }
    if (counter%20 == 0){
      //Print pwm signals for motors, acc and gyro data
      Serial.print(m1);
      Serial.print(" - ");
      Serial.print(m2);
      Serial.print(" - ");
      Serial.print(gVmed);
      Serial.print(" - ");
      Serial.print(gVHmed);
      Serial.println("  ");
    }
  }
}