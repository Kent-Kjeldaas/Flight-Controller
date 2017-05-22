#include <Servo.h>
#include <Wire.h>
#include <Adafruit_L3GD20.h>
#include <MsTimer2.h>
 
//servo
Servo motor1; //left
Servo motor2; //right
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
const int xPin   = A1;
int counter = 0;
int counter2 = 0;
boolean first = true;

//Settings
float kp1 = 0.6;
float kp2 = 0.6;
int thrust = 1400;
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
  MsTimer2::set(50, funk); // 10ms period
  MsTimer2::start();
}
 
void loop() {
  if (counter >= 4) {
 
    //Compute
    gVmed = (gVArray[0] + gVArray[1] + gVArray[2] + gVArray[3] + gVArray[4]) / 5;
    gVHmed = (gVHArray[0] + gVHArray[1] + gVHArray[2] + gVHArray[3] + gVHArray[4]) / 5;
    eV = oV - gVmed;
    oVH = eV * kp1;
    eVH = oVH - gVHmed;
    dKraft = eVH * kp2;
    m1 = thrust + dKraft;
    m2 = thrust - dKraft;
 
    //Send PWM signals
    motor1.writeMicroseconds(m1);
    motor2.writeMicroseconds(m2);
    counter2++;
  }
 
  //Print pwm signals for motors, acc and gyro data
  Serial.print(m1);
  Serial.print(" - ");
  Serial.print(m2);
  Serial.print(" - ");
  Serial.print(gV);
  Serial.print(" - ");
  Serial.print(gVH);
  Serial.println(" ");
  counter2 = 0;
}
 
void funk() {
  //Read data
  Serial.println("Started");
  gV = constrain(map(analogRead(xPin), 349, 281, 0, 180), -90, 90);
  //lagre all gyro data i et array?
  gVArray[counter] = gV;
  gyro.read();
  gVH = (int)abs(gyro.data.x);//get from acc
  //lagre all acc data i et annet array
  gVHArray[counter] = gVH;
  counter++;
  Serial.println("Ended");
}
