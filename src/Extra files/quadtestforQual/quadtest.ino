#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
 
#include <I2Cdev.h>                         // GYro
#include <MPU6050_6Axis_MotionApps20.h>     // GYro

//servo
//Servo myservo;
Servo motor1;
Servo motor2;
//variables
int m1;
int m2;
int engineRPM = 1300;
int x;
 
void accelgyroData();
 
// Specific I2C addresses may be passed as a parameter here
MPU6050 mpu;              // Default: AD0 low = 0x68
// MPU Control/Status
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
bool dmpReady = false;          // Set true if DMP init was successful
uint8_t devStatus;              // Return status after device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;           // Holds actual interrupt status byte from MPU
uint16_t packetSize;            // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;             // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64];         // FIFO storage buffer
// Orientation/Motion
// -- -- -- -- -- -- -- -- -- -- -- -- -- --
Quaternion q;                   // [w, x, y, z]       Quaternion Container
VectorFloat gravity;            // [x, y, z]            Gravity Vector
int16_t gyro[3];                // [x, y, z]            Gyro Vector
float ypr[3];                   // [yaw, pitch, roll]   Yaw/Pitch/Roll & gravity vector
float averagepitch[50];         // Used for averaging pitch value
 
 
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
boolean start = false;

 
//Need to add pins to accelerometer
 
//Might need to add the start sequence for the propeller
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 5, 2, 1, DIRECT); //Might need to change values
 
void setup()
{
  Serial.begin(115200);
  motor1.attach(11); // ESC pin
  motor2.attach(9); // ESC pin
 
  ////////// GYRO////////////
  // Initialize MPU6050
  mpu.initialize();
  Serial.println("Testing MPU connection:");
  printf("Starting");
  Serial.println(mpu.testConnection() ? "> MPU6050 connection successful" : "> MPU6050 connection failed");
  Serial.println("Initialising DMP");
  devStatus = mpu.dmpInitialize();
  // OFFSETS
  mpu.setZGyroOffset(30);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Serial.println("Enabling DMP");
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
 
    // Set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP Ready! Let's Proceed.");
    Serial.println("Propeller is ready, get away from propeller");
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // In case of an error with the DMP
    if (devStatus == 1) Serial.println("> Initial Memory Load Failed");
    else if (devStatus == 2) Serial.println("> DMP Configuration Updates Failed");
  }
  ////////////////////////////////
 
  myPID.SetOutputLimits(1100, 1900); //PID RPM limits
  Setpoint = 10; //Wanted angle
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
  // Do we need this? : Wire.onReceive(receiveEvent);
}
 
void loop() {
  for (int i = 1000; i <  1410; i += 5) {
    motor1.writeMicroseconds(i);
    motor2.writeMicroseconds(i);
    Serial.println(i);
    delay(25);
    while (i == 1405) {
      //Setpoint = Serial.read();
      
      //accelgyroData();
      myPID.Compute();
      x = Output - engineRPM; //=36

      //gitt at vinkel positiv
      if(Input <0){ 
        m1 = x + m1; //= 1436
        m2 = x - m2; //= 1364
      }
      // if negativ
      if(Input >0){
        m1 = x - m1; //= 1436
        m2 = x + m2; //= 1364
      }

      motor1.writeMicroseconds(m1);
      motor2.writeMicroseconds(m2);
      // myservo.writeMicroseconds(Output);
      Serial.print(m1);
      Serial.print(" - ");
      Serial.print(m2);
      Serial.print(" - ");
      Serial.println(pitch());
    }
  }
}

float pitch() {
  return (ypr[1] * 180 / M_PI);
}

void accelgyroData() {
 
  // Reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();
 
  // Get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // Check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // Reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println("Warning - FIFO Overflowing!");
 
    // otherwise, check for DMP data ready interrupt (this should happen exactly once per loop: 100Hz)
  } else if (mpuIntStatus & 0x02) {
    // Wait for correct available data length, should be less than 1-2ms, if any!
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
 
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
 
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    // Get sensor data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.resetFIFO();
 
    //Serial.println(pitch());
    Serial.print(pitch());
    Serial.print(" - ");
    //Serial.println(angRate());
    Input = pitch();
  }
}
