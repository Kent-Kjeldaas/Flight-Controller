///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM
#include <Servo.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4, servo_1, servo_2, servo_3, servo_4; //Added Servo
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis[4], gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float roll_signal, pitch_signal, yaw_signal;

/////////////////////////////
// Legger inn Calibration values:
//////////////////////////////
int s1 = 1500;
int s2 = 1400;
int s3 = 1500;
int s4 = 1560;

int s_inc = 400;
int s1_max, s2_max, s3_max, s4_max; 

unsigned long tid, tid2;
char data_in;
bool cal = false;
int servo_value;

int number = 0;
bool debug = true;

void calibrate_servos();

// Servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const float h[] = { 0.0151963734917116, 0.125956465856097,  0.358847160652191,  0.358847160652191,  0.125956465856097,  0.0151963734917116};
 
#define M (sizeof(h)/sizeof(float))        // Defining length of shift register
float x[M] = {0};                          // Setting all values in shiftregister B as zero value
int N = M-1;                               // Filter Order
float pitch_angle_lowpass =0;              // Output variable for y[n]

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  pinMode(A3, OUTPUT);
  
  Serial.begin(57600);
  Serial.println("starting");
  //Read EEPROM for fast access data.
  for (start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  gyro_address = eeprom_data[32];                              //Store the gyro address in the variable.

  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(12);
  servo4.attach(13);

  Wire.begin();                                                //Start the I2C as master.

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                           //Configure digital poort 4, 5, 6 and 7 as output.

  //Use the led on the Arduino for startup indication.
  //digitalWrite(A2, HIGH);                                      //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed
  while (eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  set_gyro_registers();                                        //Set the specific gyro registers.

  for (cal_int = 0; cal_int < 1250 ; cal_int ++) {             //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                   //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++) {             //Take 2000 readings for calibration.
    if (cal_int % 15 == 0)digitalWrite(A2, !digitalRead(A2));  //Change the led status to indicate calibration.
    gyro_signalen();                                           //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                          //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                          //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                          //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                    //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                    //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                    //Divide the yaw total by 2000.

  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while (receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400) {
    receiver_input_channel_3 = convert_receiver_channel(3);    //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    receiver_input_channel_4 = convert_receiver_channel(4);    //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                  //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                        //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                  //Wait 3 milliseconds before the next loop.
    if (start == 125) {                                        //Every 125 loops (500ms).
      digitalWrite(A2, !digitalRead(A2));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  start = 0;                                                   //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  
  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);
  
  calibrate_servos();
  
  s1_max = s1 + s_inc;
  s2_max = s2 - s_inc;
  s3_max = s3 + s_inc;
  s4_max = s4 - s_inc;
  
  
  servo1.write(s1);
  servo2.write(s2);
  servo3.write(s3);
  servo4.write(s4);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  receiver_input_channel_1 = convert_receiver_channel(1);      //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = convert_receiver_channel(2);      //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = convert_receiver_channel(3);      //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = convert_receiver_channel(4);      //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

  //Reset values
  roll_signal = 0;
  pitch_signal = 0;
  yaw_signal = 0;

  //Need to take an average of three samples to reduce noise.
  for(int i = 0; i < 3; i++){
    gyro_signalen();
    roll_signal += gyro_roll_input;
    pitch_signal += gyro_pitch_input;
    yaw_signal += gyro_yaw_input;
  }

  //Avg last three samples
  gyro_roll_input = roll_signal/3;
  gyro_pitch_input = pitch_signal/3;
  gyro_yaw_input = yaw_signal/3;
  
  //Let's get the current gyro data and scale it to degrees per second for the pid calculations.
  gyro_roll_input = (gyro_roll_input * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               //Gyro pid input is deg/sec.

  /*
  /////////////////////////////
  //NEW FILTER TEST
  /////////////////////////////
  int i = 0;
  for(i=N; i>0; i--) {    // Shifts all previous samples one position
    x[i] = x[i-1];        // Makes room for one new sample
  }
 
   x[0] = gyro_pitch_input;
   pitch_angle_lowpass = h[0] * x[0];   // Compute the convolution x[0]*h[0]7
 
   for(i = 1; i <= N; i++) {            // Summing the rest of the products
    pitch_angle_lowpass += h[i] * x[i]; // Convolve rest of the inputs with the filter coefficients
   }
   */
  //For starting the motors: throttle low and yaw left (step 1).
  if (receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450) {
    start = 2;
    //Reset the pid controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_1 > 1508)pid_roll_setpoint = (receiver_input_channel_1 - 1508) / 3.0;
  else if (receiver_input_channel_1 < 1492)pid_roll_setpoint = (receiver_input_channel_1 - 1492) / 3.0;

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_2 > 1508)pid_pitch_setpoint = (receiver_input_channel_2 - 1508) / 3.0;
  else if (receiver_input_channel_2 < 1492)pid_pitch_setpoint = (receiver_input_channel_2 - 1492) / 3.0;

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (receiver_input_channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508) / 3.0;
    else if (receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492) / 3.0;
  }
  //PID inputs are known. So we can calculate the pid output.
  calculate_pid();

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if (battery_voltage < 1030 && battery_voltage > 600)digitalWrite(A2, HIGH);

  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.

  if (start == 2) {                                                         //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.

    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800) {                  //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage) / (float)3500);            //Compensate the esc-4 pulse for voltage drop.
    }

    servo_1 = map(esc_1, 1000, 2000, s1, s1_max);
    servo_2 = map(esc_2, 1000, 2000, s2, s2_max);
    servo_3 = map(esc_3, 1000, 2000, s3, s3_max);
    servo_4 = map(esc_4, 1000, 2000, s4, s4_max); //what is the middle value 1500 throttle? (0 deg -> +7 deg, find the values)

    //send servo val <--------------------------------------------------------------------------------------------------------------------<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    servo1.writeMicroseconds(servo_1);
    servo2.writeMicroseconds(servo_2);
    servo3.writeMicroseconds(servo_3);
    servo4.writeMicroseconds(servo_4);

    if(debug) {
      Serial.print(number);
      number++;
      Serial.print(" ");
      Serial.print(throttle);
      Serial.print(" ");
      Serial.print(gyro_pitch_input );
      Serial.print(" ");
      Serial.print(gyro_roll_input);
      Serial.print(" ");
      Serial.println(gyro_yaw_input);
    }

    if (esc_1 < 1200) esc_1 = 1200;                                         //Keep the motors running.
    if (esc_2 < 1200) esc_2 = 1200;                                         //Keep the motors running.
    if (esc_3 < 1200) esc_3 = 1200;                                         //Keep the motors running.
    if (esc_4 < 1200) esc_4 = 1200;                                         //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                          //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                          //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                          //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                          //Limit the esc-4 pulse to 2000us.
  }

  else {
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
    servo1.writeMicroseconds(s1);
    servo2.writeMicroseconds(s2);
    servo3.writeMicroseconds(s3);
    servo4.writeMicroseconds(s4);
  }

  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while (micros() - loop_timer < 4000);                                     //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  while (PORTD >= 16) {                                                     //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if (timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if (timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if (timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if (timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect) {
  current_time = micros();
  //Channel 1=========================================
  if (PINB & B00000001) {                                      //Is input 8 high?
    if (last_channel_1 == 0) {                                 //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if (last_channel_1 == 1) {                              //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input[1] = current_time - timer_1;                //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if (PINB & B00000010 ) {                                     //Is input 9 high?
    if (last_channel_2 == 0) {                                 //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if (last_channel_2 == 1) {                              //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input[2] = current_time - timer_2;                //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if (PINB & B00000100 ) {                                     //Is input 10 high?
    if (last_channel_3 == 0) {                                 //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if (last_channel_3 == 1) {                              //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input[3] = current_time - timer_3;                //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if (PINB & B00001000 ) {                                     //Is input 11 high?
    if (last_channel_4 == 0) {                                 //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if (last_channel_4 == 1) {                              //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input[4] = current_time - timer_4;                //Channel 4 is current_time - timer_4
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading the gyro
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen() {
  //Read the L3G4200D or L3GD20H
  if (eeprom_data[31] == 2 || eeprom_data[31] == 3) {
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro (adress 1101001)
    Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 6);                           //Request 6 bytes from the gyro
    while (Wire.available() < 6);                                //Wait until the 6 bytes are received
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_axis[1] = ((highByte << 8) | lowByte);                  //Multiply highByte by 256 (shift left by 8) and ad lowByte
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_axis[2] = ((highByte << 8) | lowByte);                  //Multiply highByte by 256 (shift left by 8) and ad lowByte
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_axis[3] = ((highByte << 8) | lowByte);                  //Multiply highByte by 256 (shift left by 8) and ad lowByte
  }

  //Read the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 6);                           //Request 6 bytes from the gyro
    while (Wire.available() < 6);                                //Wait until the 6 bytes are received
    gyro_axis[1] = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data
    gyro_axis[2] = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data
    gyro_axis[3] = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data
  }

  if (cal_int == 2000) {
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];
  if (eeprom_data[28] & 0b10000000)gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];
  if (eeprom_data[29] & 0b10000000)gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];
  if (eeprom_data[30] & 0b10000000)gyro_yaw *= -1;



}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//www.youtube.com/watch?v=JBvnB0279-Q

void calculate_pid() {
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if (pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if (pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function) {
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if (eeprom_data[function + 23] & 0b10000000)reverse = 1;                     //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if (actual < center) {                                                       //The actual receiver value is lower than the center value
    if (actual < low)actual = low;                                             //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 + difference;                                 //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if (actual > center) {                                                                      //The actual receiver value is higher than the center value
    if (actual > high)actual = high;                                           //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if (reverse == 1)return 1500 - difference;                                 //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void set_gyro_registers() {
  //Setup the MPU-6050
  if (eeprom_data[31] == 1) {
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x1B);                                            //Start reading @ register 0x1B
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                //Wait until the 6 bytes are received
    if (Wire.read() != 0x08) {                                   //Check if the value is 0x08
      digitalWrite(A2, HIGH);                                    //Turn on the warning led
      while (1)delay(10);                                        //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x1A);                                            //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x03);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro
  }
  //Setup the L3G4200D
  if (eeprom_data[31] == 2) {
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x20);                                            //We want to write to register 1 (20 hex).
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x23);                                            //We want to write to register 4 (23 hex).
    Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x23);                                            //Start reading @ register 0x23
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                //Wait until the 6 bytes are received
    if (Wire.read() != 0x90) {                                   //Check if the value is 0x90
      digitalWrite(A2, HIGH);                                    //Turn on the warning led
      while (1)delay(10);                                        //Stay in this loop for ever
    }

  }
  //Setup the L3GD20H
  if (eeprom_data[31] == 3) {
    Wire.beginTransmission(gyro_address);                        //Start communicationwith the address found during search.
    Wire.write(0x20);                                            //We want to write to register 1 (20 hex).
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search.
    Wire.write(0x23);                                            //We want to write to register 4 (23 hex).
    Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                        //Start communication with the address found during search
    Wire.write(0x23);                                            //Start reading @ register 0x23
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(gyro_address, 1);                           //Request 1 bytes from the gyro
    while (Wire.available() < 1);                                //Wait until the 6 bytes are received
    if (Wire.read() != 0x90) {                                   //Check if the value is 0x90
      digitalWrite(A2, HIGH);                                    //Turn on the warning led
      while (1)delay(10);                                        //Stay in this loop for ever
    }
  }
}



/////////////////////////////////////////////
// Calibration of Servos
/////////////////////////////////////////////
void calibrate_servos () {
   tid = millis();
   
  int count_motor = 1;
  /*
     Vi vil hente inn info fra serial monitor og justere servoen derifra
     Startvinkelen skal lagres som "null-vinkel" og overføres via EEPROM

     Når vi skriver OK i Serial monitor skal vi hoppe videre til neste motor
     Funksjonen skal gå igjennom kalibrering for alle fire motorene.

  */
Serial.println("Skriv inn '*' for å komme til kalibrering");
if(!cal){
      while(tid + 7000 > tid2) {
      tid2 = millis();
       if (Serial.available()) {
        data_in = Serial.read();
        if (data_in == '*') {
          cal = true;
        }
      }
  }
}
if(cal) {
  Serial.println("Velkommen til kalllllibrering");
  delay(2000);
  Serial.println("Start med motor 1: Skriv '+' for aa oeke vinkelen og '-' for aa minke den... Skriv OK naar du vil gaa til neste motor");
  while (count_motor < 5) {

    servo_value = s1;
    Serial.println("Sett servo1 til null-pitch og skriv OK naar du er ferdig med servo 1");
    while (count_motor == 1) {
      if (Serial.available()) {
        data_in = Serial.read();
        if (data_in == '0') {
          s1 = servo_value;
          count_motor++;
        }
        else {
          if (data_in == '1') {
            servo_value++;
            servo1.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : ");
            Serial.println(servo_value);
          }
          else if (data_in == '2') {
            servo_value--;
            servo1.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }
          else if (data_in == '7') {
            Serial.println("Skriv inn Microseconds du vil sende og trykk enter: ");
            String PWM;
            while (int k = 1) {
              if (Serial.available()) {
                PWM = Serial.readString();
                servo_value = PWM.toInt();
                servo1.writeMicroseconds(servo_value);
                k = 0;
                break;
              }
            }
            Serial.print("Servoverdi er nå : ");
            Serial.println(servo_value);
          }
          else {
            Serial.println("Ikke tillat");
          }
        }

      }

    }
    Serial.print("Null-pitchvinkelen til servo 1 er satt til: ");
    Serial.println(s1);

    servo_value = s2;
    Serial.println("Sett servo2 til null-pitch og skriv OK når du er ferdig med servo 2");
    while (count_motor == 2) {
      if (Serial.available()) {
        data_in = Serial.read();

        if (data_in == '0') {
          s2 = servo_value;
          count_motor++;
        }
        else {
          if (data_in == '1') {
            servo_value++;
            servo2.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }
          else if (data_in == '2') {
            servo_value--;
            servo2.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }

          else if (data_in == '7') {
            Serial.println("Skriv inn Microseconds du vil sende og trykk enter: ");
            String PWM;
            while (int k = 1) {
              if (Serial.available()) {
                PWM = Serial.readString();
                servo_value = PWM.toInt();
                servo2.writeMicroseconds(servo_value);
                k = 0;
                break;
              }
            }
            Serial.print("Servoverdi er nå : ");
            Serial.println(servo_value);
          }

          else {
            Serial.println("Ikke tillat");
          }
        }
      }
    }
    Serial.print("Null-pitchvinkelen til servo 2 er satt til: ");
    Serial.println(s2);


    servo_value = s3;
    Serial.println("Sett servo3 til null-pitch og skriv OK når du er ferdig med servo 3");
    while (count_motor == 3) {
      if (Serial.available()) {
        data_in = Serial.read();

        if (data_in == '0') {
          s3 = servo_value;
          count_motor++;
        }
        else {
          if (data_in == '1') {
            servo_value++;
            servo3.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }
          else if (data_in == '2') {
            servo_value--;
            servo3.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }

          else if (data_in == '7') {
            Serial.println("Skriv inn Microseconds du vil sende og trykk enter: ");
            String PWM;
            while (int k = 1) {
              if (Serial.available()) {
                PWM = Serial.readString();
                servo_value = PWM.toInt();
                servo3.writeMicroseconds(servo_value);
                k = 0;
                break;
              }
            }
            Serial.print("Servoverdi er nå : ");
            Serial.println(servo_value);
          }

          else {
            Serial.println("Ikke tillat");
          }
        }
      }
    }
    Serial.print("Null-pitchvinkelen til servo 3 er satt til: ");
    Serial.println(s3);



    servo_value = s4;
    Serial.println("Sett servo4 til null-pitch og skriv OK naar du er ferdig med servo 4");
    while (count_motor == 4) {
      if (Serial.available()) {
        data_in = Serial.read();

        if (data_in == '0') {
          s4 = servo_value;
          count_motor++;
        }
        else {
          if (data_in == '1') {
            servo_value++;
            servo4.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }
          else if (data_in == '2') {
            servo_value--;
            servo4.writeMicroseconds(servo_value);
            Serial.print("Servoverdi er nå : " );
            Serial.println(servo_value);
          }

          else if (data_in == '7') {
            Serial.println("Skriv inn Microseconds du vil sende og trykk enter: ");
            String PWM;
            while (int k = 1) {
              if (Serial.available()) {
                PWM = Serial.readString();
                servo_value = PWM.toInt();
                servo4.writeMicroseconds(servo_value);
                k = 0;
                break;
              }
            }
            Serial.print("Servoverdi er nå : ");
            Serial.println(servo_value);
          }

          else {
            Serial.println("Ikke tillat");
          }
        }
      }
    }
    Serial.print("Null-pitchvinkelen til servo 4 er satt til: ");
    Serial.println(s4);

  }

  Serial.println("Kalibræææring av servoer skal nå være fullført!");
}
Serial.println("Done");
}




