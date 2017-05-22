#include <Servo.h>
 
////////////////////////////////////
// Legger inn Calibration values: //
////////////////////////////////////
int s1 = 1400;
int s2 = 1575;
int s3 = 1500;
int s4 = 1500;
 

char data_in;
bool cal = true;
int servo_value;
 
// Servo
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
 
 
void setup() {
  pinMode(2, OUTPUT);
  servo1.attach(2); // portE PE4 B00010000
  pinMode(2, OUTPUT);
  servo2.attach(3); // portE PE5 B00100000
  pinMode(2, OUTPUT);
  servo3.attach(12);// portE PE6 B01000000
  pinMode(2, OUTPUT);
  servo4.attach(13);// portE Pe6 B10000000
 
  Serial.begin(57600);
 
}
 
void loop() {
 
  int count_motor = 1;
  /*
     Vi vil hente inn info fra serial monitor og justere servoen derifra
     Startvinkelen skal lagres som "null-vinkel" og overføres via EEPROM
 
     Når vi skriver OK i Serial monitor skal vi hoppe videre til neste motor
     Funksjonen skal gå igjennom kalibrering for alle fire motorene.
  */
  if (cal) {
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
              Serial.print("Servoverdi er : ");
              Serial.println(servo_value);
            }
            else if (data_in == '2') {
              servo_value--;
              servo1.writeMicroseconds(servo_value);
              Serial.print("Servoverdi er : " );
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
              Serial.print("Servoverdi er : ");
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
      Serial.println("Sett servo2 til null-pitch og skriv OK naar du er ferdig med servo 2");
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
              Serial.print("Servoverdi er : " );
              Serial.println(servo_value);
            }
            else if (data_in == '2') {
              servo_value--;
              servo2.writeMicroseconds(servo_value);
              Serial.print("Servoverdi er : " );
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
              Serial.print("Servoverdi er : ");
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
      Serial.println("Sett servo3 til null-pitch og skriv OK naar du er ferdig med servo 3");
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
              Serial.print("Servoverdi er : " );
              Serial.println(servo_value);
            }
            else if (data_in == '2') {
              servo_value--;
              servo3.writeMicroseconds(servo_value);
              Serial.print("Servoverdi er : " );
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
              Serial.print("Servoverdi er : ");
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
              Serial.print("Servoverdi er : " );
              Serial.println(servo_value);
            }
            else if (data_in == '2') {
              servo_value--;
              servo4.writeMicroseconds(servo_value);
              Serial.print("Servoverdi er : " );
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
              Serial.print("Servoverdi er : ");
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
 
    Serial.println("Kalibrering av servoer er ferdig!");
  }
  Serial.println("Done");
}
