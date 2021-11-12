//https://www.instructables.com/Arduino-Self-Balancing-Robot-1/
// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) {
  obj.print(arg);
  return obj;
}
template<>        inline Print& operator <<(Print &obj, float arg) {
  obj.print(arg, 4);
  return obj;
}


#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
//#include <NewPing.h>
//
//#include <Servo.h>
//Servo servox;

//#define leftMotorPWMPin   6
//#define leftMotorDirPin   7
//#define rightMotorPWMPin  5
//#define rightMotorDirPin  4
//
//#define TRIGGER_PIN 9
//#define ECHO_PIN 8
//#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

//void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
//  if(leftMotorSpeed >= 0) {
//    analogWrite(leftMotorPWMPin, leftMotorSpeed);
//    digitalWrite(leftMotorDirPin, LOW);
//  }
//  else {
//    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
//    digitalWrite(leftMotorDirPin, HIGH);
//  }
//  if(rightMotorSpeed >= 0) {
//    analogWrite(rightMotorPWMPin, rightMotorSpeed);
//    digitalWrite(rightMotorDirPin, LOW);
//  }
//  else {
//    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
//    digitalWrite(rightMotorDirPin, HIGH);
//  }
//}
 SoftwareSerial odrive_serial(8, 9);
 ODriveArduino odrive(odrive_serial);





void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
   odrive_serial.begin(115200);
   Serial.begin(115200); 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop



  // Serial to PC
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 10.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }


//      init_PID();
   Serial.println("HAIIIOOO");
}

void loop() {
      char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c - '0';
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, true)) return;

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, true, 25.0f)) return;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      if (!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
    }
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationX();  
  gyroX = mpu.getRotationZ();
  // set motor power after constraining it
//  motorPower = constrain(motorPower, -255, 255);

//  motorPower = constrain(motorPower, 0, 180);
//  Serial.println(currentAngle);

// The ISR will be called every 5 milliseconds
}
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
