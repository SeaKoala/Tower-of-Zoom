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


#define Kp  .1
#define Kd  0.05
#define Ki  .5
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0, var, prevPower = 0;
volatile byte count = 0;

// pin 19: RX - connect to ODrive TX PIN 1
// pin 18: TX - connect to ODrive RX PIN 2
HardwareSerial& odrive_serial = Serial1;


// ODrive object
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
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  Wire.begin();
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

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");

  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
      Serial.println("check");
  mpu.initialize();
Serial.println("check2222");
  mpu.setYAccelOffset(833);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop
  //  init_PID();
Serial.println("HAIIIOOO");
}

void loop() {

  if (Serial.available()) {
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

    // Sinusoidal test move
    if (c == 't') {
      Serial.println("Init PID");
      init_PID();
    }

    if (c == 'x') {
      Serial.println("Executing Angle Calc");
      while (c == 'x') {
        accY = mpu.getAccelerationY();
        accZ = mpu.getAccelerationX();
        gyroX = mpu.getRotationZ();
        var = map(currentAngle, -100, 100, -30, 30);
        var = var/50;
        Serial.println(motorPower);
        odrive.SetPosition(1, 0);
        odrive.SetPosition(0, 0);

//        var = var / 100;
//        if(var > 0 ){
//          odrive.SetPosition(1, var);
//          odrive.SetPosition(0, -var);
//        }
//        if(var < 0 ){
//          odrive.SetPosition(0, -var);
//          odrive.SetPosition(1, var);
//        }
//        if(var == 0 ){
//          odrive.SetVelocity(0, 0);
//          odrive.SetVelocity(1, 0);
//        }
//        delay(5);
//        prevPower = motorPower;
      }
    }

    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 2.0f * cos(ph);
        float pos_m1 = 2.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      Serial.println("check");
      odrive_serial << "r vbus_voltage\n";
      Serial.println("check2222");
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 2.0f * cos(ph);
        float pos_m1 = 2.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }
  }
}



ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum) * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;
}
