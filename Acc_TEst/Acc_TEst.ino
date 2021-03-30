#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
# include <Servo.h>


MPU6050 mpu;
Servo servo;

int16_t accY, accX;
float accAngle;


void setup() {  
  servo.attach(9);
  mpu.initialize();
  Serial.begin(9600);
}

void loop() {  
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
   
  accAngle = atan2(accY, accX)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.println(accAngle);
    servo.write(map(accAngle, -100, 100, 0, 180)); 
}
