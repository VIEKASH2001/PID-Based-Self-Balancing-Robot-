#include <Wire.h>
#include "Kalman.h" 

Kalman kalmanX;

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

float accXangle;//, accYangle; // Angle calculate using the accelerometer
float gyroXangle;//, gyroYangle; // Angle calculate using the gyro
float kalAngleX;//, kalAngleY; // Calculate the angle using a Kalman filter

unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data
float CurrentAngle;

// Motor controller pins
const int AIN1 = 5;  // (pwm) pin 3 connected to pin AIN1 
const int AIN2 = 6;  // (pwm) pin 9 connected to pin AIN2 
const int BIN1 = 9; // (pwm) pin 10 connected to pin BIN1  
const int BIN2 = 10;  // (pwm) pin 11 connected to pin BIN2 

int speed;

// PID
const float Kp = 4; 
const float Ki = 1;
const float Kd = 1;
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9*1.12;
#define   GUARD_GAIN   10.0

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))


void setup() {  
  pinMode(AIN1, OUTPUT); // set pins to output
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.begin(57600);
  Wire.begin();
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 

    while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  gyroXangle = accXangle;
  timer = micros();
}


void loop() {
  runEvery(25)  // run code @ 40 Hz
  {
    dof();
    if (CurrentAngle <= 180.2 && CurrentAngle >= 179.8)
    {
      stop();
    }
    else{
    if (CurrentAngle < 230 && CurrentAngle > 130)
    {
    Pid();
    Motors();
    }
    else
    {
      stop();
    }
  }
  }
  Serial.println(speed);
}

void Motors(){
  if (speed > 0)
  { 
    //forward 
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
  }
  else
  { 
    // backward
    speed = map(speed,0,-255,0,255);
    analogWrite(AIN1, 0);
    analogWrite(AIN2, speed);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
  }
}

void stop()
{
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

void Pid(){
  error = 0 - CurrentAngle;  // 0 = SETPT
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);
  last_error = error;
  speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}

void dof()
{
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  double gyroXrate = (double)gyroX/131.0;
  CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
  timer = micros();
}
