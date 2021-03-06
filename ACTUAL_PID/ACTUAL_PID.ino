#include <Wire.h>
#include <Kalman.h>  
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead 
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

int IN4 = 10;       
int IN3 = 9;     
int IN2 = 6;      
int IN1 = 5;    
int Speed=0;

/*working variables*/
unsigned long lastTime=0;
double Input=0, Output=0, AOutput=0, Setpoint=0;
double errSum=0, lastErr=0;
double kp=0, ki=0, kd=0;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void forward() 
  {
    analogWrite(IN1, Speed);         
    analogWrite(IN2, 0);
    analogWrite(IN3, Speed);                                
    analogWrite(IN4, 0);
    Serial.print("forward"); Serial.print("\t");
  } 

void backward() 
  {
    analogWrite(IN1, 0);         
    analogWrite(IN2, Speed);
    analogWrite(IN3, 0);                                
    analogWrite(IN4, Speed);
    Serial.print("backward"); Serial.print("\t");
  } 
  
void Stop() 
  {
    analogWrite(IN1, 0);         
    analogWrite(IN2, 0);
    analogWrite(IN3, 0);                                
    analogWrite(IN4, 0);
    Serial.print("stop"); Serial.print("\t");
  } 


void Compute()
{
   /*How long since we last calculated*/
   unsigned long now = (millis());
   double timeChange = (double)(now - lastTime);
   /*Compute all the working error variables*/
   double error = Setpoint - Input;
  if(Output<(kp*15) && Output>-(kp*15))
  {
     errSum += (error * timeChange)/1000;
  }
  
   double dErr = ((error - lastErr)*1000) / timeChange;
   /*Compute PID Output*/
   Output = kp * error + ki * errSum + kd * dErr;
   
  
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
}

void setup() {
   kp = 15;
   ki = 200;
   kd = 0.5;
   Setpoint = -5;
   pinMode(IN4, OUTPUT);      
   pinMode(IN3, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(IN1, OUTPUT);
   
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
   
}

void loop() {
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
  gyroYangle += gyroYrate * dt;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
    
  Serial.print("PITCH: ");Serial.print(kalAngleY); Serial.print("\t");
  
  Input = kalAngleY;
  
  Compute();

  Serial.print("Output: ");Serial.print(Output); Serial.print("\t");
  
  
  if(Output>30)
  {
    Speed = map(Output, 30, (kp*15), 40, 230);
    forward();
    
  }
 if(-30<=Output & Output<30)
  {
    Stop(); 
  }
  if(Output<-30)
  {
    AOutput=abs(Output);
    Serial.print("AOutput: ");Serial.print(AOutput); Serial.print("\t");
    Speed = map(AOutput, 30, (kp*15), 40, 230);
    backward();
  }
  Serial.print("speed: ");Serial.println(Speed);
}
