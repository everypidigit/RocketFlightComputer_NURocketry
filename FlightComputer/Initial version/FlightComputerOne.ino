#include <Wire.h>
#include <SPI.h>

const byte address[6] = "000001";
const int MPU = 0x68; // MPU6050 I2C address
int counter = 0; //will be used in the MPU control loop
/*
These values will be used to modify the g-range and gyro value that we will be getting from the accelerometer. 
It should be constant so immutable and it can be left as an integer because integers take less memory and when we divide our input (which is a double), 
the result will be the same as if it was a double or a float, since the modifier will be casted(?) into a double or a float during the runtime(?) :) 
*/
const int gRangeModifier = 8192; 
const int gyroModifier = 131;

double AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; 
double roll, pitch, yaw; 
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; 
float elapsedTime, currentTime, previousTime; 

//one-time execute
void setup() {
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x0);                  //Set the register bits as 00011000 (+/- 16g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x0);                   // Set the register bits as 00011000 (2000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  
  //Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
}

//main part of the file
void loop() {

  take_mpu_data(100); //will be taking the MPU data with delay of 100ms or 10 times a second

  return 0;
}

//completed
void calculate_IMU_error() {
    // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
    while (counter < 201) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      AccX = (Wire.read() << 8 | Wire.read()) / gRangeModifier ;
      AccY = (Wire.read() << 8 | Wire.read()) / gRangeModifier ;
      AccZ = (Wire.read() << 8 | Wire.read()) / gRangeModifier ;
      // Sum all readings
      AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      counter = counter++;
    }
    //Divide the sum by the value of the counter to get the average error value for the acceleration
    AccErrorX = AccErrorX / counter;
    AccErrorY = AccErrorY / counter;
    //Reset the counter so we can use it in another loop
    counter = 0;
    while (counter < 201) {
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      GyroX = Wire.read() << 8 | Wire.read();
      GyroY = Wire.read() << 8 | Wire.read();
      GyroZ = Wire.read() << 8 | Wire.read();
      // Sum all readings
      GyroErrorX = GyroErrorX + (GyroX / gyroModifier);
      GyroErrorY = GyroErrorY + (GyroY / gyroModifier);
      GyroErrorZ = GyroErrorZ + (GyroZ / gyroModifier);
      counter = counter++;
    }
    
    //Divide the sum by the value of the counter to get the average error value for the gyroscope
    GyroErrorX = GyroErrorX / counter;
    GyroErrorY = GyroErrorY / counter;
    GyroErrorZ = GyroErrorZ / counter;
    
    /*Print the error values on the Serial Monitor
    The output is in the format of AccErX/AccErY/GyrErrX/GyrErrY/GyrErrZ */
    Serial.print(AccErrorX);
    Serial.print("/");
    Serial.print(AccErrorY);
    Serial.print("/");
    Serial.print(GyroErrorX);
    Serial.print("/");
    Serial.print(GyroErrorY);
    Serial.print("/");
    Serial.print(GyroErrorZ);
}

//completed
void take_mpu_data(int time) {


    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers 
    //We will be using just +-4g since we will not need more. gRangeModifier is equal to 8192, which is a value we have taken from the MPU6050 datasheet.
    AccX = (Wire.read() << 8 | Wire.read()) / gRangeModifier; 
    AccY = (Wire.read() << 8 | Wire.read()) / gRangeModifier; 
    AccZ = (Wire.read() << 8 | Wire.read()) / gRangeModifier;  
    // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.94; 
    // AccErrorY ~(-1.58)
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 3.8;  
    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / (double)1000.0; // Divide by 1000 to get seconds
    
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    // We will be using 250deg/s since we will not need more. gyroModifier is equal to 131, which is a value we have taken from the MPU6050 datasheet.
    GyroX = (Wire.read() << 8 | Wire.read()) / gyroModifier; 
    GyroY = (Wire.read() << 8 | Wire.read()) / gyroModifier;
    GyroZ = (Wire.read() << 8 | Wire.read()) / gyroModifier;
    // Correct the outputs with the calculated error values
    GyroX = GyroX + 2.89; // GyroErrorX ~(-0.56)
    GyroY = GyroY + 1.97; // GyroErrorY ~(2)
    GyroZ = GyroZ + 0.83; // GyroErrorZ ~ (-0.8)
    
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; 
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw = yaw + GyroZ * elapsedTime;
    // Calculating Roll and Pitch from the accelerometer data
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    // Print the values on the serial monitor
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.println(yaw);

    delay(time)
  }

//up to be implemented
void take_pressure(int time){

  return 0;
}

//up to be implemented
void take_location(int time){
  return 0;
}

//up to be implemented
void transfer_to_card() {

  return 0;
}

//up to be implemented
void deploy_parachute(bool cond) {
  return 0; 
}