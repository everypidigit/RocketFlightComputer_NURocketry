#include <Wire.h>
#include <SPI.h>

const byte address[6] = "000001";
const int MPU = 0x68; // MPU6050 I2C address

/* These values will be used to modify the g-range and gyro value that we will be getting from the accelerometer. 
It should be constant so immutable and it can be left as an integer because integers take less memory, and when we divide our input (which is a double), 
the result will be the same as if it was a double or a float, since the modifier will be casted(?) into a double or a float during the runtime(?). Source: Trust me bro :) 
-- Dan
*/
const int gRangeModifier = 8192; 
const int gyroModifier = 131;

double AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, yaw; 
double roll, pitch; 
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; 
float elapsedTime, currentTime, previousTime; 
int counter = 0;
int frequency = 500;

void setup(){
Serial.begin(19200);
Wire.begin();                      // Initialize comunication
Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
Wire.write(0x6B);                  // Talk to the register 6B
Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
Wire.endTransmission(true);        //end the transmission

/* Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g) */
Wire.beginTransmission(MPU);
Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
Wire.write(0x0);                  //Set the register bits as 00011000 (+/- 16g full scale range)
Wire.endTransmission(true);
/* Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s) */
Wire.beginTransmission(MPU);
Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
Wire.write(0x0);                   // Set the register bits as 00011000 (2000deg/s full scale)
Wire.endTransmission(true);

delay(20);

}

void loop(){
    // Name of the sensor: MPU6250 
Wire.beginTransmission(MPU);
Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers 
//We will be using just +-4g since we will not need more. gRangeModifier is equal to 8192, which is a value we have taken from the MPU6050 datasheet.
AccX = (Wire.read() << 8 | Wire.read()) / gRangeModifier; 
AccY = (Wire.read() << 8 | Wire.read()) / gRangeModifier; 
AccZ = (Wire.read() << 8 | Wire.read()) / gRangeModifier;  
//Serial.println(AccX);
// AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI); 
//Serial.println(accAngleX);
// AccErrorY ~(-1.58)
accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);  
//Serial.println(accAngleY);
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
GyroX = GyroX; // GyroErrorX ~(-0.56)
GyroY = GyroY; // GyroErrorY ~(2)
GyroZ = GyroZ; // GyroErrorZ ~ (-0.8)
//Serial.println(GyroY);

gyroAngleX = (gyroAngleX + GyroX * elapsedTime); 
roll =(0.96 * gyroAngleX + 0.04 * accAngleX); //fine for now, about 0.03 degree/sec. sometimes fluctuates aroung 0
//Serial.println(counter);
//Serial.println(GyroZ);


gyroAngleY = gyroAngleY + GyroY * elapsedTime;
yaw = yaw + GyroZ * elapsedTime;
pitch = -(counter * 0.085)+(0.96 * gyroAngleY + 0.04 * accAngleY); //fine for now too. fluctuates between -0.1 and 0.3 degrees. cool

//store all of the acquired data in an array of doubles
double data[] = {roll, pitch, yaw}; //roll is fine for now. pitch is fine for now too. 
//pass the array to an SD card function
//print all of the data to the serial monitor
Serial.println(data[1]);

//Serial.println(data[2]);
counter = counter + 1;

delay(frequency);
}



/*void get_mpu_data(){
    // Name of the sensor: MPU6250 
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
roll = (0.96 * gyroAngleX + 0.04 * accAngleX); //the f are these coefficients.
pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

//store all of the acquired data in an array of doubles
double data[] = {roll, pitch, yaw};
//pass the array to an SD card function
//print all of the data to the serial monitor
Serial.println(data[0]);


} */
