#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

/* Edit Date: August 7th, 11:15

  Important Edits: Included the get_pressure() implementation. uploaded the needed libraries to the drive
  Other important things:
  Some comments: 
  -- Dan
*/
const byte address[6] = "000001";
const int MPU = 0x68; // MPU6050 I2C address
File mpuLogFile, barLogFile, locationLogFile, logFile;

/* These values will be used to modify the g-range and gyro value that we will be getting from the accelerometer. 
It should be constant so immutable and it can be left as an integer because integers take less memory, and when we divide our input (which is a double), 
the result will be the same as if it was a double or a float, since the modifier will be casted(?) into a double or a float during the runtime(?). Source: Trust me bro :) 
-- Dan
*/
const int gRangeModifier = 8192; 
const int gyroModifier = 131;

bool deploymentCondition; // Might be redundant
double AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, yaw; 
double roll, pitch; 
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ; 
float elapsedTime, currentTime, previousTime; 

const double g = 9.81; // will be used in the parachute deployment control loop
double VelocityY = AccY*(elapsedTime); // might be used in the parachute deployment control loo
float flightHeight = (1/2*AccY*(elapsedTime*elapsedTime)); // might be used in parachute deployment control loop. also will be used to record data. might need to integrate
// if we will have a really jumping acceleration value. 
const double deploymentVelocity = (-5 * g); //might be used in the parachute deployment control loop


// One-time execute
void setup() {
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

  /* Initializing the SD card */
  if (!SD.begin(4)) {
    Serial.println("SD-card initialization failed! Check wiring or else!");
    while (1);
  }
  Serial.println("initialization done.");
  logFile = SD.open("log.txt", FILE_WRITE);

  /* Running the IMU error */
  calculate_IMU_error();

  /* Initializing the pressure sensor */
  if(!bmp.begin())
  {
    Serial.print("Pressure sensor initialization failed! Check your wiring or I2C ADDR!");
    while(1);
  }
  displaySensorDetails();
}

// Main
void loop() {

  /* I am not sure how to design the function calling. 
  I feel like if we put time as an argument for functions and call them from the main, it might be weird? 
  I do not know why and we should definitely test it. I hope it will not break and will work, since it would be easier to set different frequencies in that case.
  If it is a problem, though, we can always change it, though the code will look dirtier. No problem. 
  --Dan
  */

  get_mpu_data(100); //10Hz frequency of data acquisition
  get_pressure(100);
  get_location(100);

  /* The following comment is about the parachute deployment condition.

    Acceleration should be constant after the apogee, 9.81 m/s downwards. Now a genius question: does this sensor measure acceleration or velocity?
    Because velocity will increase with time.... So, if "AccY" is actually a velocity, we can try to predict what kind of behavior the rocket will have.
    It will start falling, gaining velocity. Mass is constant (I hope so), but the kinetic energy grows with a factor of g^2 every second. Momentum grows with just g each
    second, though it is still a lot. 10 times a second.

    I think about this approach: we firstly need to find how much time is needed for the rocket to fall to the ground. I will omit the air resistense (it's a simpler
    calculation, we do not really have to do a lot of stuff right now). There is a formula: deltaX = V0t + (1/2)at^2. Solving for time yields t = sqrt(2deltaX/a).
    deltaX = 3000m, a = 9.81 m/s. Solving yields t = 24.7309s. This much time it should take for the rocket to fall if there was no air. With air it should be additional
    about 5-10(???) seconds? Should calculate, depends on the bottom surface area and the surface area of the parachute. And the time at which the parachute will be deployed :)
    Shite :)

    We need to deploy parachute at such a point of time that when it will be near the ground, its momentum will not destroy the body of the rocket in the ground contact.
    So it should depend on the material we will be using for the body. It also may depend on the height or the rocket? Not sure.
    In this case, I think a conservative estimate should be that the momentum should be at least two times smaller that the amount of pressure/force the bottom can take.
    Just for the safety sake. Knowing how much pressure/force the bottom can take in some arbitrary time, we can do the calculations. I think it should not be that difficult.
    Important note: the ground contact time will be large, so we should account for that. 

    So, main conditions are ready, I think. We should do calculations after we will know the materials that will be used.
    Of course, we can deploy the parachute about 2-3 seconds after the apogee, or choose some other arbitrary value, however, since the apogee is 3km away,
    deploying parachute at a too early time will take the rocket really far away from the launch position, and we do not really want that either. So the calculations have
    to be really good and as exact as possible. We can include other variables in here, too. No problem, it's just a sketch of a solution.

    The deployment condition, therefore, might depend on either the height (or relative height to the apogee if we reset the height after the apogee) of the flight,
    or time after the apogee (we should reset the timer after the acceleration becomes 0, but we should be sure that it will show 0 only in the apogee and there won't be some
    hardware bug with the sensor), or the velocity in Y axis. I think that using the Y axis velocity should be easier since we do not need to modify code in that case and
    it seems to be more error-prone, but I did not really think through it that much and there might be some issues. We also might need to modify acceleration in Y axis
    in such a way that it will be negative after the apogee if it does not work this way already. It will depend on the 
   */

  if (VelocityY < deploymentVelocity) {
    deploy_parachute();
  }

  return 0;
}

// COMPLETED. HAS TO BE TESTED. I know there is a lot of bugs in there... 
void calculate_IMU_error() {
    /* We can call this funtion in the setup section to calculate the accelerometer and gyro data error */
    int counter = 0;
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
}

// COMPLETED. HAS TO BE TESTED. A lot of bugs might be in here.... (int time) is the delay with which the function has to give output.
void get_mpu_data(int time) {
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
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX; //the f are these coefficients.
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    //store all of the acquired data in an array of doubles
    double data[] = {roll, pitch, yaw};
    //pass the array to an SD card function
    write_mpu_to_card(data);
    delay(time);
  }

// COMPLETED. HAS TO BE TESTED. (int time) is the delay with which the function has to give output.
void get_pressure(int time){ 
  // Name of the sensor: BMP180 GY-68
  sensors_event_t event;
  bmp.getEvent(&event);

  float pressure = event.pressure;
  float temperature = event.temperature; //or bmp.getTemperature(&temperature)
  float altitudeDueToPressure = bmp.pressureToAltitude(seaLevelPressure,event.pressure);

  double data = {temperature, pressure, altitudeDueToPressure};

  write_bar_to_card(data); 
  delay(time);
}

// Up to be implemented by Zhakhangir. (int time) is the delay with which the function has to give output.
void get_location(int time){

  /*

  xPos = ;
  yPos = ;
  zPos = ;

  double data[] = {xPos, yPos, zPos};

  write_location_to_card(data); */
  delay(time);
}

// Up to be implemented, but only after we will understand what mechanism will be used. Can think about the condition for deployment already.
void deploy_parachute() {

  /* This function should activate the servomotor that holds the parachute. implementation depends on the way the parachute team does its work. 
     So waiting for them.
     -- Dan
     */

  // I think we also should record the time that is being elapsed before the parachute is deployed. we can use that data, dunno how --Dan
  return 0; 
}

/* COMPLETED. HAS TO BE TESTED. These three functions simply record respective data to the SD card. Nothing special. They are called inside of the respective functions.
Gotta do a lot of testing though, huh. And I might have to come up with a clever loop condition. And all of these functions have to be updated so they 
record the exact needed number of data types. It is one of the last parts of the prog and this should not be a problem once the higher order functions
are working fine.
--Dan
 */ 
void write_mpu_to_card(double data[]) {

  mpuLogFile = SD.open("mpuLog.txt", FILE_WRITE);

  mpuLogFile.print(data[0]);
  mpuLogFile.print(" / ");
  mpuLogFile.print(data[1]);
  mpuLogFile.print(" / ");
  mpuLogFile.println(data[2]);
  mpuLogFile.print(" / ");

  mpuLogFile.close();
  delay(200);
}
void write_bar_to_card(double data[]) {

  barLogFile = SD.open("barLog.txt", FILE_WRITE);
  
  barLogFile.print(data[0]);
  barLogFile.print(" / ");
  barLogFile.print(data[1]);
  barLogFile.print(" / ");
  barLogFile.println(data[2]);
  barLogFile.print(" / ");

  barLogFile.close();
  delay(200);
}
void write_location_to_card(double data[]) {

  locationLogFile = SD.open("locationLog.txt", FILE_WRITE);
  
  locationLogFile.print(data[0]);
  locationLogFile.print(" / ");
  locationLogFile.print(data[1]);
  locationLogFile.print(" / ");
  locationLogFile.println(data[2]);
  locationLogFile.print(" / ");

  locationLogFile.close();
  delay(200);
}
