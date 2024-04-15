#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AccelStepper.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


//initialize
MPU6050 mpu;
AccelStepper stepperX(AccelStepper::DRIVER, 2, 5), stepperY(AccelStepper::DRIVER, 3, 6);
SoftwareSerial serial_connection(17, -1);
TinyGPSPlus gps;


//TIMING VARIABLES
unsigned long lastMPUUpdate = 0;
unsigned long updateIntervalMPU = 100; 
unsigned long lastGPSUpdateTime = 0;
const unsigned long motorUpdateInterval = 100; 
const unsigned long gpsUpdateInterval = 1000; 


//MOTOR VARIABLES
const float STEPS_PER_REV = 1600; // Steps per revolution of stepper motor
const float GEAR_RATIO = 2; // Gear ratio
const float STEPS_PER_DEGREE = (STEPS_PER_REV * GEAR_RATIO) / 360.0; //


//MPU VARIABLES
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float roll = 0;
float pitch = 0;
float yaw = 0.0;
float gyroY_bias = 0.0;
float targetYaw = 90;
float currentYaw = 0;
float lastGoodYaw = 0;
const float gyroScale = 131.0;
const float alpha = 0.98;
const float dt = 0.5;
float yawDriftPerSecond = 0.0;
const int calibrationDuration = 10000;


// Azimuth and Elevation Variables
int currentAzimuthPosition = 0;
int currentElevationPosition = 0;


// Data 
String inputString = "";
boolean stringComplete = false;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  serial_connection.begin(9600);

  //MPU DRIFT CALIBRATION
  calibrateGyroForDrift();

  //MOTOR SPEEDS
  stepperX.setMaxSpeed(200); 
  stepperY.setMaxSpeed(200);
  stepperX.setAcceleration(100); 
  stepperY.setAcceleration(100);

  //MPU OFFSETS
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(16387); 
  mpu.setXGyroOffset(2); 
  mpu.setYGyroOffset(2); 
  mpu.setZGyroOffset(1);
}


void calculateMoveSteps(float targetAzimuth, float targetElevation, int &moveStepsAzimuth, int &      moveStepsElevation) {
  moveStepsAzimuth = targetAzimuth * STEPS_PER_DEGREE - currentAzimuthPosition;
  moveStepsElevation = targetElevation * STEPS_PER_DEGREE - currentElevationPosition;
}


void loop() {
    processGPS();
    processMPU();
    updateMotors();
}


void parseInputString() {
    if (stringComplete) {
    // parse the inputString here
    int separatorIndex = inputString.indexOf(',');
    String azimuthStr = inputString.substring(0, separatorIndex);
    String elevationStr = inputString.substring(separatorIndex + 1);

    // Convert strings to float
    float azimuth = azimuthStr.toFloat();
    float elevation = elevationStr.toFloat();

    int targetAzimuthSteps, targetElevationSteps;
    calculateMoveSteps(azimuth, elevation, targetAzimuthSteps, targetElevationSteps);

    stepperX.move(targetAzimuthSteps);
    stepperY.move(targetElevationSteps);
    
    // clear the string for new input:
    inputString = "";
    stringComplete = false;
  }
}


void processGPS() {
  while (serial_connection.available() > 0) {
    if (gps.encode(serial_connection.read())) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastGPSUpdateTime >= gpsUpdateInterval) {
        lastGPSUpdateTime = currentMillis; // Update the last update time

        // Now, check if the GPS data is valid and if so, send it
        if (gps.location.isValid()) {
          Serial.print("Lat: ");
          Serial.println(gps.location.lat(), 6); // Print latitude with 6 decimal places
          Serial.print("Lng: ");
          Serial.println(gps.location.lng(), 6); // Print longitude with 6 decimal places
        }

        if (gps.altitude.isValid()) {
          Serial.print("Altitude: ");
          Serial.print(gps.altitude.meters());
          Serial.println(" Meters"); // Ensure to add line break for altitude
        }

        if (gps.satellites.isValid()) {
          Serial.print("Satellites: ");
          Serial.println(gps.satellites.value());
        }
      }
    }
  }
}


void processMPU() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMPUUpdate >= updateIntervalMPU) {
    float previousTime = lastMPUUpdate;
    lastMPUUpdate = currentMillis;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float f_ax = constrain(ax, -32768, 32767);
    float f_ay = constrain(ay, -32768, 32767);
    float f_az = constrain(az, -32768, 32767);

    // Calculate angles based on the accelerometer
    float accRoll = atan2(f_ay, f_az) * RAD_TO_DEG;
    float accPitch = atan2(-f_ax, sqrt(f_ay * f_ay + f_az * f_az)) * RAD_TO_DEG;

    // Integrate gyro data to get angles
    float gyroX = gx / 131.0;
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    float gyroRoll = roll + gyroX * dt;
    float gyroPitch = pitch + gyroY * dt;
    float gyroYaw = yaw + gyroZ * dt;

    roll = 0.98 * gyroRoll + 0.02 * accRoll;
    pitch = 0.98 * gyroPitch + 0.02 * accPitch;

    // Calculate time step
    float dt = (currentMillis - previousTime) / 1000.0;

    // Apply yaw drift correction
    gyroZ -= yawDriftPerSecond;

    // Integrate gyro data to update yaw
    yaw += gyroZ * dt;

    // Normalize yaw
    while (yaw < 0) yaw += 360;
    while (yaw > 360) yaw -= 360;

    // Serial prints
    Serial.print("MPU: ");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(yaw);
    Serial.println();
  }
}



void calibrateGyroForDrift() {
    long startTime = millis();
    long endTime = startTime + calibrationDuration;
    float yawSum = 0;
    int count = 0;

    while (millis() < endTime) {
        if (millis() - lastMPUUpdate >= updateIntervalMPU) {
            lastMPUUpdate = millis();
            int16_t gx, gy, gz;
            mpu.getRotation(&gx, &gy, &gz);
            float gyroZ = gz / 131.0;  // Assuming sensitivity is set to +/-250 deg/s
            yawSum += gyroZ;
            count++;
        }
    }
    yawDriftPerSecond = yawSum / count;
    Serial.print("Calibrated Yaw Drift Per Second: ");
    Serial.println(yawDriftPerSecond);
}


void updateMotors() {
    // Check if motors need to move and update their positions accordingly
    if (!stepperX.isRunning() && !stepperY.isRunning()) {
        // Only calculate new steps if motors are not currently moving
        if (stringComplete) {
            parseInputString();
            stringComplete = false;
        }
    }

    // Continuously called to execute the steps
    stepperX.run();
    stepperY.run();
}


void returnToHome() {
    // Calculate steps back to home from current position
    int stepsToHomeAzimuth = -currentAzimuthPosition; // Negative because we're returning
    int stepsToHomeElevation = -currentElevationPosition;

    // Move motors back to home position
    moveStepper(stepperX, stepsToHomeAzimuth);
    moveStepper(stepperY, stepsToHomeElevation);

    // Reset current position
    currentAzimuthPosition = 0;
    currentElevationPosition = 0;
}


void moveStepper(AccelStepper &stepper, int steps) {
    stepper.move(steps);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
}


void serialEvent() {
  static String serialBuffer = ""; // Use a static variable to accumulate characters
  
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    // Append the read character to the buffer
    serialBuffer += inChar;

    // Check if the character is a newline, indicating the end of a command
    if (inChar == '\n') {
      // Trim any whitespace from the command for clean processing
      serialBuffer.trim();
      
      // Process the command stored in serialBuffer
      if (serialBuffer.startsWith("RTH")) {
        returnToHome();
      } else if (serialBuffer.startsWith("MOT")) {
        int firstCommaIndex = serialBuffer.indexOf(',');
        int secondCommaIndex = serialBuffer.indexOf(',', firstCommaIndex + 1);
        if (firstCommaIndex != -1 && secondCommaIndex != -1) {
          // Extracting the azimuth and elevation from the command
          String azStr = serialBuffer.substring(firstCommaIndex + 1, secondCommaIndex);
          String elStr = serialBuffer.substring(secondCommaIndex + 1);
          float azimuth = azStr.toFloat();
          float elevation = elStr.toFloat();
          track_celestial_object(azimuth, elevation);
        }
      } 
      // Clear the serialBuffer for the next command
      serialBuffer = "";
    }
  }
}

void track_celestial_object(float azimuth, float elevation) {
  int targetAzimuthSteps = azimuth * STEPS_PER_DEGREE;
  int targetElevationSteps = elevation * STEPS_PER_DEGREE;

  // Combine calculations
  int moveStepsAzimuth = targetAzimuthSteps - currentAzimuthPosition;
  int moveStepsElevation = targetElevationSteps - currentElevationPosition;

  // Move the stepper motors
  stepperX.move(moveStepsAzimuth);
  stepperY.move(moveStepsElevation);

  // Wait for both motors to reach their target positions
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    stepperX.run();
    stepperY.run();
  }

  // Update current positions
  currentAzimuthPosition = targetAzimuthSteps;
  currentElevationPosition = targetElevationSteps;
}
// /dev/ttyACM0