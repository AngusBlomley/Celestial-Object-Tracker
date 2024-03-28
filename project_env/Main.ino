#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <AccelStepper.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

MPU6050 mpu;
AccelStepper stepperX(AccelStepper::DRIVER, 2, 3), stepperY(AccelStepper::DRIVER, 4, 5);
SoftwareSerial serial_connection(17, -1);
TinyGPSPlus gps;

unsigned long lastMPUUpdate = 0;
unsigned long updateIntervalMPU = 500; // Interval for MPU updates
unsigned long lastGPSUpdateTime = 0;
const unsigned long gpsUpdateInterval = 500; // Update GPS data every 1000 milliseconds (1 second)

const float STEPS_PER_REV = 1600;                                    // Steps per revolution of your stepper motor
const float GEAR_RATIO = 2;                                          // Gear ratio of any gearing attached to the motors
const float STEPS_PER_DEGREE = (STEPS_PER_REV * GEAR_RATIO) / 360.0; //

int currentAzimuthPosition = 0; // Current azimuth step position
int currentElevationPosition = 0;

String inputString = ""; // a string to hold incoming data
boolean stringComplete = false;

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  serial_connection.begin(9600);
  stepperX.setMaxSpeed(1000);
  stepperY.setMaxSpeed(1000);
  stepperX.setAcceleration(100);
  stepperY.setAcceleration(100);
  mpu.setXAccelOffset(-2776);
  mpu.setYAccelOffset(655);
  mpu.setZAccelOffset(4363);
  mpu.setXGyroOffset(4);
  mpu.setYGyroOffset(2);
  mpu.setZGyroOffset(14);
}

void loop()
{
  // GPS data processing
  while (serial_connection.available() > 0)
  {
    if (gps.encode(serial_connection.read()))
    {
      unsigned long currentMillis = millis();
      if (currentMillis - lastGPSUpdateTime >= gpsUpdateInterval)
      {
        lastGPSUpdateTime = currentMillis; // Update the last update time

        // Now, check if the GPS data is valid and if so, send it
        if (gps.location.isValid())
        {
          Serial.print("Lat: ");
          Serial.println(gps.location.lat(), 6); // Print latitude with 6 decimal places
          Serial.print("Lng: ");
          Serial.println(gps.location.lng(), 6); // Print longitude with 6 decimal places
        }

        if (gps.altitude.isValid())
        {
          Serial.print("Altitude: ");
          Serial.print(gps.altitude.meters());
          Serial.println(" Meters"); // Ensure to add line break for altitude
        }

        if (gps.satellites.isValid())
        {
          Serial.print("Satellites: ");
          Serial.println(gps.satellites.value());
        }
      }
    }
  }

  // MPU data processing
  unsigned long currentMillis = millis();
  if (currentMillis - lastMPUUpdate >= updateIntervalMPU)
  {
    lastMPUUpdate = currentMillis;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Serialize the MPU data
    Serial.print("MPU: ");
    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");
    Serial.println(gz);
  }

  if (stringComplete)
  {
    // parse the inputString here
    int separatorIndex = inputString.indexOf(',');
    String azimuthStr = inputString.substring(0, separatorIndex);
    String elevationStr = inputString.substring(separatorIndex + 1);

    // Convert strings to float
    float azimuth = azimuthStr.toFloat();
    float elevation = elevationStr.toFloat();

    // Here, add your logic to convert azimuth and elevation to stepper steps
    // and move the motors accordingly

    // clear the string for new input:
    inputString = "";
    stringComplete = false;
  }

  if (stringComplete)
  {
    if (inputString == "RTH")
    {
      returnToHome(); // Ensure you have this function defined to handle returning home
    }
    else
    {
      // Existing logic to handle MOT commands
    }
    inputString = ""; // Clear the string for new input
    stringComplete = false;
  }
}

void returnToHome()
{
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

void moveStepper(AccelStepper &stepper, int steps)
{
  stepper.move(steps);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      stringComplete = true;
      inputString.trim(); // Remove any trailing whitespace

      if (inputString.startsWith("RTH"))
      {
        returnToHome();
      }
      else if (inputString.startsWith("MOT"))
      {
        int firstCommaIndex = inputString.indexOf(',');
        int secondCommaIndex = inputString.indexOf(',', firstCommaIndex + 1);
        if (firstCommaIndex != -1 && secondCommaIndex != -1)
        {
          // Extracting the azimuth and elevation from the command
          String azStr = inputString.substring(firstCommaIndex + 1, secondCommaIndex);
          String elStr = inputString.substring(secondCommaIndex + 1);
          float azimuth = azStr.toFloat();
          float elevation = elStr.toFloat();
          track_celestial_object(azimuth, elevation);
        }
      }
      // Reset for the next command
      inputString = "";
      stringComplete = false;
    }
  }
}

void track_celestial_object(float azimuth, float elevation)
{
  // Calculate target steps for each motor
  int targetAzimuthSteps = azimuth * STEPS_PER_DEGREE;
  int targetElevationSteps = elevation * STEPS_PER_DEGREE;

  // Calculate the difference between the current and target positions
  int moveStepsAzimuth = targetAzimuthSteps - currentAzimuthPosition;
  int moveStepsElevation = targetElevationSteps - currentElevationPosition;

  // Move the motors to the target positions
  stepperX.move(moveStepsAzimuth);
  stepperY.move(moveStepsElevation);

  // You might need to adjust this loop depending on your setup
  // to ensure motors reach their target positions
  while (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0)
  {
    stepperX.run();
    stepperY.run();
  }

  // Update current positions
  currentAzimuthPosition = targetAzimuthSteps;
  currentElevationPosition = targetElevationSteps;
}
