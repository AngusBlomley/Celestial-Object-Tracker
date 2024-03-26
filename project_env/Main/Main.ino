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

//Variables
unsigned long lastMPUUpdate = 0;
long targetX = 0, targetY = 0;
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

void calibrateAccelerometer() {
    long xAccum = 0, yAccum = 0, zAccum = 0;
    int numReadings = 1000;

    for (int i = 0; i < numReadings; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        xAccum += ax;
        yAccum += ay;
        zAccum += az;
        delay(2);  // Short delay before the next reading
    }

    ax_offset = xAccum / numReadings;
    ay_offset = yAccum / numReadings;
    az_offset = zAccum / numReadings + 16384; // Assuming the scale is +/-2g, adjust if different
}

void calibrateGyroscope() {
    long xGyroAccum = 0, yGyroAccum = 0, zGyroAccum = 0;
    int numReadings = 1000;

    for (int i = 0; i < numReadings; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        xGyroAccum += gx;
        yGyroAccum += gy;
        zGyroAccum += gz;
        delay(2);  // Short delay before the next reading
    }

    gx_offset = xGyroAccum / numReadings;
    gy_offset = yGyroAccum / numReadings;
    gz_offset = zGyroAccum / numReadings;
}

void applyGyroOffsets(int16_t &gx, int16_t &gy, int16_t &gz) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  serial_connection.begin(9600);
  calibrateAccelerometer();
  stepperX.setMaxSpeed(1000); 
  stepperY.setMaxSpeed(1000);
  stepperX.setAcceleration(100); 
  stepperY.setAcceleration(100);
}

void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  applyGyroOffsets(gx, gy, gz);

  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;

  if (serial_connection.available() > 0 && gps.encode(serial_connection.read())) {
    if (gps.location.isUpdated()) {
      Serial.print("Lat: "); 
      Serial.println(gps.location.lat(), 6);
      Serial.print("Lng: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("Altitude: "); 
      Serial.println(gps.altitude.meters());
      Serial.print("Satellites: "); 
      Serial.println(gps.satellites.value());
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastMPUUpdate >= 500) {
    int16_t ax, ay, az, gx, gy, gz; // Simplified timing check for demonstration. Consider using a variable to track last update time.
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("MPU,"); 
    Serial.print(ax); Serial.print(","); 
    Serial.print(ay); Serial.print(","); 
    Serial.print(az); Serial.print(","); 
    Serial.print(gx); Serial.print(","); 
    Serial.print(gy); Serial.print(","); 
    Serial.println(gz);

    lastMPUUpdate = currentMillis;
  }

  // Check for incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the command
    if (command.startsWith("MOVE,")) {
      int commaIndex = command.indexOf(',');
      int secondCommaIndex = command.indexOf(',', commaIndex + 1);
      
      targetX = command.substring(commaIndex + 1, secondCommaIndex).toInt();
      targetY = command.substring(secondCommaIndex + 1).toInt();
      
      // Move the X and Y steppers to the target positions
      stepperX.moveTo(targetX);
      stepperY.moveTo(targetY);
    } else if (command.startsWith("REQUEST_MPU")) {
      // Send the MPU data
      sendMPUData();
    }
  }

  // Regularly call the run function in order to actually move the motors
  stepperX.run();
  stepperY.run();
}

void sendMPUData() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print("MPU,");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);
}