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

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  serial_connection.begin(9600);
  stepperX.setMaxSpeed(1000); 
  stepperY.setMaxSpeed(1000);
  stepperX.setAcceleration(100); 
  stepperY.setAcceleration(100);
}

void loop() {
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

    lastMPUUpdate = currentMillis;
  }
  
  stepperX.run(); stepperY.run();
}
