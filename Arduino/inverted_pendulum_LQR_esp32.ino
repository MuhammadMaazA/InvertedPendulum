#include "BluetoothSerial.h"

// ----- Bluetooth is initialized for optional debugging -----
BluetoothSerial SerialBT;

// ----- LQR Gains -----
float K1 = 10;
float K2 = 10.69922871;
float K3 = 67.1756893 * 10;
float K4 = 18.3842782 * 10;

// ----- Control Loop Timing -----
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 10; // milliseconds

// Variables to store sensor data received from Arduino
float pendulumAngle = 0.0;
float theta = 0.0;
float linearPosition = 0.0;
float dtheta = 0.0;
float linearVelocity = 0.0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Master");
  Serial.println("ESP32 Master: Setup complete. Waiting for sensor data from Arduino.");
  lastControlTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastControlTime >= controlInterval) {
    lastControlTime = currentTime;
    
    if (Serial.available() > 0) {
      String sensorLine = Serial.readStringUntil('\n');
      if (sensorLine.startsWith("SENSOR,")) {
        sensorLine.remove(0, 7);  // Remove "SENSOR,"
        int index1 = sensorLine.indexOf(',');
        int index2 = sensorLine.indexOf(',', index1 + 1);
        int index3 = sensorLine.indexOf(',', index2 + 1);
        int index4 = sensorLine.indexOf(',', index3 + 1);
        if (index1 != -1 && index2 != -1 && index3 != -1 && index4 != -1) {
          pendulumAngle = sensorLine.substring(0, index1).toFloat();
          theta = sensorLine.substring(index1 + 1, index2).toFloat();
          linearPosition = sensorLine.substring(index2 + 1, index3).toFloat();
          dtheta = sensorLine.substring(index3 + 1, index4).toFloat();
          linearVelocity = sensorLine.substring(index4 + 1).toFloat();
        }
        
        float u = 2.0 * (K1 * linearPosition) 
                + (K2 * linearVelocity)
                + (K3 * theta)
                + (K4 * dtheta);
                
        int motorCommand = (int) u;
        if (motorCommand > 255) motorCommand = 255;
        if (motorCommand < -255) motorCommand = -255;
        
        Serial.print("MOTOR,");
        Serial.println(motorCommand);
        
        Serial.print("Control: ");
        Serial.print(u);
        Serial.print(", Motor Command: ");
        Serial.println(motorCommand);
      }
    }
  }
}
