#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

bool overrideActive = false;
int overrideState = 0;
unsigned long overrideStartTime = 0;
float setpointAngle = 0.0;

void setup() {
  Serial.begin(115200);
  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Bluetooth initialization failed!");
  } else {
    Serial.println("Bluetooth initialized. Waiting for data...");
  }
}

void loop() {
  if (SerialBT.available()) {
    char incomingChar = SerialBT.read();
    if (incomingChar == 'g' && !overrideActive) {
      setpointAngle = 1.0;
      overrideActive = true;
      overrideState = 1;
      overrideStartTime = millis();
      Serial.println("Received g: Target angle set to 2° for 2 seconds");
    }
  }
}
