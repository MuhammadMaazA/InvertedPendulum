#include <Wire.h>

// I2C address of the ESP32 acting as the slave device
#define ESP32_SLAVE_ADDR 0x08

// Control loop timing (in milliseconds)
const unsigned long CONTROL_INTERVAL_MS = 10;

// LQR gains (tune these for your system)
float K1 = 10.0;
float K2 = 10.69922871;
float K3 = 671.756893;   // (67.1756893 * 10)
float K4 = 183.842782;   // (18.3842782 * 10)

// Desired pendulum angle (setpoint in degrees) and override parameters
float desiredPendulumAngle = 0.0;  // Default is upright (0°)
bool overrideActive = false;
int overrideStage = 0;             // 0: none, 1: +2°, 2: -2°
unsigned long overrideStartTime = 0;

// Variables for state estimation (for computing velocities)
float previousThetaError = 0.0;
float previousLinearPosition = 0.0;
unsigned long lastControlTime = 0;

// ----------------- Helper Functions -----------------

// Computes the minimal difference between target and measured angle (wraps around at 360°)
float calculateAngleError(float target, float measured) {
  float error = target - measured;
  while (error > 180.0)  error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

// Request sensor data (pendulum angle and linear position) from the ESP32 over I2C
bool requestSensorData(float &pendulumAngle, float &linearPosition) {
  // Request 8 bytes: two floats (4 bytes each)
  Wire.requestFrom(ESP32_SLAVE_ADDR, 8);
  if (Wire.available() < 8) {
    return false; // Not enough data received
  }
  
  union {
    byte bytes[4];
    float value;
  } data;
  
  // Retrieve pendulum angle (4 bytes)
  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }
  pendulumAngle = data.value;
  
  // Retrieve linear position (4 bytes)
  for (int i = 0; i < 4; i++) {
    data.bytes[i] = Wire.read();
  }
  linearPosition = data.value;
  
  return true;
}

// Send the computed motor command (an int16_t value) to the ESP32 over I2C
void sendMotorCommand(int motorCommand) {
  Wire.beginTransmission(ESP32_SLAVE_ADDR);
  // Transmit high byte then low byte
  Wire.write((motorCommand >> 8) & 0xFF);
  Wire.write(motorCommand & 0xFF);
  Wire.endTransmission();
}

// ----------------- Main Program -----------------

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Start I2C as master
  lastControlTime = millis();
  
  Serial.println("Arduino Uno: Control computation started.");
}

void loop() {
  unsigned long currentTime = millis();
  
  // --- Optional: Check for serial input to trigger an override ---
  // If a 'g' is received via Serial, start an override sequence.
  if (Serial.available()) {
    char inputChar = Serial.read();
    if (inputChar == 'g' && !overrideActive) {
      desiredPendulumAngle = 2.0;  // Override: set target to +2°
      overrideActive = true;
      overrideStage = 1;
      overrideStartTime = currentTime;
      Serial.println("Override activated: Target angle set to +2°");
    }
  }
  
  // --- Override State Management ---
  if (overrideActive) {
    if (overrideStage == 1 && (currentTime - overrideStartTime >= 3000)) {
      desiredPendulumAngle = -3.0;  // Change target to -2° (adjust value as needed)
      overrideStage = 2;
      overrideStartTime = currentTime;
      Serial.println("Override updated: Target angle set to -2°");
    } else if (overrideStage == 2 && (currentTime - overrideStartTime >= 2000)) {
      desiredPendulumAngle = 0.0;   // Return to default upright position
      overrideActive = false;
      overrideStage = 0;
      Serial.println("Override ended: Target angle reset to 0°");
    }
  }
  
  // --- Run control loop at defined interval ---
  if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (currentTime - lastControlTime) / 1000.0; // Convert to seconds
    lastControlTime = currentTime;
    
    float measuredPendulumAngle, measuredLinearPosition;
    if (!requestSensorData(measuredPendulumAngle, measuredLinearPosition)) {
      Serial.println("Error: Failed to retrieve sensor data from ESP32.");
      return;
    }
    
    // Calculate error between desired and measured pendulum angles
    float thetaError = calculateAngleError(desiredPendulumAngle, measuredPendulumAngle);
    
    // Calculate angular velocity (d(theta)/dt)
    float angularVelocity = (thetaError - previousThetaError) / dt;
    previousThetaError = thetaError;
    
    // Calculate linear velocity based on change in position
    float linearVelocity = (measuredLinearPosition - previousLinearPosition) / dt;
    previousLinearPosition = measuredLinearPosition;
    
    // --- Safety Cutoff: If the pendulum deviates too much, send a zero command ---
    if (abs(thetaError) > 15.0) {
      sendMotorCommand(0);
      Serial.println("Safety Cutoff: Pendulum deviation >15°. Motor command set to 0.");
      return;
    }
    
    // --- LQR Control Law ---
    // The control signal is computed using state feedback
    float controlSignal = 2.0 * (K1 * measuredLinearPosition 
                                + K2 * linearVelocity 
                                + K3 * thetaError 
                                + K4 * angularVelocity);
    
    // Convert the control signal to a motor command in the range [-255, 255]
    int motorCommand = (int)controlSignal;
    if (motorCommand > 255)  motorCommand = 255;
    if (motorCommand < -255) motorCommand = -255;
    
    // Send the computed motor command to the ESP32
    sendMotorCommand(motorCommand);
    
    // Debug output for monitoring
    Serial.print("x = ");
    Serial.print(measuredLinearPosition, 3);
    Serial.print(" m, dx = ");
    Serial.print(linearVelocity, 3);
    Serial.print(" m/s, theta_error = ");
    Serial.print(thetaError, 2);
    Serial.print("°, dtheta/dt = ");
    Serial.print(angularVelocity, 2);
    Serial.print(" deg/s, motorCommand = ");
    Serial.println(motorCommand);
  }
}
