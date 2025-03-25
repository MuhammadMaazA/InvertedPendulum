#include <Wire.h>
#include "BluetoothSerial.h"

// -------- Bluetooth Setup (optional) --------
BluetoothSerial SerialBT;

// -------- Motor Driver Pin Definitions --------
#define MOTOR_DRIVER_EN1 13
#define MOTOR_DRIVER_EN2 12
#define MOTOR_DRIVER_IN1_A 16
#define MOTOR_DRIVER_IN1_B 17

#define MOTOR_DRIVER_EN3 14
#define MOTOR_DRIVER_EN4 27
#define MOTOR_DRIVER_IN2_A 19
#define MOTOR_DRIVER_IN2_B 23

// -------- Encoder Pin Definitions & Constants --------
#define PENDULUM_ENCODER_PIN_A 25
#define PENDULUM_ENCODER_PIN_B 26
#define PENDULUM_CPR 2000  // Counts per revolution

#define MOTOR1_ENCODER_PIN_A 5
#define MOTOR1_ENCODER_PIN_B 18
#define MOTOR2_ENCODER_PIN_A 32
#define MOTOR2_ENCODER_PIN_B 33

#define MOTOR_ENCODER_CPR 48
#define GEAR_RATIO (9.7 * 2)
#define EFFECTIVE_CPR (MOTOR_ENCODER_CPR * GEAR_RATIO)

#define TIRE_DIAMETER 0.08  // in meters
const float TIRE_CIRCUMFERENCE = PI * TIRE_DIAMETER;

// -------- Global Variables for Encoder Counts --------
volatile long pendulumEncoderCount = 0;
volatile long motor1EncoderCount = 0;
volatile long motor2EncoderCount = 0;

// -------- I2C Configuration --------
#define I2C_SLAVE_ADDRESS 0x08

// -------- Sensor Values --------
float currentPendulumAngle = 0.0;  // in degrees
float currentLinearPosition = 0.0; // in meters

// -------- LEDC PWM Setup (for motor drivers) --------
#define MOTOR_PWM_FREQUENCY 5000
#define MOTOR_PWM_RESOLUTION 8   // 8-bit PWM resolution (0-255)
#define PWM_CHANNEL_EN1 0
#define PWM_CHANNEL_EN2 1
#define PWM_CHANNEL_EN3 2
#define PWM_CHANNEL_EN4 3

// ----------------- Interrupt Service Routines -----------------

// ISR for the pendulum encoder
void IRAM_ATTR pendulumEncoderISR() {
  int pinA = digitalRead(PENDULUM_ENCODER_PIN_A);
  int pinB = digitalRead(PENDULUM_ENCODER_PIN_B);
  if ((pinA == HIGH) != (pinB == LOW)) {
    pendulumEncoderCount--;
  } else {
    pendulumEncoderCount++;
  }
}

// ISR for motor 1 encoder
void IRAM_ATTR motor1EncoderISR() {
  int pinA = digitalRead(MOTOR1_ENCODER_PIN_A);
  int pinB = digitalRead(MOTOR1_ENCODER_PIN_B);
  if ((pinA == HIGH) != (pinB == LOW)) {
    motor1EncoderCount--;
  } else {
    motor1EncoderCount++;
  }
}

// ISR for motor 2 encoder
void IRAM_ATTR motor2EncoderISR() {
  int pinA = digitalRead(MOTOR2_ENCODER_PIN_A);
  int pinB = digitalRead(MOTOR2_ENCODER_PIN_B);
  if ((pinA == HIGH) != (pinB == LOW)) {
    motor2EncoderCount--;
  } else {
    motor2EncoderCount++;
  }
}

// ----------------- Motor Control Function -----------------

// Drives the motor drivers based on the provided motor command
// motorCommand should be in the range [-255, 255]
void setMotorOutputs(int motorCommand) {
  int pwmValue = abs(motorCommand);
  if (pwmValue > 255) pwmValue = 255;
  
  // Set directions for first motor driver (normal wiring)
  if (motorCommand > 0) {
    digitalWrite(MOTOR_DRIVER_IN1_A, HIGH);
    digitalWrite(MOTOR_DRIVER_IN1_B, LOW);
  } else if (motorCommand < 0) {
    digitalWrite(MOTOR_DRIVER_IN1_A, LOW);
    digitalWrite(MOTOR_DRIVER_IN1_B, HIGH);
  } else {
    digitalWrite(MOTOR_DRIVER_IN1_A, LOW);
    digitalWrite(MOTOR_DRIVER_IN1_B, LOW);
  }
  
  // Set directions for second motor driver (flipped wiring)
  if (motorCommand > 0) {
    digitalWrite(MOTOR_DRIVER_IN2_A, LOW);
    digitalWrite(MOTOR_DRIVER_IN2_B, HIGH);
  } else if (motorCommand < 0) {
    digitalWrite(MOTOR_DRIVER_IN2_A, HIGH);
    digitalWrite(MOTOR_DRIVER_IN2_B, LOW);
  } else {
    digitalWrite(MOTOR_DRIVER_IN2_A, LOW);
    digitalWrite(MOTOR_DRIVER_IN2_B, LOW);
  }
  
  // Write the PWM signal to all enable channels
  ledcWrite(PWM_CHANNEL_EN1, pwmValue);
  ledcWrite(PWM_CHANNEL_EN2, pwmValue);
  ledcWrite(PWM_CHANNEL_EN3, pwmValue);
  ledcWrite(PWM_CHANNEL_EN4, pwmValue);
}

// ----------------- I2C Callbacks -----------------

// Callback when master sends data (motor command)
void onI2CReceive(int numBytes) {
  if (numBytes < 2) return; // We expect at least 2 bytes (int16_t)
  int motorCommand = 0;
  motorCommand = Wire.read() << 8; // High byte
  motorCommand |= Wire.read();     // Low byte
  setMotorOutputs(motorCommand);
}

// Callback when master requests sensor data
void onI2CRequest() {
  // Get current encoder counts atomically
  noInterrupts();
  long pendulumCount = pendulumEncoderCount;
  long motor1Count = motor1EncoderCount;
  long motor2Count = motor2EncoderCount;
  interrupts();
  
  // Calculate the pendulum angle (in degrees)
  currentPendulumAngle = (pendulumCount / float(PENDULUM_CPR)) * 360.0;
  while (currentPendulumAngle < 0)   currentPendulumAngle += 360.0;
  while (currentPendulumAngle >= 360.0) currentPendulumAngle -= 360.0;
  
  // Calculate the linear position using the average of motor counts
  float averageMotorCount = (motor1Count + motor2Count) / 2.0;
  currentLinearPosition = (averageMotorCount / EFFECTIVE_CPR) * TIRE_CIRCUMFERENCE;
  
  // Pack the sensor data (two floats) into an 8-byte stream
  union {
    float value;
    byte bytes[4];
  } dataUnion;
  
  // Send pendulum angle (4 bytes)
  dataUnion.value = currentPendulumAngle;
  for (int i = 0; i < 4; i++) {
    Wire.write(dataUnion.bytes[i]);
  }
  // Send linear position (4 bytes)
  dataUnion.value = currentLinearPosition;
  for (int i = 0; i < 4; i++) {
    Wire.write(dataUnion.bytes[i]);
  }
}

// ----------------- Setup & Loop -----------------

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth (optional)
  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Bluetooth initialization failed!");
  } else {
    Serial.println("Bluetooth initialized.");
  }
  
  // Setup motor driver pins
  pinMode(MOTOR_DRIVER_EN1, OUTPUT);
  pinMode(MOTOR_DRIVER_EN2, OUTPUT);
  pinMode(MOTOR_DRIVER_IN1_A, OUTPUT);
  pinMode(MOTOR_DRIVER_IN1_B, OUTPUT);
  
  pinMode(MOTOR_DRIVER_EN3, OUTPUT);
  pinMode(MOTOR_DRIVER_EN4, OUTPUT);
  pinMode(MOTOR_DRIVER_IN2_A, OUTPUT);
  pinMode(MOTOR_DRIVER_IN2_B, OUTPUT);
  
  // Setup PWM channels for motor control
  ledcSetup(PWM_CHANNEL_EN1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_EN2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_EN3, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_EN4, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_DRIVER_EN1, PWM_CHANNEL_EN1);
  ledcAttachPin(MOTOR_DRIVER_EN2, PWM_CHANNEL_EN2);
  ledcAttachPin(MOTOR_DRIVER_EN3, PWM_CHANNEL_EN3);
  ledcAttachPin(MOTOR_DRIVER_EN4, PWM_CHANNEL_EN4);
  
  // Setup encoder pins and attach interrupts
  pinMode(PENDULUM_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(PENDULUM_ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PENDULUM_ENCODER_PIN_A), pendulumEncoderISR, CHANGE);
  
  pinMode(MOTOR1_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_PIN_A), motor1EncoderISR, CHANGE);
  
  pinMode(MOTOR2_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTOR2_ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_PIN_A), motor2EncoderISR, CHANGE);
  
  // Initialize I2C in slave mode
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  
  Serial.println("ESP32: Motor control and sensor acquisition ready.");
}

void loop() {
  // The ESP32 continuously updates sensor readings and responds to I2C events.
  // Additional tasks can be added here if necessary.
  delay(5); // Small delay to yield CPU time
}
