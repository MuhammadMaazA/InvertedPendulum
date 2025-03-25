#include <Arduino.h>

// ----- Motor Driver Pins -----
#define J1_EN 13
#define J2_EN 12
#define J1_INT1 16
#define J1_INT2 17

#define J3_EN 14
#define J4_EN 27
#define J3_INT1 19
#define J3_INT2 23

// ----- Encoder Pins & Constants -----
#define PENDULUM_ENCODER_A 25
#define PENDULUM_ENCODER_B 26
#define PENDULUM_CPR 2000

#define MOTOR1_ENCODER_A 5
#define MOTOR1_ENCODER_B 18
#define MOTOR2_ENCODER_A 32
#define MOTOR2_ENCODER_B 33

#define MOTOR_ENCODER_CPR 48
#define GEAR_RATIO (9.7 * 2)
#define EFFECTIVE_CPR (MOTOR_ENCODER_CPR * GEAR_RATIO)

#define TIRE_DIAMETER 0.08  // meters
const float TIRE_CIRCUMFERENCE = PI * TIRE_DIAMETER;

// ----- Global Variables for Encoder Counts -----
volatile long encoderCountPendulum = 0;
volatile long encoderCountMotor1 = 0;
volatile long encoderCountMotor2 = 0;

// ----- Variables for Control -----
float setpointAngle = 0.0;   // Default target: upright (0°)
float lastTheta = 0.0;
float lastLinearPosition = 0.0;

// ----- Control Loop Timing -----
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 10; // milliseconds

// ----- LEDC PWM Setup -----
#define MOTOR_PWM_FREQ 5000
#define MOTOR_PWM_RESOLUTION 8   // 8-bit (0-255)
#define PWM_CHANNEL_J1_EN 0
#define PWM_CHANNEL_J2_EN 1
#define PWM_CHANNEL_J3_EN 2
#define PWM_CHANNEL_J4_EN 3

// --------------- Interrupt Service Routines ---------------
void IRAM_ATTR pendulumEncoderISR() {
  int A = digitalRead(PENDULUM_ENCODER_A);
  int B = digitalRead(PENDULUM_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoderCountPendulum--;
  } else {
    encoderCountPendulum++;
  }
}

void IRAM_ATTR motor1EncoderISR() {
  int A = digitalRead(MOTOR1_ENCODER_A);
  int B = digitalRead(MOTOR1_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoderCountMotor1--;
  } else {
    encoderCountMotor1++;
  }
}

void IRAM_ATTR motor2EncoderISR() {
  int A = digitalRead(MOTOR2_ENCODER_A);
  int B = digitalRead(MOTOR2_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    encoderCountMotor2--;
  } else {
    encoderCountMotor2++;
  }
}

// Helper: minimal angular difference in degrees (range -180 to 180)
float angleDifference(float target, float measured) {
  float error = target - measured;
  while (error > 180.0)  error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

// --------------- Motor Control ---------------
void setMotors(int motorCommand) {
  int pwmSignal = abs(motorCommand);
  if (pwmSignal > 255) pwmSignal = 255;
  
  // Driver 1 (normal wiring)
  if (motorCommand > 0) {
    digitalWrite(J1_INT1, HIGH);
    digitalWrite(J1_INT2, LOW);
  } else if (motorCommand < 0) {
    digitalWrite(J1_INT1, LOW);
    digitalWrite(J1_INT2, HIGH);
  } else {
    digitalWrite(J1_INT1, LOW);
    digitalWrite(J1_INT2, LOW);
  }
  
  // Driver 2 (flipped wiring)
  if (motorCommand > 0) {
    digitalWrite(J3_INT1, LOW);
    digitalWrite(J3_INT2, HIGH);
  } else if (motorCommand < 0) {
    digitalWrite(J3_INT1, HIGH);
    digitalWrite(J3_INT2, LOW);
  } else {
    digitalWrite(J3_INT1, LOW);
    digitalWrite(J3_INT2, LOW);
  }
  
  ledcWrite(PWM_CHANNEL_J1_EN, pwmSignal);
  ledcWrite(PWM_CHANNEL_J2_EN, pwmSignal);
  ledcWrite(PWM_CHANNEL_J3_EN, pwmSignal);
  ledcWrite(PWM_CHANNEL_J4_EN, pwmSignal);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(J1_EN, OUTPUT);
  pinMode(J2_EN, OUTPUT);
  pinMode(J1_INT1, OUTPUT);
  pinMode(J1_INT2, OUTPUT);
  
  pinMode(J3_EN, OUTPUT);
  pinMode(J4_EN, OUTPUT);
  pinMode(J3_INT1, OUTPUT);
  pinMode(J3_INT2, OUTPUT);
  
  ledcSetup(PWM_CHANNEL_J1_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_J2_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_J3_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_J4_EN, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  
  ledcAttachPin(J1_EN, PWM_CHANNEL_J1_EN);
  ledcAttachPin(J2_EN, PWM_CHANNEL_J2_EN);
  ledcAttachPin(J3_EN, PWM_CHANNEL_J3_EN);
  ledcAttachPin(J4_EN, PWM_CHANNEL_J4_EN);
  
  pinMode(PENDULUM_ENCODER_A, INPUT_PULLUP);
  pinMode(PENDULUM_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PENDULUM_ENCODER_A), pendulumEncoderISR, CHANGE);
  
  pinMode(MOTOR1_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_A), motor1EncoderISR, CHANGE);
  
  pinMode(MOTOR2_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR2_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_A), motor2EncoderISR, CHANGE);
  
  lastControlTime = millis();
  
  noInterrupts();
  long initCount = encoderCountPendulum;
  interrupts();
  lastTheta = (initCount / float(PENDULUM_CPR)) * 360.0;
  lastLinearPosition = 0.0;
  
  Serial.println("Arduino Slave: Setup complete.");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastControlTime >= controlInterval) {
    float dt = (currentTime - lastControlTime) / 1000.0;
    lastControlTime = currentTime;
    
    noInterrupts();
    long pendulumCount = encoderCountPendulum;
    long m1Count = encoderCountMotor1;
    long m2Count = encoderCountMotor2;
    interrupts();
    
    float pendulumAngle = (pendulumCount / float(PENDULUM_CPR)) * 360.0;
    while (pendulumAngle < 0)   pendulumAngle += 360.0;
    while (pendulumAngle >= 360.0) pendulumAngle -= 360.0;
    
    float theta = angleDifference(setpointAngle, pendulumAngle);
    
    if (abs(theta) > 15.0) {
      long avgMotorCount = (m1Count + m2Count) / 2;
      float linearPosition = (avgMotorCount / float(EFFECTIVE_CPR)) * TIRE_CIRCUMFERENCE;
      setMotors(0);
      Serial.println("Safety Cutoff: Pendulum angle > 15°. Control disabled.");
      
      Serial.print("SENSOR,");
      Serial.print(pendulumAngle, 2);
      Serial.print(",");
      Serial.print(theta, 2);
      Serial.print(",");
      Serial.print(linearPosition, 3);
      Serial.println(",SAFETY");
      return;
    }
    
    float dtheta = (theta - lastTheta) / dt;
    lastTheta = theta;
    
    float avgMotorCount = (m1Count + m2Count) / 2.0;
    float linearPosition = (avgMotorCount / float(EFFECTIVE_CPR)) * TIRE_CIRCUMFERENCE;
    float linearVelocity = (linearPosition - lastLinearPosition) / dt;
    lastLinearPosition = linearPosition;
    
    Serial.print("SENSOR,");
    Serial.print(pendulumAngle, 2);
    Serial.print(",");
    Serial.print(theta, 2);
    Serial.print(",");
    Serial.print(linearPosition, 3);
    Serial.print(",");
    Serial.print(dtheta, 2);
    Serial.print(",");
    Serial.println(linearVelocity, 3);
    
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n');
      if (input.startsWith("MOTOR,")) {
        int motorCommand = input.substring(6).toInt();
        setMotors(motorCommand);
      }
    }
  }
}
