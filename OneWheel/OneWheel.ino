#include <Wire.h>
#include <Adafruit_MPU9250.h>

Adafruit_MPU9250 mpu;

const int motorPwmPin = 9;      // PWM pin for motor speed
const int motorDirPin = 8;      // Direction pin
const int brakeSwitchPin = 7;   // Brake switch pin

float kp = 1.0, ki = 0.5, kd = 0.1;  // PID constants (adjust these based on testing)
float setpoint = 0.0;  // Target angle for balance
float integral = 0.0, previous_error = 0.0;  // PID variables

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU9250 sensor, check wiring!");
    while (1);
  }
  Serial.println("MPU9250 Found!");

  pinMode(motorPwmPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  pinMode(brakeSwitchPin, INPUT_PULLUP);  // Using internal pull-up resistor
}

// Function to get tilt angle from the MPU6050
float getTiltAngle() {
  mpu.read();
  
  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  
  // Calculate the tilt angle (adjust based on your orientation)
  float angle = atan2(ay, az) * 180 / PI;
  return angle;
}

// PID control function to determine the motor speed
float pidControl(float currentAngle) {
  float error = setpoint - currentAngle;
  integral += error;
  float derivative = error - previous_error;
  previous_error = error;
  
  float output = kp * error + ki * integral + kd * derivative;
  return output;
}

void loop() {
  // Read tilt angle
  float currentAngle = getTiltAngle();
  
  // Calculate PID output
  float pidOutput = pidControl(currentAngle);
  int motorSpeed = constrain(abs(pidOutput), 0, 255);  // Constrain to valid PWM range

  // Set motor direction based on tilt angle
  if (pidOutput > 0) {
    digitalWrite(motorDirPin, HIGH);  // Forward
  } else {
    digitalWrite(motorDirPin, LOW);   // Backward
  }

  // Check brake switch
  if (digitalRead(brakeSwitchPin) == LOW) {
    motorSpeed = 0;  // Stop motor if brake is pressed
  }
  
  // Output speed using PWM
  analogWrite(motorPwmPin, motorSpeed);

  // Optional: Print debug info
  Serial.print("Angle: "); Serial.print(currentAngle);
  Serial.print(" | Motor Speed: "); Serial.println(motorSpeed);
  
  delay(10);  // Small delay for stability
}
