#include <Wire.h>

const int MPU_addr=0x68; // I2C address for the MPU
int16_t AcX,AcY,AcZ; // Variables for accelerometer X,Y,Z
 
int minVal=265; // Minimum value for angle
int maxVal=402; // Maximum value for angle 
 
double x; //x, y, and z angle, for MPU
double y;
double z;

const int motorPwmPin = 9;      // PWM pin for motor speed
const int motorDirPin = 8;      // Direction pin
const int brakeSwitchPin = 7;   // Brake switch pin

float kp = 1.0, ki = 0.5, kd = 0.1;  // PID constants (adjust these based on testing)
float setpoint = 0.0;  // Target angle for balance
float integral = 0.0, previous_error = 0.0;  // PID variables

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);

  pinMode(motorPwmPin, OUTPUT);
  pinMode(motorDirPin, OUTPUT);
  pinMode(brakeSwitchPin, INPUT_PULLUP);  // Using internal pull-up resistor
}

// Function to get tilt angle from the MPU6050
float getTiltAngle() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
 
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  return z;
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
