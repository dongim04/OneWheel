#include <Wire.h>

// MPU6050 I2C address and accelerometer variables
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ;

// Angle calibration values
const int MIN_VAL = 265;
const int MAX_VAL = 402;

// Tilt angles
double x, y, z;

// Motor and control pins
const int MOTOR_PWM_PIN = 9;
const int SPEED_PIN = 12;
const int MOTOR_DIR_PIN = 2;
const int BRAKE_SWITCH_PIN = 7;

// Motor direction and PID control variables
bool motorDirection = false; // Forward: false, Backward: true
float kp = 1.0, ki = 0.5, kd = 0.1;
float setpoint = 0.0;
float integral = 0.0, previousError = 0.0;

// Wheel constants
const double BAUD_RATE = 115200;
const double WHEEL_CIRCUMFERENCE_IN = 28.27; // inches
const double WHEEL_CIRCUMFERENCE_CM = 71.82; // centimeters
// const double WHEEL_DIAMETER_IN = 9;       // inches
// const double WHEEL_DIAMETER_CM = 22.86;   // centimeters

void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(BAUD_RATE);

  pinMode(SPEED_PIN, INPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(BRAKE_SWITCH_PIN, INPUT_PULLUP); // Internal pull-up resistor
}

// Get tilt angle from MPU6050
float getTiltAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  int xAng = map(AcX, MIN_VAL, MAX_VAL, -90, 90);
  int yAng = map(AcY, MIN_VAL, MAX_VAL, -90, 90);
  int zAng = map(AcZ, MIN_VAL, MAX_VAL, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  return x;
}

// PID control to calculate motor speed
float pidControl(float currentAngle) {
  float error = setpoint - currentAngle;
  integral += error;
  float derivative = error - previousError;
  previousError = error;

  return kp * error + ki * integral + kd * derivative;
}

void loop() {
  float currentAngle = getTiltAngle();
  int motorPwmSpeed;

  // Determine motor speed and direction using PID
  if (currentAngle <= 90.0) {
    int translateAngle = map(currentAngle, 0, 45, 0, 255);
    float pidOutput = pidControl(translateAngle);
    motorPwmSpeed = constrain(abs(pidOutput), 0, 255);
    digitalWrite(MOTOR_DIR_PIN, HIGH); // Forward
    motorDirection = false;
  } else {
    int adjustedAngle = 360 - currentAngle;
    int translateAngle = map(adjustedAngle, 0, 45, 0, 255);
    float pidOutput = pidControl(translateAngle);
    motorPwmSpeed = constrain(abs(pidOutput), 0, 255);
    digitalWrite(MOTOR_DIR_PIN, LOW); // Backward
    motorDirection = true;
  }

  analogWrite(MOTOR_PWM_PIN, motorPwmSpeed);

  // Calculate speed metrics
  unsigned long onTime = pulseIn(SPEED_PIN, HIGH);
  double freq = 0, rpm = 0, mph = 0, kph = 0;

  if (onTime > 0) {
    unsigned long period = onTime * 2;
    freq = 1000000.0 / period;
    rpm = freq / 45 * 60;
    mph = (WHEEL_CIRCUMFERENCE_IN * rpm * 60) / 63360; // (WHEEL_DIAMETER_IN * PI * rpm * 60) / 63360
    kph = (WHEEL_CIRCUMFERENCE_CM * rpm * 60) / 100000; // (WHEEL_DIAMETER_CM * PI * rpm * 60) / 1000
  }

  // Serial output
  Serial.print("PWM: "); Serial.print(motorPwmSpeed);
  Serial.print(" | Angle: "); Serial.print(currentAngle);
  Serial.print(" | Direction: "); Serial.print(motorDirection ? "Backward" : "Forward");
  Serial.print(" | Freq: "); Serial.print(freq);
  Serial.print(" | RPM: "); Serial.print(rpm);
  Serial.print(" | MPH: "); Serial.print(mph);
  Serial.print(" | KPH: "); Serial.println(kph);

  // delay(10); // Small delay for stability
}
