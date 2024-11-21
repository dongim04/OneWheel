#include <Wire.h>

const int MPU_addr=0x68; // I2C address for the MPU
int16_t AcX,AcY,AcZ; // Variables for accelerometer X,Y,Z
 
int minVal=265; // Minimum value for angle
int maxVal=402; // Maximum value for angle 
 
double x; //x, y, and z angle, for MPU
double y;
double z;

const unsigned int BUFFER_SIZE = 16;          // Serial receive buffer size
const double BAUD_RATE = 115200;  // Serial port baud rate
const double WHEEL_DIAMETER_IN = 9;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 28.27;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 22.86;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 71.82;      // Motor wheel circumference (centimeters)


const int motorPwmPin = 9;      // PWM pin for motor speed
const int speedPin = 12;        // SC Speed Pulse Output from RioRand board
const int motorDirPin = 2;      // Direction pin
const int brakeSwitchPin = 7;   // Brake switch pin
bool _dir = 0;            // Direction of the motor
int correctAngle = 0;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(BAUD_RATE);

  pinMode(speedPin, INPUT);
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

  return x;
}

void loop() {

  // put your main code here, to run repeatedly:
  float _currentAngle = getTiltAngle();

  int PWM;

  if (_currentAngle <= 90.0) {
          int translateAngle = map(_currentAngle, 0, 45, 0, 255);
          PWM = constrain(translateAngle, 0, 255);
          digitalWrite(motorDirPin, HIGH);  // Forward
          _dir = 0;
      } else {
          int adjustedAngle = 360 - _currentAngle;
          int translateAngle = map(adjustedAngle, 0, 45, 0, 255);
          PWM = constrain(translateAngle, 0, 255);
          digitalWrite(motorDirPin, LOW);   // Backward
          _dir = 1;
      }
  /*
  int translateAngle = map(abs(_currentAngle), 0, 45, 0, 255);
  int PWM = constrain(abs(translateAngle), 0, 255);
  */
  analogWrite(motorPwmPin, PWM);
  
  double freq = 0;
  double rpm = 0;
  double mph = 0;
  double kph = 0;

  // Measure the time the pulse is Hi
  // If pulseIn times out then ontime will equal 0
  unsigned long ontime = pulseIn(speedPin, HIGH);

  // Only run calculations if ontime > 0
  if (ontime > 0)
  {
    // Calculate the period of the signal
    unsigned long period = ontime * 2;

    // Calculate the frequency
    freq = 1000000.0 / period;

    // Calculate the revolutions per minute
    rpm = freq / 45 * 60; 

    // Calculate the miles per hour (mph) based on the wheel diameter or circumference
    //mph = (WHEEL_DIAMETER_IN * PI * rpm * 60) / 63360;
    mph = (WHEEL_CIRCUMFERENCE_IN * rpm * 60) / 63360; 

    // Calculate the miles per hour (kph) based on the wheel diameter or circumference
    //kph = (WHEEL_DIAMETER_CM * PI * rpm * 60) / 1000;
    kph = (WHEEL_CIRCUMFERENCE_CM * rpm * 60) / 100000; 
  }   
  
  // Write data to the serial port
  Serial.print("PWM: "); Serial.print(PWM);
  Serial.print(" | Angle: "); Serial.print(_currentAngle);
  Serial.print(" | Direction: "); Serial.print(_dir);
  Serial.print((String)" | Freq:" + freq + " ");
  Serial.print((String)" | RPM:" + rpm + " ");
  Serial.print((String)" | MPH:" + mph + " ");
  Serial.println((String)" | KPH:" + kph + " ");
}
