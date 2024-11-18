// Pin declarations for UNO board
const int PIN_DIR = 8;    // Motor direction signal
const int PIN_PWM = 9;    // 490Hz pins 3, 9, 10, 11
const int DELAY = 100;     // Amount of time to delay between increments

// Variables
int _pwmCtrl = 0;         // PWM control signal 0-255
int _pwmInc = 1;          // Increment the PWM by this amount 
bool _dir = 0;            // Direction of the motor

// Constants
const double BAUD_RATE = 115200;  // Serial port baud rate
const double WHEEL_DIAMETER_IN = 9;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 28.27;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 22.86;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 71.82;      // Motor wheel circumference (centimeters)

// Pin declarations for UNO board
const int PIN_SPEED = 12;         // SC Speed Pulse Output from RioRand board

void setup() 
{
  // Set pin directions
  pinMode(PIN_SPEED, INPUT);

  // Initialize serial port
  Serial.begin(BAUD_RATE);

  // Set DIR pin to output
  pinMode(PIN_DIR, OUTPUT);
  
  // Set PWM pin to output
  pinMode(PIN_PWM, OUTPUT);
  Serial.println("---- Program Started ----");
}

void loop() 
{
  // Increment or Decrement the pwm control
  _pwmCtrl += _pwmInc;

  // Change increment to negative if we are too high
  if (_pwmCtrl >= 255)
      _pwmInc = -1;

  // Change increment to positive if we are at zero
  // Change direction of motor if we are at zero
  if (_pwmCtrl <= 0)
  {
      _pwmInc = 1;
      _dir = !_dir;
  }

  // Set the PWM output

  _pwmCtrl = 20;
  analogWrite(PIN_PWM, _pwmCtrl);

  // Set the direction
  digitalWrite(PIN_DIR, _dir);
  Serial.print("PWM: "); Serial.print(_pwmCtrl);
  Serial.print(" | Direction: "); Serial.print(_dir);

  double freq = 0;
  double rpm = 0;
  double mph = 0;
  double kph = 0;

  // Measure the time the pulse is Hi
  // If pulseIn times out then ontime will equal 0
  unsigned long ontime = pulseIn(PIN_SPEED, HIGH);

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
  Serial.print((String)" | Freq:" + freq + " ");
  Serial.print((String)" | RPM:" + rpm + " ");
  Serial.print((String)" | MPH:" + mph + " ");
  Serial.println((String)" | KPH:" + kph + " ");
  // Add a delay so we don't change too fast
}

