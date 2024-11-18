// To use in serial monitor make sure you're at the correct baud rate and have serial monitor
// set to carriage return mode. To send a command to change the PWM send "PWM" followed by a comma 
// and your desired PWM value. EX: "PWM,25" This will set the PWM to 25. Other commands coming later.

// Constants
const unsigned int BUFFER_SIZE = 16;          // Serial receive buffer size
const double BAUD_RATE = 115200;  // Serial port baud rate
const double WHEEL_DIAMETER_IN = 9;            // Motor wheel diamater (inches)
const double WHEEL_CIRCUMFERENCE_IN = 28.27;     // Motor wheel circumference (inches)
const double WHEEL_DIAMETER_CM = 22.86;           // Motor wheel diamater (centimeters)
const double WHEEL_CIRCUMFERENCE_CM = 71.82;      // Motor wheel circumference (centimeters)
bool _dir = 0;            // Direction of the motor

// Pin declarations for UNO board
const int PIN_PWM = 9;                        // 490Hz pins 3, 9, 10, 11
const int PIN_SPEED = 12;         // SC Speed Pulse Output from RioRand board
const int PIN_DIR = 8;    // Motor direction signal

// Variables used in ReadFromSerial function
String _command = "";                         // Command received in Serial read command
int _data = 0;                                // Data received in Serial read command

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

  // Initialize PWM to off
  analogWrite(PIN_PWM, 0);

  // Initialize serial port
  Serial.begin(BAUD_RATE);
  Serial.println("---- Program Started ----");
}

void loop() 
{
  // Read serial data and set dataReceived to true if command is ready to be processed
  bool dataReceived = ReadFromSerial();

  // Process the received command if available
  if (dataReceived == true)
      ProcessCommand(_command, _data);
  
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
  Serial.print("PWM: "); Serial.print(_data);
  Serial.print(" | Direction: "); Serial.print(_dir);
  Serial.print((String)" | Freq:" + freq + " ");
  Serial.print((String)" | RPM:" + rpm + " ");
  Serial.print((String)" | MPH:" + mph + " ");
  Serial.println((String)" | KPH:" + kph + " ");
}

// Receives string data from the serial port
// Data should be in the format <command>,<data>
// Data should be terminated with a carriage return
// Function returns true if termination character received 
bool ReadFromSerial()
{    
    // Local variables   
    static String cmdBuffer;        // Stores the received command
    static String dataBuffer;       // Stores the received data
    static bool isCommand = true;   // Flag to store received bytes in command or data buffer
    byte recByte;                   // Byte received from the serial port
    
    // Check if any new data is available, if not exit
    if (Serial.available() == false)
        return false;
    
    // Read single byte from serial port
    recByte = Serial.read();
    
    // Check if byte is termination character (carriage return)
    if (recByte == '\r')
    {
        // Save buffers to global variables
        cmdBuffer.toUpperCase();
        _command = cmdBuffer;
        _data = dataBuffer.toInt();
      
        // Write what was received back to the serial port
        Serial.print("Received: "); 
        Serial.print(_command); 
        Serial.print(",");
        Serial.println(_data);
      
        // Clear local variables
        cmdBuffer = "";
        dataBuffer = "";
        isCommand = true;
      
        return true;
    }
    
    // Check if byte is a comma which separates the command from the data
    if ((char)recByte == ',')
    {
        isCommand = false;  // Next byte will be a data byte
        return false;
    }

    // Save data to one of the receive buffers
    if (isCommand)
        cmdBuffer += (char)recByte;
    else
        dataBuffer += (char)recByte;
    
    return false;
}

// Processes the command and data, sends result to serial port
void ProcessCommand(String command, int data)
{  
    // Process PWM command
    if (command == "PWM")
    {
      Serial.print("Setting pwm:  ");
      Serial.println(data);
      analogWrite(PIN_PWM, data);
    }
}
