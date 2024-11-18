/*   PWM_Oscillating
 *   Increases the PWM duty cycle from 0 to 100% and then back down
 *   Changes the direction of the wheel when at 0 duty cycle
 *     
 *   created 2021
 *   Mad-EE  (Mad Electrical Engineer)
 *   www.mad-ee.com
 *   
 *   This example code is in the public domain.
 *   
 *   Platform:  Arduino UNO
 */

const int PIN_DIR = 12;    // Motor direction signal
const int PIN_PWM = 9;    // 490Hz pins 3, 9, 10, 11
const int DELAY = 20;     // Delay time between increments

int _pwmCtrl = 0;         // PWM control signal 0-255
int _pwmInc = 1;          // Increment the PWM by this amount 
bool _dir = 0;            // Direction of the motor

void setup() 
{
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);
}

void loop() 
{
//     _pwmCtrl += _pwmInc; // In/Decrement the pwm control

//     if (_pwmCtrl >= 255) // If we are too high
//         _pwmInc = -1;    // Change increment to negative

//     if (_pwmCtrl <= 0) // If we are at zero
//     {
//         _pwmInc = 1; // Change increment to positive
//         _dir = !_dir; // Change direction of motor
//     }

    analogWrite(PIN_PWM, 70); // Set the PWM output
    digitalWrite(PIN_DIR, _dir); // Set the direction

    delay(DELAY);
}