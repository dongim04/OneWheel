# OneWheel

## Components

- **Arduino Uno**
- **RioRand 400W 6-60V PWM Brushless Motor Driver**
- **MPU9250 Gyroscope + Accelerometer**
- **Brushless Motor**
- **Brake Switch**
- **Connecting Wires**
- **Power Supply** (for motors and Arduino)

## Hardware Connections

### 1. **Motor Driver (RioRand 400W)**
   - **M+ and M-**: Connect to the brushless motor.
   - **PWM Input**: Connect to Arduino pin **9** (PWM pin).
   - **DIR Input**: Connect to Arduino pin **8** (digital pin) to control motor direction.
   - **Power Terminals**: Connect to an appropriate power supply for your motor driver and motor (e.g., 24V or 36V).

### 2. **Gyroscope + Accelerometer (MPU9250)**
   - **SDA (Serial Data)**: Connect to Arduino pin **A4**.
   - **SCL (Serial Clock)**: Connect to Arduino pin **A5**.
   - **VCC**: Connect to Arduino **5V**.
   - **GND**: Connect to Arduino **GND**.

### 3. **Brake Switch**
   - Connect one side of the switch to Arduino pin **7** (digital pin).
   - Connect the other side to **GND**.
   - This configuration uses the internal pull-up resistor of the Arduino, so no external resistor is needed.

### 4. **Arduino Power Supply**
   - Ensure your Arduino is connected to a suitable power supply.

## Tuning the PID Constants

To ensure stable balancing, the **PID constants (kp, ki, kd)** need careful tuning. Here’s how:

1. **Initialize Kp**:
   - Start with only the proportional constant, **kp**.
   - Gradually increase **kp** until the hoverboard starts to balance but may oscillate. This constant helps respond proportionally to the tilt error.
   
2. **Add Kd**:
   - Increase the **kd** value to reduce oscillations. The derivative constant helps to slow down the response as the hoverboard reaches the setpoint, making the motion smoother.
   - Adjust until the oscillations minimize, allowing the hoverboard to balance more stably.

3. **Introduce Ki**:
   - If there’s a steady drift or small error, add **ki** in small increments. The integral constant helps correct any cumulative errors over time.
   - Be cautious, as too high a value can cause instability.

4. **Testing**:
   - Place the hoverboard on a stable surface and observe its behavior.
   - Adjust each constant incrementally and observe the impact. Aim for smooth, minimal oscillations with a quick response to tilt changes.

## Troubleshooting

- **Oscillations**: Reduce **kp** or increase **kd** if the hoverboard oscillates too much.
- **Slow Response**: Increase **kp** for a faster reaction to tilt changes.
- **Drift or Steady Error**: Increase **ki** in small increments.