# RC Car Control System

Arduino-based remote-controlled (RC) car system with the ability to switch between **manual** and **autonomous** (WIP) modes.  
The system uses **RF signals** for manual control and **MPU9255 IMU** for autonomous navigation.

## Components

- **Arduino Board** (e.g., Arduino Uno)
- **L293D Motor Driver IC** (for controlling the motors)
- **MPU6050 IMU** (for reading accelerometer and gyroscope data)
- **HC-SR04 Ultrasonic Sensor** (for obstacle avoidance in autonomous mode / crash detection in manual mode)
- **433 MHz RF Transmitter/Receiver** (for receiving joystick signals)
- **Active Buzzer** (for indicating status and mode changes)
- **Motors** (for controlling the movement of the car)

## Features

- **Manual Mode**: Control the car using a joystick.
- **Autonomous Mode** (WIP): The car uses ultrasonic sensors and the IMU to navigate and avoid obstacles.
- **Mode Switching**: Toggle between manual and autonomous modes using a button.
- **Buzzer**: The buzzer sounds when a button is pressed or on mode change.

## Pin Configuration

The system uses the following pins:

- **Left Motor Pins**:
  - `LEFT_MOTOR_IN1`: Pin 2
  - `LEFT_MOTOR_IN2`: Pin 3
  - `LEFT_MOTOR_EN`: Pin 5
- **Right Motor Pins**:
  - `RIGHT_MOTOR_IN3`: Pin 4
  - `RIGHT_MOTOR_IN4`: Pin 7
  - `RIGHT_MOTOR_EN`: Pin 6
- **Additional Pins**:
  - `CS_PIN`: Pin 11  / Pin 12 (for SPI communication, default pins in RH_ASK library)
  - `BUZZER_PIN`: Pin 10
  - `MODE_BUTTON_PIN`: Pin 6 (button for mode switching)
  - `ULTRASONIC_TRIG`: Pin 12
  - `ULTRASONIC_ECHO`: Pin 13

## Libraries Used

- **SPI**: For SPI communication (used by the RadioHead library).
- **RH_ASK**: Library for transmitting / receiving data via 433MHz RF
- **Wire**: For I2C communication.
- **I2Cdev**: For communication with the MPU6050.

See platformio.ini file for exact libs used.

## Modes

### Manual Mode

In manual mode, the car is controlled by joystick data received via the RF receiver. The joystick's X and Y axes control the forward/backward movement and the left/right turn. The car ramps its speed gradually for smooth control.
Great for adjusting motor responses - the current cofiguration is unique to each build: powerpack config (2SxP, 3SxP...), motor torque, power, drive ratio, weight, grip etc. will all affect motor response and car movement.

### Autonomous Mode (WIP)

In autonomous mode, the car attempts to navigate using the ultrasonic sensor to avoid obstacles. The IMU can be used for more advanced steering corrections. This mode is still a work-in-progress and will be improved in future updates.

## Mode Switching

You can switch between manual and autonomous modes by pressing the **mode toggle button** (`MODE_BUTTON_PIN`). The system will give a short beep for manual mode and a long beep for autonomous mode.

## Immediate Fixes TODO

FS1000A is extremely noise-sensitive - add decoupling capacitors directly on module pins / as close to the module as possible:

Transmitter: 10μF electrolytic + 0.1μF ceramic between VCC/GND

    VCC   --  [10μF electrolytic] -- GND
          --  [0.1μF ceramic]   ---- GND

Receiver: 10nF ceramic between DATA and GND

    DATA  --  [10nF ceramic]    ---- GND

## Unit Testing

Unit tests are set up using **PlatformIO** and the **Unity** test framework.

To run the tests, run in terminal (project context):

```sh
pio test -e native
```

Add `-v (or -vv, -vvv)` flags to change verbosity. For `printf` logs to be visible, minimum `-v` is required.

This will execute all tests in the `test/` directory using the `native` environment.  
Refer to the [PlatformIO documentation](https://docs.platformio.org/en/latest/core/userguide/cmd_test.html) for more info.

## Bugs

Big'uns:

- MAJOR: Microcontroller turns off on left turn (previously on active braking from full speed). Voltage sag or noise.
  => add caps:

  | Location               | Type                   | Value(s)             | Comments
  | ---------------------- | ---------------------- | -------------------- | --------------------
  | L293D Vcc2 (pin 8)     | Electrolytic + ceramic | 470–1000 µF + 100 nF | VCC2 = motor +12V supply 
  | L293D Vcc1 (pin 16)    | Ceramic                | 100 nF               | VCC1 = +5V  supply (100nF to 1uF)
  | 5V Buck Output         | Electrolytic + ceramic | 1000 µF + 100 nF     | Larger cap to account for voltage sag due to current draw.
  | Radio / Receiver       | Electrolytic + ceramic | 100–220 µF + 100 nF  | Done 
  | Arduino Vcc (optional) | Ceramic                | 100 nF               | VCC pin (unregulated, RAW with 6-12V regulated without cap, less efficient)

  => increase 5V out to 5.05V - 5.1V on buck converter

- MAJOR: Joystick stick drift due to bad breadboard - perfboard eveything (+ joystick?)

Minor:
- Antenna issues. Trying different sizes and shapes - straight vs hellical. Boost Tx to ~10-12V.
- Loose shakey wires. finalize then perboard or pcb
  - 90% done for receiver

## Future Improvements

- Use MPU to keep car straight in manualMode in 
- Isolate MPU analogue signals (interference)
- Implement autonomous navigation using algorithms (e.g., PID control, SLAM).
- Add more sensors for better obstacle detection (though, sonic sensor may be enough, TBC).
- Refine the IMU-based steering correction for smoother control.
- Implement a more precise control system for sharper turns and better navigation in autonomous mode.
