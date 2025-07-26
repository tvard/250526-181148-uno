# RC Car Control System

This is a PlatformIO / Arduino 

## Features

- **Manual Mode**: Control the car using a joystick.
- **Autonomous Mode** (WIP): The car uses ultrasonic sensors and the IMU to navigate and avoid obstacles.

See platformio.ini file for exact libs used.

## Modes

### Manual Mode

In manual mode, the car is controlled by joystick data received via the RF receiver. The joystick's X and Y axes control the forward/backward movement and the left/right turn. The car ramps its speed gradually for smooth control.

### Autonomous Mode (WIP)

In autonomous mode, the car attempts to navigate using the ultrasonic sensor to avoid obstacles. The IMU can be used for more advanced steering corrections. This mode is still a work-in-progress and will be improved in future updates.

<!-- ## Mode Switching

You can switch between manual and autonomous modes by pressing the **mode toggle button** (`MODE_BUTTON_PIN`). The system will give a short beep for manual mode and a long beep for autonomous mode. -->


## Unit Testing

Unit tests are set up using **PlatformIO** and the **Unity** test framework.

To run the tests, run in terminal (project context):

```sh
pio test -e native
```

Add `-v (or -vv, -vvv)` flags to change verbosity. For `printf` logs to be visible, minimum `-v` is required.

This will execute all tests in the `test/` directory using the `native` environment.  
Refer to the [PlatformIO documentation](https://docs.platformio.org/en/latest/core/userguide/cmd_test.html) for more info.

#####  Bugs

### Big'uns:

- ~~MAJOR: Microcontroller turns off on left turn (previously on active braking from full speed). Voltage sag or noise.~~
  --> solved - wrong PWM pin selected. No voltage sag detected after diagnostics. 
  --> Regardless, swapping to NRF24L01 for reliability and data rate.
  --> ~~increase 5V out to 5.05V - 5.1V on buck converter~~ ==> already at 5.05V. No change.

- MAJOR: Joystick stick drift due to bad breadboard - perfboard eveything
  => ~~CAR done~~, Controller pending.

Minor:

## Future Improvements

### NOW
~~- Use MPU to keep car straight in manualMode in ~~ => Swapped to RWD, FWD was unbalanced + had tracktion issues
- Swap RF modules to NRF24L01 tranceiver modules
- Add voltage divider for battery voltage measurement
  --> controller module - 18650 is 3.7V => no need for voltage divider
- Add OLED screen to controller
- Improve unit tests for car. Refactor to data driven (JSON test-case datasets)
- Implement unit tests for controller.
- Currently - small level of RF interference due to motor noise. Swapping to NRF24L01 likely will solve issue.

### LATER
- Implement autonomous navigation using algorithms (e.g., PID control, SLAM).
- Add more sensors for better obstacle detection (though, sonic sensor may be enough, TBC).
- Refine the IMU-based steering correction for smoother control.
- Implement a more precise control system for sharper turns and better navigation in autonomous mode.
- Isolate MPU analogue signals (interference)

