![Foodpump3000](img/FP3000_logo.jpg)

The Food Pump 3000 is a standalone automatic cat feeder pump.
However, its true potential is realized when it is integrated into the [Purr Pleaser 3000](https://github.com/Poing3000/PurrPleaser3000), a fully automated cat feeding system designed for multiple cats.

The Food Pump 3000 is compatible with the Raspberry Pico W and can be conveniently programmed using the Arduino IDE.
All design parts can be found at...


## To do:
- [x] Finish code (see [FP3000_PicoW](https://github.com/Poing3000/FoodPump3000/tree/main/FP3000_PicoW))
- [ ] Finish Readme
  - [ ]  Extend description
  - [ ]  Explain usage/installation
  - [ ]  Explain the config
  - [ ]  Refer to CAD and building the pump (instructions).

# Main Features
## Return Values
- 0 = OK / BUSY
- 1 = OK / FINISHED
- 2 = ERROR
- 3 = WARNING

Warnings and errors can be checked via the functions CheckError() and CheckWarning().

## Setup Functions
- **FP3000::FP3000**: Constructor to initialize the pump.
- **SetupMotor**: Sets up the motor. Returns 1 if successful, 2 for error, and 3 for warning.
- **SetupScale**: Sets up the scale. Returns 1 if successful, 2 for error, and 3 for warning.

## Public Functions
- **CheckError**: Checks and resets the error status.
- **CheckWarning**: Checks and resets the warning status.
- **MotorTes**t: Tests the motor movement.
- **Prime**: Prepares the pump for feeding.
- **MoveCycle**: Executes a movement cycle.
- **MoveCycleAccurate**: Executes an accurate movement cycle.
- **EmptyScale**: Empties the scale.
- **MoveTo**: Moves the motor to a specific position.
- **HomeMotor**: Executes the motor homing function.
- **Test_Connection**: Tests the connection to the stepper motor driver.
- **AutotuneStall**: Performs an automatic tuning of the stall value.
