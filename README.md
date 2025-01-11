![Foodpump3000](img/FP3000_logo.jpg)

**NOTE**, although the libray is working and can be used to individual needs, the following Readme is not finished yet. I will improve it in near future, and further will implement a better example for a simple standalone Pump (e.g. not using an MCP).
Further, please note that I am not a professional - this is a hobby project - so don't be rude against imperfections in coding etc.

The Food Pump 3000 is a standalone automatic cat feeder pump.
However, its true potential is realized when it is integrated into the [Purr Pleaser 3000](https://github.com/Poing3000/PurrPleaser3000), a fully automated cat feeding system designed for multiple cats.

The Food Pump 3000 is compatible with the Raspberry Pico W and can be conveniently programmed using the Arduino IDE. Further it is using TMC2209 stepper drivers.
All design parts can be found at: [PRINTABLES](https://www.printables.com/model/1144252-food-pump-3000-cat-food-pump).

A more detailed description on the CatFeeder Project can be found at: [CLIMBING-ENGINEER.COM](https://www.climbing-engineer.com/automatic-cat-feeder.html).

# Main Features
In the folder [FP3000_PicoW](FP3000_PicoW) there are three files and an src folder.

**FP3000.h plus FP3000.cpp** form the FoodPump library that can be used in individual projects. However, the library does not function solely on its own, it's got dependencies.
One dependency - SpeedyStepper4Purr - is already included in the [src](FP3000_PicoW/src) folder, though the following external dependencies need to be downloaded seperatly (there are more though should be covered by arduinio - see code for mor information):

## External Dependencies:
  - [TMCStepper](https://github.com/teemuatlut/TMCStepper)
  - [MCP23017](https://github.com/wollewald/MCP23017_WE/tree/master)
  - [HX711](https://github.com/RobTillaart/HX711)

## Exmample
Further there is the file **FP3000_PicoW.ino**, this is an example on how the libray may be used. Note that it was intitially written to support the [Purr Pleaser 3000](https://github.com/Poing3000/PurrPleaser3000), which is able to automatically feed to cats independently, so it is a bit biased in a way and uses the MCP to extend GPIO intputs to the Rapsberry Pico - this is surley not need in smaller projects where only one Pump is used. So it is **only** an example and does not show any way how the library may be use - e.g. if an MCP is not wanted it just can be left out (though the libray still depends on the MCP library).

So to wrap it up; the easiest way is to download [FP3000_PicoW](FP3000_PicoW) and open "FP3000_PicoW.ino" with the arduino IDE and to have a play / adapt it to individual needs.

# Usage
The follwowing briefly describes the usage of the **FP3000** library, further details can be found in the comments in the code itself.
Note the physical use (e.g. hardware wiring) depends on the individual implementation. The example **FP3000_PicoW.ino** provides an example on which pins may be used etc (see #define sections). In case further help is needed I suggest to have a look at the [Purr Pleaser 3000](https://github.com/Poing3000/PurrPleaser3000) project or for Single feeding: [SINGLE FEEDER]https://www.climbing-engineer.com/automatic-single-cat-feeder-2.html. These projects have a finsihed hardware implementation (schematics etc.) and show how **FP3000** can be used.

## Constructor
To initialize the FP3000 object, use the following (long) constructor:
```c++
FP3000::FP3000(byte MotorNumber, long std_distance, long max_range, long dir_home, float stepper_speed,
    uint8_t stall_val, bool auto_stall_red, HardwareSerial &serial, float driver_rsense, uint8_t driver_address,
    MCP23017 &mcpRef, bool use_expander, byte mcp_INTA);
```
### Parameters:
- **MotorNumber**: Unique device identifier for the individual pump or dumper motor (e.g. first motor 0, second motor 1).
- **std_distance**: Standard range (steps) the slider should moves when feeding (4600 is good) - equivilant for dumper (scale).
- **max_range**: Max range (steps) the slider can move inside the pump (6000 is good) - equivilant for dumper (scale).
- **dir_home**: Direction for homing the stepper (1 = CW, -1 = CCW).
- **stepper_speed**: Speed (steps/s) (10000 is good)
- **stall_val**: Stall threshold [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value. Set to 0 if you want stall values loaded from file.
- **auto_stall_red**: This allows for automatic stall threshold reduction / adaption (not part of TMCStepper library).
- **serial**: HardwareSerial port to connect to driver (e.g. Serial1 - TX: 0, RX: 1)
- **driver_rsense**: Sense resistor value of the driver fur current cal (e.g. 0.11f)
- **driver_address**: Drivers address (e.g. 0b01: MS1 is LOW and MS2 is HIGH)
- **mcpRef**: Reference to the MCP23017 object (e.g. mcp).
- **use_expander**: Use expander (true) or not (false); if false other mcp settings are ignored.
- **mcp_INTA**: Interrupt pin A - IMPORTANT: SET TO 1 FOR HOMING WITHOUT MCP23017!!!

# Main Features
## Return Values
- 0 = OK / BUSY
- 1 = OK / FINISHED
- 2 = ERROR
- 3 = WARNING

Warnings and errors can be checked via the functions CheckError() and CheckWarning().

## Setup Functions
To setup the pump use:
- **SetupMotor**: Sets up the motor. Returns 1 if successful, 2 for error, and 3 for warning.
and if also a scale shall be used:
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
