/*
 * Name:	FoodPump3000
 * Author:	Poing3000
 * Status:	Dev
 *
 * Description:
 * This code is header for the cat food pump, also called Futterpumpe or Foodpump3000.
 * It is ment to be a libary used for Arduino compatible microcontrollers.
 * Further info at: https://github.com/Poing3000/FoodPump3000
*/

#ifndef _FP3000_h
#define _FP3000_h

#include <Arduino.h>
#include <SpeedyStepper4Purr.h>
#include <TMCStepper.h>
#include <MCP23017.h>

class FP3000 {

public:

	// pulblic members
	FP3000(byte MotorNumber, long std_distance, long dir_home, HardwareSerial &serialT, float driver_rsense, uint8_t driver_adress, MCP23017 &mcpRef);

	byte SetupPump(uint16_t motor_current, uint8_t stall_val, uint8_t home_stall_val, uint16_t mic_steps, uint32_t tcool,
		byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, long max_range, bool use_expander, byte mcp_INTA);

	byte SetupScale(uint16_t motor_current, uint8_t stall_val, uint8_t home_stall_val, uint16_t mic_steps, uint32_t tcool,
		byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, long max_range, bool use_expander, byte mcp_INTA);

	//TESTING - DELETE LATER
	//void PumpFood(byte food_amount);
	void Test(bool moveUP);
	byte Test_Connection();
	byte HomeMotor(uint8_t _home_stall_val, bool _use_expander, byte _mcp_INTA);

private:

	byte SetupDriveUnit(uint16_t motor_current, uint8_t stall_val, uint16_t mic_steps, uint32_t tcool, byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin,
		float stepper_speed, float stepper_accel, uint8_t home_stall_val, bool use_expander);

	// private members
	SpeedyStepper4Purr StepperMotor;
	TMC2209Stepper StepperDriver;
	//void MCP_Interrupt();

	//byte _PumpNumber;						// NEEDED / DELETE?
	long _std_distance;						// Standard range (steps) the slider should moves when feeding
	long _dir_home;							// Direction to home (1 = CW, -1 = CCW)
	byte _MotorNumber;
	MCP23017 &mcp;

};

#endif