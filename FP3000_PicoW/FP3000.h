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
	FP3000(byte MotorNumber, long std_distance, long max_range, long dir_home, float stepper_speed, uint8_t stall_val, uint8_t home_stall_val,
		HardwareSerial &serialT, float driver_rsense, uint8_t driver_address, MCP23017 &mcpRef, bool use_expander, byte mcp_INTA);

	byte SetupMotor(uint16_t motor_current, uint16_t mic_steps, uint32_t tcool, byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_accel);
	byte SetupScale();
	byte HomeMotor();
	byte ErrorHandling(byte error_code);
	byte AutotuneStall(bool quickCheck);
	byte CheckError();
	byte CheckWarning();


	//TESTING - DELETE LATER
	void Test(bool moveUP);
	byte Test_Connection();

private:

	// private members
	SpeedyStepper4Purr StepperMotor;
	TMC2209Stepper StepperDriver;

	byte _MotorNumber;
	MCP23017& mcp;
	long _std_distance;						// Standard range (steps) the slider should moves when feeding
	long _max_range;						// Max range for Motor movement
	long _dir_home;							// Direction to home (1 = CW, -1 = CCW)
	float _stepper_speed;					// Speed of the stepper motor
	uint8_t _stall_val;						// Stall value for normal operation
	uint8_t _home_stall_val;				// Stall value for homing
	bool _use_expander;						// Use MCP23017 for endstop
	byte _mcp_INTA;							// INTA pin for MCP23017

	// Error and Warning Codes
	enum ErrorCode : byte {
		NO_ERROR,
		DRIVER_CONNECTION,	
		STEPPER_UNKOWN,
		STEPPER_JAMMED,
		SCALE_CONNECTION
	}; ErrorCode Error;

	enum WarningCode : byte {
		NO_WARNING,
		STEPPER_FREEDRIVE,
		STEPPER_ENDSTOP,
		STEPPER_STALL
	}; WarningCode Warning;


};

#endif