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


class FP3000 {

public:

	// pulblic members
	FP3000(byte MotorNumber, long std_distance, long dir_home, HardwareSerial& serialT, float driver_rsense, uint8_t driver_adress);

	void SetupPump(uint16_t motor_current, uint8_t stall_val, uint16_t mic_steps, uint32_t tcool,
		byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, long max_range, bool use_expander);

	//TESTING - DELETE LATER
	//void PumpFood(byte food_amount);
	void Test(bool moveUP);
	byte Test_Connection();

private:

	// private members
	SpeedyStepper4Purr StepperMotor;
	TMC2209Stepper StepperDriver;

	//byte _PumpNumber;						// NEEDED / DELETE?
	long _std_distance;						// Standard range (steps) the slider should moves when feeding
	long _dir_home;							// Direction to home (1 = CW, -1 = CCW)

};

#endif