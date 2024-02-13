/*
 * Name:	FoodPump3000
 * Author:	Poing3000
 * Status:	Dev
 *
 * Description:
 * This code is for the cat food pump, also called Futterpumpe or FoodPump3000.
 * It is ment to be a libary used for Arduino compatible microcontrollers.
 * Note, this code considers the "Pump" only, for the Scale please find the FoodScale3000 libary.
 * Also note that all motors (Driver: <TMCStepper.h> and Stepper: <SpeedyStepper4Purr.h>) setting need to be
 * defined in the main sketch (.ino), this libary only controls the pump.
 * Further info at: https://github.com/Poing3000/FoodPump3000
 *
 * Syntax for function returns:
 * 0 = OK / PROCESSING
 * 1 = OK / FINISHED
 * 2 = ERROR
 * 3 = WARNING
 * >> Warnings and Errors can be checked via function "CheckError() / CheckWarning()".
 * 
 * Updates - DEV:
 * [ ] - [...]
*/

#include "Arduino.h"
#include "FP3000.h"


// SETUP FUNCTIONS

FP3000::FP3000(byte MotorNumber, long std_distance, long max_range, long dir_home, float stepper_speed, uint8_t stall_val, uint8_t home_stall_val,
	HardwareSerial &serial, float driver_rsense, uint8_t driver_address, MCP23017 &mcpRef, bool use_expander, byte mcp_INTA)
	: StepperMotor(MotorNumber), StepperDriver(&serial, driver_rsense, driver_address), mcp(mcpRef) {

	// Remember settings
	_MotorNumber	= MotorNumber;				// Uniuqe Pump number - NEEDED / DELETE?
	_std_distance	= std_distance;				// Standard range (steps) the slider should moves when feeding
	_dir_home		= dir_home;					// Direction to home (1 = CW, -1 = CCW)
	_stepper_speed	= stepper_speed;			// Speed of the stepper motor
	_stall_val		= stall_val;				// Stall value for normal operation
	_home_stall_val = home_stall_val;			// Stall value for homing
	_max_range		= max_range;				// Max range for Motor movement
	_use_expander	= use_expander;				// Use MCP23017 for endstop
	_mcp_INTA		= mcp_INTA;					// INTA pin for MCP23017

	// Initialize Status Codes
	Error			= NO_ERROR;					// Set default error code
	Warning			= NO_WARNING;				// Set default warning code

}

// Setup Motor (BLOCKING)
// Returns 1 if successful, 2 for error and 3 for warning.
byte FP3000::SetupMotor(uint16_t motor_current, uint16_t mic_steps, uint32_t tcool,
	byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_accel) {

	byte setup_result = 0;
	
	// Set Up Driver
	StepperDriver.begin();						// Start driver
	StepperDriver.toff(4);						// Not used, but required to enable the motor
	StepperDriver.blank_time(24);				// Recommended blank time
	StepperDriver.I_scale_analog(false);		// false = external current sense resistors
	StepperDriver.internal_Rsense(false);		// false = deactivates internal resistor (it can't handle necessary currents).
	StepperDriver.mstep_reg_select(true);		// Microstep through UART, not by Pins.
	StepperDriver.rms_current(motor_current);	// Sets the current in milliamps.
	StepperDriver.SGTHRS(_stall_val);			// Set the stall value from 0-255. Higher value will make it stall quicker.
	StepperDriver.microsteps(mic_steps);		// Set microsteps.
	StepperDriver.TCOOLTHRS(tcool);				// Min. speed for stall detection.
	StepperDriver.TPWMTHRS(0);					// Disable StealthChop PWM.
	StepperDriver.semin(0);						// Turn off CoolStep.
	StepperDriver.en_spreadCycle(false);		// Turn off SpreadCycle
	StepperDriver.pdn_disable(true);			// Enable UART

	// Set Up Stepper
	StepperMotor.connectToPins(step_pin, dir_pin, limit_pin, diag_pin);
	StepperMotor.setSpeedInStepsPerSecond(_stepper_speed);
	StepperMotor.setAccelerationInStepsPerSecondPerSecond(stepper_accel);

	// BLOCKING - Wait for homing to be done
	while (setup_result == 0) {
		setup_result = HomeMotor();
	}

	// TODO: Implement Status return
	return setup_result;

}

byte FP3000::SetupScale() {

	// TODO: Implement Scale Setup

	// TODO: Implement Status return
	return 0;

}


// END OF SETUP FUNCTIONS++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// PUBLIC FUNCTIONS

// Check Error
byte FP3000::CheckError() {
	byte _Error = Error;
	Error = NO_ERROR;	// Reset Error
	return _Error;
}

// Check Warning
byte FP3000::CheckWarning() {
	byte _Warning = Warning;
	Warning = NO_WARNING;	// Reset Warning
	return Warning;
}

// TEST FUNCTIONS - DELETE LATER

void FP3000::Test(bool moveUP) {
	if (moveUP) {
		StepperMotor.moveRelativeInSteps(_std_distance);
		//mcp.setPin(0, B, HIGH);
	}
	else {
		StepperMotor.moveRelativeInSteps(-_std_distance);
		//mcp.setPin(0, B, LOW);
	}
}

// END OF TEST FUNCTIONS ++++++++++++++++++++++++DELETE LATER
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Home Motor
byte FP3000::HomeMotor() {

	// =================================================================================================================================
	// This is to home the motor:
	// Homing is done via an endstop signal which is either from a digital pin, an expander pin (MCP23017) or via the driver stall
	// detection. The choice is set via _use_expander and _mcp_INTA. If _use_expander is true, the _mcp_INTA (exp.) pin is used.
	// Else if _use_expander is false, a digiital pin (limit_pin) can be used or the stall detection, depening on _mcp_INTA value (1 for
	// dig. pin). When homing is finished, the function will either return 1 for success, 2 for error or 3 for warning tough it will
	// also try to resolve an error before if possible via ErrorHandling().
	// ================================================================================================================================= 

	// Homing States
	enum HomingState {
		START,
		HOMING,
		DONE,
		ERROR,
	};

	// Flags and results
	static HomingState homingState = START;
	static byte homing_result = 0;
	static bool expander_endstop_signal = false;
	static bool firstRun = true;

	// Homing
	switch (homingState) {
	case START:
		// Set a differnt (more sensitive) stall value for homing if wanted
		StepperDriver.SGTHRS(_home_stall_val);
		homingState = HOMING;
		firstRun = true;
		break;
	case HOMING:
		// Use external endstop (MCP23017) or use digital pin
		if (_use_expander) {
			if (firstRun || digitalRead(_mcp_INTA)) {
				expander_endstop_signal = mcp.getPin(_MotorNumber, A);
				firstRun = expander_endstop_signal;
				mcp.getIntCap(A);
			}
			else {
				expander_endstop_signal = false;
			}

			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, expander_endstop_signal);		// Home Pump
		}

		// Use digital pin as endstop or use stall detection, set via _mcp_INTA
		// Note that the digital pin needs to be set by limit_pin in SetupMotor().
		else {
			bool homeWithPin = false;
			if (_mcp_INTA == 1) {
				homeWithPin = true;
			}

			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, homeWithPin);		// Home Pump
		}

		// Check if homing is done
		if (homing_result != 0) {
			homingState = DONE;
		}
		break;
	case DONE:

		// Reset stall value to normal and homing result
		StepperDriver.SGTHRS(_stall_val);

		// Check if homing was successful
		if (homing_result != 1) {
			// Check Homing Error
			homing_result = ErrorHandling(homing_result);

			// TODO: Implement Error notification
			if (homing_result == 0 || homing_result == 3) {
				// Implement Error notification
			}
			

		}

		homingState = START; // Reset state for next time
		return homing_result;
	}
	return 0;
}

// Test Connection to Stepper Driver
byte FP3000::Test_Connection() {
	byte cTest = 99;
	cTest = StepperDriver.test_connection(); // 0 = OK, else Error
	return cTest;
}

// Autotune Stall
byte FP3000::AutotuneStall(bool quickCheck) {

	// =================================================================================================================================
	// This is to automatically find and set the stall value for the motor:
	// Set _stall_val to a reasonable value of 200 (100 for quick check).
	// Move once away from endstop by std_distance * (factor = 1%).
	// Check if stall occurs; if yes, decrease stall value by 1, return to start and repeat; though if _stall_val < 10 return 0 (error).
	// If no stall occurs, return to start and increase factor by 1% and repeat until factor is 100%.
	// Finally check if stall occurs when moving away (std_distance) and back to endstop.
	// If no stall occurs finish and return _stall_val; though if stall is true start again from beginning.
	// =================================================================================================================================

	float factor;
	float stepFactor;
	byte checkStep;

	// If quick check is true, checking is way faster.
	if (quickCheck) {
		_stall_val = 100;
		factor = 0.1;
		stepFactor = 0.1;
		checkStep = 10;
	}
	else {
		_stall_val = 200;
		factor = 0.01;
		stepFactor = 0.01;
		checkStep = 1;
	}
	bool stallFlag = false;
	bool checkFlag = false;

	while (factor <= 1) {
		// Set stall value
		StepperDriver.SGTHRS(_stall_val);

		// Move away from endstop
		stallFlag = StepperMotor.moveRelativeInSteps(_std_distance * factor * (-_dir_home));

		if (stallFlag) {
			// Reduce stall value
			_stall_val -= checkStep;
			// Return to start
			StepperMotor.moveRelativeInSteps(_std_distance * factor * _dir_home);
			// Repeat
			factor = stepFactor;
		}
		else {
			// Return to start
			StepperMotor.moveRelativeInSteps(_std_distance * factor * _dir_home);
			// No stall occured, increase factor
			factor += stepFactor;
			if (factor >= 0.9 && checkFlag == false) {
				factor = stepFactor;
				checkFlag = true;
			}

		}
	}
	_home_stall_val = _stall_val-2;		// Set stall value for homing (reduce by 2 just to make sure)
	_stall_val -= 10;					// Reduce stall value by a safety margin of 10
	StepperDriver.SGTHRS(_stall_val);	// Set final stall value
	return _stall_val;
}


/*
// Approximate Filling
void FP3000::ApproximateFill(byte appr_amount) {

}

*/

/*
// Accurate Filling
void FP3000::AccurateFill(byte accu_amount) {

*/


/*
// Pump Food
void FP3000::PumpFood(byte fill_amount) {

	// TODO: Function for Approximate filling

	// TODO: Funtion for Accurate filling
}
*/

// END OF PUBLIC FUNCTIONS+++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// PRIVATE FUNCTIONS

// Error Handling
byte FP3000::ErrorHandling(byte error_code) {

	// =================================================================================================================================
	// This is to handle errors:
	// TODO: finalise this function... [...]
	// 
	// The function receives an error code and tries to resolve it. If the error is resolved, a warning (3) is triggered and the
	// function returns 3. If the error could not be resolved, a major error (2) is triggered and the function returns 2. Errors and
	// warnings are saved at Errors / Warrnings and can be checked via function "CheckError() / CheckWarning()".
	// =================================================================================================================================

	byte result = 0;
	bool stallFlag = false;

	// Check type of Error
	if (error_code == 2) {

		// Endstop sensor stuck high, try to home with stall detection.
		byte homing_result = 0;
		while (homing_result == 0) {
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, false);
		}
		if (homing_result != 1) {
			// Error could not be resolved
			result = 2; // Error (2), major failure!
			Error = STEPPER_UNKOWN;
		}
		else {
			// Homed with stall detection
			result = 3;	// Homing was successful, yet a warning (3) should be triggered.
			Warning = STEPPER_ENDSTOP;
		}
			
	}
	else {
		// Further investigation needed
		result = StepperMotor.ErrorHandling(_dir_home, _max_range, _std_distance);
		
		// Error resolved though a warning (3) should be triggered
		if (result == 0 || result == 3) {
			if (result == 0) {
				Warning = STEPPER_FREEDRIVE;
			}
			else {
				Warning = STEPPER_ENDSTOP;
			}
			result = 3;	// Homing was successful, yet a warning (3) should be triggered.
		}
		else {
			// Error could not be resolved
			if (result == 2) {
				Error = STEPPER_UNKOWN;
			}
			else {
				Error = STEPPER_JAMMED;
			}
			result = 2; // Error (2), major failure!
		}
	}
	return result;
}


// END OF PRIVATE FUNCTIONS++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++