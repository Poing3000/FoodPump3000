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
 *
 * Updates - DEV:
 * [ ] - [...]
*/

#include "Arduino.h"
#include "FP3000.h"



// SETUP FUNCTIONS

FP3000::FP3000(byte MotorNumber, long std_distance, long dir_home, HardwareSerial &serial, float driver_rsense, uint8_t driver_adress, MCP23017 &mcpRef)
	: StepperMotor(MotorNumber), StepperDriver(&serial, driver_rsense, driver_adress), mcp(mcpRef) {

	// Remember settings
	_MotorNumber = MotorNumber;					// Uniuqe Pump number - NEEDED / DELETE?
	_std_distance = std_distance;				// Standard range (steps) the slider should moves when feeding
	_dir_home = dir_home;						// Direction to home (1 = CW, -1 = CCW)


}
//TODO: Add error handling / Return error - change from void to byte and return error code
byte FP3000::SetupPump(uint16_t motor_current, uint8_t stall_val, uint8_t home_stall_val, uint16_t mic_steps, uint32_t tcool,
	byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, long max_range, bool use_expander, byte mcp_INTA) {
	
	SetupDriveUnit(motor_current, stall_val, mic_steps, tcool, step_pin, dir_pin, limit_pin, diag_pin, stepper_speed, stepper_accel, home_stall_val, use_expander);



	return 0;

}

byte FP3000::SetupScale(uint16_t motor_current, uint8_t stall_val, uint8_t home_stall_val, uint16_t mic_steps, uint32_t tcool,
	byte step_pin, byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, long max_range, bool use_expander, byte mcp_INTA) {


	SetupDriveUnit(motor_current, stall_val, mic_steps, tcool, step_pin, dir_pin, limit_pin, diag_pin, stepper_speed, stepper_accel, home_stall_val, use_expander);



	return 0;

}



// END OF SETUP FUNCTIONS++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// PUBLIC FUNCTIONS

// TEST FUNCTIONS - DELETE LATER

void FP3000::Test(bool moveUP) {
	if (moveUP) {
		StepperMotor.moveRelativeInSteps(4600);
		//mcp.setPin(0, B, HIGH);
	}
	else {
		StepperMotor.moveRelativeInSteps(-4600);
		//mcp.setPin(0, B, LOW);
	}
}

byte FP3000::Test_Connection() {
	byte cTest = 99;
	cTest = StepperDriver.test_connection(); // 0 = OK, else Error
	return cTest;
}


// END OF TEST FUNCTIONS ++++++++++++++++++++++++DELETE LATER
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// Home Pump
byte FP3000::HomeMotor(uint8_t _home_stall_val, bool _use_expander, byte _mcp_INTA) {
	
	// Homing States
	enum HomingState {
		START,
		HOMING,
		DONE
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

			homing_result = StepperMotor.moveToHome(_dir_home, 10000, expander_endstop_signal);		// Home Pump
		}

		// Use digital pin as endstop or use stall detection, set via _mcp_INTA
		else {
			bool homeWithStall = false;
			if (_mcp_INTA == 1) {
				homeWithStall = true;
			}

			homing_result = StepperMotor.moveToHome(_dir_home, 10000, homeWithStall);		// Home Pump
		}

		// Check if homing is done
		if (homing_result != 0) {
			homingState = DONE;
		}
		break;
	case DONE:
		// Reset stall value to normal and homing result
		StepperDriver.SGTHRS(_home_stall_val);
		if (homing_result != 1) {
			// Error
			// TODO: Implement emergency mode here?
		}

		homingState = START; // Reset state for next time
		return homing_result;
	}
	return 0;
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

	// Setup Stepper Motor, Driver and Endstop Combination

	byte FP3000::SetupDriveUnit(uint16_t motor_current, uint8_t stall_val, uint16_t mic_steps, uint32_t tcool, byte step_pin,
		byte dir_pin, byte limit_pin, byte diag_pin, float stepper_speed, float stepper_accel, uint8_t home_stall_val, bool use_expander) {

				
		// Set Up Driver
		StepperDriver.begin();						// Start driver
		StepperDriver.toff(4);						// Not used, but required to enable the motor
		StepperDriver.blank_time(24);				// Recommended blank time
		StepperDriver.I_scale_analog(false);		// false = external current sense resistors
		StepperDriver.internal_Rsense(false);		// false = deactivates internal resistor (it can't handle necessary currents).
		StepperDriver.mstep_reg_select(true);		// Microstep through UART, not by Pins.
		StepperDriver.rms_current(motor_current);	// Sets the current in milliamps.
		StepperDriver.SGTHRS(stall_val);			// Set the stall value from 0-255. Higher value will make it stall quicker.
		StepperDriver.microsteps(mic_steps);		// Set microsteps.
		StepperDriver.TCOOLTHRS(tcool);				// Min. speed for stall detection.
		StepperDriver.TPWMTHRS(0);					// Disable StealthChop PWM.
		StepperDriver.semin(0);						// Turn off CoolStep.
		StepperDriver.en_spreadCycle(false);		// Turn off SpreadCycle
		StepperDriver.pdn_disable(true);			// Enable UART

		// Set Up Stepper
		StepperMotor.connectToPins(step_pin, dir_pin, limit_pin, diag_pin);
		StepperMotor.setSpeedInStepsPerSecond(stepper_speed);
		StepperMotor.setAccelerationInStepsPerSecondPerSecond(stepper_accel);

		// BLOCKING - Wait for homing to be done
		while (HomeMotor(home_stall_val, use_expander, 15) == 0);
		// TODO: Implement Error handling for homing here or in HomePump()?


		return 0;
	}

/*
void FP3000::MCP_Interrupt() {


}
*/

// END OF PRIVATE FUNCTIONS++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++