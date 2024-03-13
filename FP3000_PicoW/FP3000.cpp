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
 * 0 = OK / BUSY
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

	// Set Up Driver
	// (Check TMC2209Stepper.h for more details on the functions and settings)
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

	// Wait for homing to be done (BLOCKING)
	byte motorResult = BUSY;
	while (motorResult == BUSY) {
		motorResult = HomeMotor();
	}

	// Test Connection to Stepper Driver
	byte driverResult = Test_Connection();

	// Return Status
	if (motorResult == OK && driverResult == OK) {
		return OK;
	}
	else if (motorResult == ERROR || driverResult == ERROR) {
		return ERROR; 
	}
	else {
		return WARNING;
	}
}

byte FP3000::SetupScale(uint8_t nvmAddress, uint8_t dataPin, uint8_t clockPin) {
	_nvmAddress = nvmAddress;
	float scaleCalVal;
	
	// IMPLEMENT:
	if (!LittleFS.begin()) {
		Error = FILE_SYSTEM;
		return ERROR;
	}

	// Read scale calibration from file
	char filename[20];
	sprintf(filename, "/scale_%d.bin", _nvmAddress);

	// Read from the file
	File file = LittleFS.open(filename, "r");
	if (file) {
		file.read((uint8_t*)&scaleCalVal, sizeof(scaleCalVal));
		file.close();
	
	/* // Uncomment if you want to see the calibration value
	Serial.print("Scale Calibration Value: ");
	Serial.println(scaleCalVal);
	*/

	}
	else {
		// Error opening file, calibration needed to create data
		Warning = SCALE_CALFILE;
		return WARNING;
	}
	LittleFS.end();
	Scale.begin(dataPin, clockPin, true);
	Scale.set_scale(scaleCalVal);
	Scale.tare(20);
	return OK;
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
	return _Warning;
}

// Test Function - Move Motor Up/Down
void FP3000::MotorTest(bool moveUP) {
	if (moveUP) {
		StepperMotor.moveRelativeInSteps(_std_distance);
	}
	else {
		StepperMotor.moveRelativeInSteps(-_std_distance);
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
	// also try to resolve an error before if possible via ManageError().
	// ================================================================================================================================= 

	// Homing States
	enum HomingState {
		START,
		HOMING,
		DONE,
		ERROR
	};

	// Flags and results
	static HomingState homingState = START;
	static byte homing_result = BUSY;
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
			// Home Pump with expander
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, expander_endstop_signal);
		}

		// Use digital pin as endstop or use stall detection, set via _mcp_INTA
		// Note that the digital pin needs to be set by limit_pin in SetupMotor().
		else {
			bool homeWithPin = false;
			if (_mcp_INTA == 1) {
				homeWithPin = true;
			}
			// Home Pump with digital pin / stall detection
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, homeWithPin);
		}

		// Check if homing is done
		if (homing_result != BUSY) {
			homingState = DONE;
		}
		break;
	case DONE:

		// Reset stall value to normal and homing result
		StepperDriver.SGTHRS(_stall_val);

		// Check if there was an issue during homing
		if (homing_result != OK) {
			// Check Error
			homing_result = ManageError(homing_result);
		}

		homingState = START;	// Reset state for next time
		return homing_result;	// Homing finished, return result.
	}
	return BUSY;	// Homing in progress
}

// Test Connection to Stepper Driver
byte FP3000::Test_Connection() {
	byte cTest;

	cTest = StepperDriver.test_connection();
	//----------------------------------------------
	// Conection Test returns 0 if OK, else an error 
	// code (1/2) (see TMC2209Stepper.h for details)
	//----------------------------------------------
	if(cTest != 0) {
		Error = DRIVER_CONNECTION;
		cTest = ERROR;	// Error, Driver Connection failure
	}else{
		cTest = OK;		// Driver Connection OK
	}
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

	float factor;		// Percental factor for moving distance
	float stepFactor;	// Factor for increasing factor
	byte checkStep;		// Step by which stall sensivity is decreased

	// If quick check is true, checking is way faster.
	if (quickCheck) {
		// QUICK CHECK SETTINGS
		_stall_val = 100;
		factor = 0.1;
		stepFactor = 0.1;
		checkStep = 10;
	}
	else {
		// SLOW CHECK SETTINGS
		_stall_val = 200;
		factor = 0.01;
		stepFactor = 0.01;
		checkStep = 1;
	}
	bool stallFlag = false;		// Indicates a stall
	bool checkFlag = false;		// Helper flag to finish checks

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

// Measure Food
float FP3000::Measure() {
	float Weight = Scale.get_units(7);	
	return Weight;
}

// Calibrate Scale
float FP3000::CalibrateScale() {

	Serial.println("\n\nCALIBRATION\n===========");
	Serial.println("remove all weight from the loadcell");
	//  flush Serial input
	while (Serial.available()) Serial.read();

	Serial.println("and press enter\n");
	while (Serial.available() == 0);

	Serial.println("Determine zero weight offset");
	Scale.tare(20);  // average 20 measurements.
	uint32_t offset = Scale.get_offset();

	Serial.print("OFFSET: ");
	Serial.println(offset);
	Serial.println();


	Serial.println("place a weight on the loadcell");
	//  flush Serial input
	while (Serial.available()) Serial.read();

	Serial.println("enter the weight in (whole) grams and press enter");
	uint32_t weight = 0;
	while (Serial.peek() != '\n')
	{
		if (Serial.available())
		{
			char ch = Serial.read();
			if (isdigit(ch))
			{
				weight *= 10;
				weight = weight + (ch - '0');
			}
		}
	}
	Serial.print("WEIGHT: ");
	Serial.println(weight);
	Scale.calibrate_scale(weight, 20);
	float scale = Scale.get_scale();

	Serial.print("SCALE:  ");
	Serial.println(scale, 6);

	Serial.print("\nuse scale.set_offset(");
	Serial.print(offset);
	Serial.print("); and scale.set_scale(");
	Serial.print(scale, 6);
	Serial.print(");\n");
	Serial.println("in the setup of your project");

	Serial.println("\n\n");

	// Save calibration to file
	if (!LittleFS.begin()) {
		Serial.println("Failed to mount file system");
	}
	else {
		Serial.println("File system mounted");
	}

	// Write scale calibration to file
	char filename[20];
	sprintf(filename, "/scale_%d.bin", _nvmAddress);
	File file = LittleFS.open(filename, "w");

	// Write to the file
	if (file) {
		file.write((uint8_t*)&scale, sizeof(scale));		
		file.close();
	}
	else {
		Serial.println("There was an error opening the file for writing");
	}
	LittleFS.end();


	return scale;
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
byte FP3000::ManageError(byte error_code) {

	// =================================================================================================================================
	// This is to handle errors:
	// TODO: finalise this function... [...]
	// 
	// The function receives an error code and tries to resolve it. If the error is resolved, a warning (3) is triggered and the
	// function returns 3. If the error could not be resolved, a major error (2) is triggered and the function returns 2. Errors and
	// warnings are saved at Errors / Warrnings and can be checked via function "CheckError() / CheckWarning()".
	// =================================================================================================================================

	byte result = BUSY;
	bool stallFlag = false;

	// Check type of Error
	// ---------------------------------------------------------
	// In a failure case, moveToHome() returns (at HomeMotor()):
	// 2 - Enstop always triggered.
    // 3 - Enstop never triggered.
	// Hence error_code is checked for 2 (else 3).
	// ---------------------------------------------------------
	if (error_code == 2) {
		// Endstop sensor STUCK HIGH, try to home with stall detection.
		byte homing_result = BUSY;
		while (homing_result == BUSY) {
			homing_result = StepperMotor.moveToHome(_dir_home, _max_range, false);
		}
		// Check if homing was successful
		if (homing_result == OK) {
			// Successfully homed with stall detection, yet a warning (3) should be triggered.
			Warning = STEPPER_ENDSTOP;
			result = WARNING;	// Warning (3) 
		}
		else {
			// Error could not be resolved
			Error = STEPPER_UNKOWN;
			result = ERROR; // Error (2), major failure!
		}
			
	}
	else {
		// Endstop sensor STUCK LOW, further investigation needed
		result = StepperMotor.ErrorHandling(_dir_home, _max_range, _std_distance);
		// -------------------------------------------------------
		// ErrorHandling() returns:
		// 0 - was stuck but solved/freed
		// 1 - unknown drivetrain malfunction
		// 2 - slider is jammed
		// 3 - endstop malfunction, solved with stall detection
		// So 0 & 3 equal a warning (3), 1 & 2 equal an error (2).
		// -------------------------------------------------------
		switch (result) {
		case 0:
			Warning = STEPPER_FREEDRIVE;
			result = WARNING;
			break;
		case 1:
			Error = STEPPER_UNKOWN;
			result = ERROR;
			break;
		case 2:
			Error = STEPPER_JAMMED;
			result = ERROR;
			break;
		case 3:
			Warning = STEPPER_ENDSTOP;
			result = WARNING;
			break;
		default:
			// Handle unexpected return
			Error = STEPPER_UNKOWN;
			result = ERROR;
		}
	}
	return result;
}



// END OF PRIVATE FUNCTIONS++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++