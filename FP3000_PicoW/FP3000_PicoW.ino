/*
 * Name:	FoodPump3000
 * Author:	Poing3000
 * Status:	Dev
 *
 * Description:
 * This code is for the cat food pump, called Futterpumpe (Foodpump3000).
 * It is built for a Raspberry Pico W and a TMC2209 stepper driver.
 * Further info at: https://github.com/Poing3000/FoodPump3000
 *
 * Updates - DEV:
 * [ ] - Prime (find endstop)
 *		// Note, can be changed by setting "LIMIT" or "STALL" in Homing (Line 134)
 *		[x] - Find endstop with IR Sensor
 *		[x] - Find endstop with Stall
 *		[x] - Use IR as default homing sensor (later use stall as backup and for error management).
 *		[x] - Implement own homing function (error management/stall detection) >> SEE SpeedyStepper4Purr.h
 * [ ] - Approx. filling
 * [ ] - Accurate filling
 * [ ] - TMC2209 Prime with IR Sensor check
 * [ ] - Calibrate with IR Sensor
 * [ ] - Error Management:
 *    [ ] - Detect Calibration Error (dif. IR vs Stall)
 *    [ ] - Handle unexpected Stall
 *    [ ] - EMGY Mode
 * [ ] - Comunicate
 * [ ] - Handle/implement multicore. >> Comunication on Core 0, Motor on Core 1.
 * [ ] - Implement multible pumps control.
 * 
 * [ ] - *Future wish: Use RP2040 PIO for Stepper control. 
 * 
 * 
 * >> Notes: Aktuell bei der Implementierung der ErrorHandling Funktion. Diese funktioniert aber noch nicht. Problem mit Switch/Case?
 * 
 * DEBUG CODES:
 * ----------------------------------------------------------
 * Status Codes (S):
 * [...]
 * 
 * Error Codes	(E):
 * 1 - Homing Error, Endstop not found.
 * 
 * Warning Codes (W):
 * 1 - Stall detected.
 * 2 - Homing successful at 2nd try.
 * 
 * -- Error/Warning sytnax ---
 * E		0			0
 * ^		^			^
 * Type		Device		Actual Problem
 * ===================
 */


 // Parts for individual configuration are highlited below.
 // ----------------------------------------------------------
 // 
 // CONFIGURATION:
 // ----------------------------------------------------------
 // Libraries
	// Stepper
	#include <TMCStepper.h>
	#include <SpeedyStepper4Purr.h>

	// Debugging
	#include <Arduino_DebugUtils.h>
// ---------------------------------*

// DELETE LATER!!! INDICATOR LED
#define LED  7

// Global (NEEDS TO BE ADAPTED/CHANGED FOR LIBRARY!!!)
#define ENABLE          15      // Enable Pin
#define	SERIAL_PORT_2	Serial2	// HardwareSerial port (TX: 8, RX: 9)
#define R_SENSE         0.11f   // Sense resistor value of the driver fur current cal.
#define CURRENT         200    // Max current (mA) supplied to the motor
#define MIRCO_STEPS     32      // Set microsteps (32 is a good compromise between CPU load and noise)
#define TCOOLS          400		// max 20 bits
#define MAX_RANGE       6600    // Max physical slider range is app. 6230 steps. Thus, 6600 is a safe value for error management.
#define	STALL_VALUE		50		// Stall threshold [0..255] (lower = more sensitive) - 30 is quite low, yet safe for stable error management (50 for accuracy)
#define START_DIST_S	200		// Start distance to go from endstop (only used for stall homing)

// Stepper Driver (TMC2209)
#define DIR_1	11				// Direction pin
#define STEP_1	12				// Step pin
#define	DIAG_1	13				// DIAG pin for stall detection
#define	DRIVER_ADDRESS_1 0b00	// Drivers address
//---------------------------------*

// Stepper Motor (NEMA 17)
#define LIMIT_1	4				// Limit switch pin
#define SPEED	10000			// Speed (steps/s) (10000 is good)
#define ACCEL	100000			// Acceleration (steps/s^2) (100000	is good)

TMC2209Stepper driver_1(&SERIAL_PORT_2, R_SENSE, DRIVER_ADDRESS_1);	// Create Driver
//SpeedyStepper4Purr stepper_0 (0);									// Create Stepper motor	
SpeedyStepper4Purr stepper_1 (1);
//---------------------------------*

//Variables
const int Feed_Range = 4600;	// Typical feeding distance

// Flags
//[...]
//---------------------------------*

// Debugging
#define DEBUG_LEVEL DBG_VERBOSE	//Set desired debug level
	/* ---------------------------------
	DBG_NONE - no debug output is shown
	DBG_ERROR - critical errors
	DBG_WARNING - non - critical errors
	DBG_INFO - information
	DBG_DEBUG - more information
	DBG_VERBOSE - most information
	Further info at: https://github.com/arduino-libraries/Arduino_DebugUtils
	------------------------------------
	*/


// END OF CONFIG+++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// FUNCTIONS:
//[...]

// END OF FUNCTIONS++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SETUP:
void setup() {

	//DELETE DELAY FOR SERIAL
	delay(2000);

	// Debugging
	Debug.setDebugLevel(DEBUG_LEVEL);
		// DELETE LATER! DEBUG/TESTING
		pinMode(LED, OUTPUT);				// Indicator LED

	//Stepper Setup
	//---------------------------------
		// Pin Setup
		pinMode(ENABLE, OUTPUT);

		// Driver Setup
		SERIAL_PORT_2.begin(115200);

		driver_1.begin();					  // Start driver
		driver_1.toff(4);					  // Not used, but required to enable the motor
		driver_1.blank_time(24);              // Recommended blank time
		driver_1.I_scale_analog(false);       // false = extrenal current sense resistors
		driver_1.internal_Rsense(false);      // false = deactivates internal resistor (it can't handle necessary currents).
		driver_1.mstep_reg_select(true);      // Microstep through URART, not by Pins.
		driver_1.rms_current(CURRENT);        // Sets the current in milliamps.
		driver_1.SGTHRS(STALL_VALUE);         // Set the stall value from 0-255. Higher value will make it stall quicker.
		driver_1.microsteps(MIRCO_STEPS);     // Set microsteps.
		driver_1.TCOOLTHRS(TCOOLS);           // Min. speed for stall detection.
		driver_1.TPWMTHRS(0);                 // Disable StealthChop PWM.
		driver_1.semin(0);                    // Turn off CoolStep.
		driver_1.en_spreadCycle(false);       // Turn off SpreadCycle
		driver_1.pdn_disable(true);           // Enable UART

		// Motor Setup
		//stepper_0.connectToPins(0, 0, 0, 0);
		stepper_1.connectToPins(STEP_1, DIR_1, LIMIT_1, DIAG_1);
		stepper_1.setSpeedInStepsPerSecond(SPEED);
		stepper_1.setAccelerationInStepsPerSecondPerSecond(ACCEL);

		//Start Driver and Motor
		digitalWrite(ENABLE, LOW);			// Enable motor

		/*
		// DELETE - TESTING ONLY: Move Home
		bool home_test = false;
		byte home_result = 0;
		byte error_result = 0;
		while (1) {
			home_result = stepper_1.moveToHome(-1, SPEED, MAX_RANGE, START_DIST_S, true);

			if (home_result > 0) {
				if (home_result >= 2) {
					DEBUG_VERBOSE("Error occured");
					if ((stepper_1.ErrorHandling(1, home_result, SPEED, MAX_RANGE) == 0) {
						//HOME AGAIN
					}
					else {
						//EMGY
					}
				}

				DEBUG_VERBOSE("error_result: %d", error_result);
				DEBUG_VERBOSE("Home again in 2s..");
				delay(2000);
				while (1) {
					home_result = stepper_1.moveToHome(-1, SPEED, MAX_RANGE, START_DIST_S, true);
					if (home_result > 0) {
					break;
					}
				}
			break;
			}
		} DEBUG_VERBOSE("home_result: %d", home_result);
		*/

	//---------------------------------*
}
// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// MAIN PROGRAM (LOOP):
void loop() {





}
// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++