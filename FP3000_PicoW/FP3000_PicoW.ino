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
 *		[ ] - Implement own homing function (error management/stall detection)
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
#define	STALL_VALUE		 30		// Stall threshold [0..255] (lower = more sensitive) - 30 is quite low, yet safe for stable error management.

// Stepper Driver (TMC2209)
#define DIR_1	11				// Direction pin
#define STEP_1	12				// Step pin
#define	DIAG_1	13				// DIAG pin for stall detection
#define	DRIVER_ADDRESS_1 0b00	// Drivers address
//---------------------------------*

// Stepper Motor (NEMA 17)
#define LIMIT_1	4				// Limit switch pin
#define SPEED	10000			// Speed (steps/s)
#define ACCEL	100000			// Acceleration (steps/s^2)

TMC2209Stepper driver_1(&SERIAL_PORT_2, R_SENSE, DRIVER_ADDRESS_1);	// Create Driver
//SpeedyStepper4Purr stepper_0 (0);									// Create Stepper motor	
SpeedyStepper4Purr stepper_1 (1);
//---------------------------------*

//**********************************
//DELETE
/*
void StallIndication();

void StallIndication()
{
	flag_stalled = true;
}

  // Setup Driver Interrupt for Motor Stall Detection
  attachInterrupt(digitalPinToInterrupt(homeDiagPin), StallIndication, RISING);

*/
//**********************************

//Variables
//[...]

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
/*
	// Homing (find endstop) - Note, blocking function!
	void GoHome() {
		DEBUG_DEBUG("GoHome()");

		// Pump 1 - Homing
		DEBUG_VERBOSE("Homing Pump 1.");
		if (stepper_1.moveToHomeInSteps(1, SPEED, MAX_RANGE) != true) {

			// E001 - Pump 1: Homing Error, Endstop not found.
			DEBUG_ERROR("E011");
			PumpErrorHandling(0, 1);
		}
		stepper_1.setCurrentPositionInSteps(0);
		DEBUG_VERBOSE("Homing finished.");
	}

	void PumpErrorHandling(byte pump, byte error) {
	DEBUG_DEBUG("Pump Error Handling");
		switch (error) {
			// 1 - SOLVE HOMING ERROR
			case 1: 
				// ===========================================================================================
				// If stall is true, either there is an endstop malfunction or the slider is stuck.
				// So stall flag is reset to then check if stall apears again while trying to move the slider.
				// If stall is again true, the slider is stuck and we need to try to free it up.
				// If stall is false/we got to get the slider unstuck, we can try to home again.
				// If this fails again, the endstop may not working, so we can try to home with stall.
				// -------------------------------------------------------------------------------------------
				DEBUG_VERBOSE("Checking slider error.");
				if (flag_stalled_1 == true) {						
					flag_stalled_1 = false;							
					stepper_1.moveRelativeInSteps(1000);			
					if (flag_stalled_1 == true) {					

						// Slider stuck, try to free up.
						DEBUG_VERBOSE("Slider stuck, trying to free up..");
						int i = 1, y = 1;
						stepper_1.setSpeedInStepsPerSecond(1000);	// Reduce to generate more torque.

						// Vibrate slider to free up.
						// ---------------------------------------------------
						while (i <= 1000) {
							stepper_1.moveRelativeInSteps(i);
							if (i > 0) {
								i = -i;
							}
							else {

								if (y < 200) {
									i = -i + 1;
									y++;
								}
								else {
									i = -i + y;
								}
								stepper_1.setSpeedInStepsPerSecond(1000 * y);
							}
						}
						// ---------------------------------------------------
						stepper_1.setSpeedInStepsPerSecond(SPEED);	// Reset speed.

						// Check if slider is free now.
						flag_stalled_1 = false;
						stepper_1.moveRelativeInSteps(1000);
						if (flag_stalled_1 == true) {
							// Slider still stuck, EMGY mode.
							DEBUG_VERBOSE("Slider still stuck, EMGY mode.");
							// [TODO] >>BREAK<<
						}

					}
					// Slider is free, try to home again.
					DEBUG_VERBOSE("Slider is free, trying to home again.");

					if (stepper_1.moveToHomeInSteps(1, SPEED, MAX_RANGE, LIMIT_1) != true) {

						DEBUG_VERBOSE("Possible endstop failure, try to home with stall.");
						// TODO Homing with stall.
					}
					DEBUG_VERBOSE("2nd Homing attempt successful.");

				}
				else {
					// If stall is false, neither the endstop works nor the stall detection.
					DEBUG_VERBOSE("Drivetrain/Motor malfunction detected.");
					// Drivetrain/Motor malfunction >> EMGY mode [TODO] >>BREAK<<
				}
			break;
		}
	}
*/
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

		// DELETE - DELAY FOR SERIAL MONITOR
		delay(1000);

		// Move Home
		while (stepper_1.moveToHome(-1, SPEED, MAX_RANGE, true) != 1);
		Serial.print("HOMING DONE");

	//---------------------------------*
}
// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// MAIN PROGRAM (LOOP):
void loop() {
	// Move to Position
	/*
	delay(2000);
	DEBUG_VERBOSE("+ MOVE UP");
	stepper_1.moveToPositionInSteps(6000);
	DEBUG_VERBOSE("~ STOPPED");
	delay(2000);

	DEBUG_VERBOSE("- MOVE DOWN");
	stepper_1.moveToPositionInSteps(0);
	DEBUG_VERBOSE("~STOPPED");
	*/
}
/*
void loop1() {
	if (stalled == true) {
		DEBUG_VERBOSE("********************* STALLED *********************");
		stalled = false;
	}
	// Only print if TSTEP has changed by +/- 10:
	static int previousTSTEP = 0; // Variable to store previous TSTEP value

	int currentTSTEP = driver_0.TSTEP(); // Get the current TSTEP value

	if (abs(currentTSTEP - previousTSTEP) >= 10) {
		DEBUG_VERBOSE.print("TSTEP: ");
		DEBUG_VERBOSE.println(currentTSTEP);
		previousTSTEP = currentTSTEP; // Update the previous TSTEP value
	}
	delay(1);
}
*/
// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++