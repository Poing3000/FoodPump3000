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
 *		[ ] - Decide if homing with IR or Stall >> Probably best to use IR as default homing sensor. Then use stall as backup and for error management.
 *		[ ] - Handle/implement multicore. >> Comunication on Core 0, Motor on Core 1.
 *		[ ] - Implement own homing function
 * [ ] - Approx. filling
 * [ ] - Accurate filling
 * [ ] - TMC2209 Prime with IR Sensor check
 * [ ] - Calibrate with IR Sensor
 * [ ] - Error Management:
 *    [ ] - Detect Calibration Error (dif. IR vs Stall)
 *    [ ] - Handle unexpected Stall
 *    [ ] - EMGY Mode
 * [ ] - Comunicate
 *
 */


 // Parts for individual configuration are highlited below.
 // ----------------------------------------------------------
 // 
 // CONFIGURATION:
 // ----------------------------------------------------------
 // Libraries
#include <TMCStepper.h>
#include <SpeedyStepper.h>
// ---------------------------------*

// DELETE LATER!!! INDICATOR LED
#define LED  7

// Global (NEEDS TO BE ADAPTED/CHANGED FOR LIBRARY!!!)
#define ENABLE          15      // Enable Pin
#define	SERIAL_PORT_2	Serial2	// HardwareSerial port (TX: 8, RX: 9)
#define R_SENSE         0.11f   // Sense resistor value of the driver fur current cal.
#define CURRENT         1000    // Max current (mA) supplied to the motor
#define MIRCO_STEPS     32      // Set microsteps
#define TCOOLS          400		// max 20 bits
#define MAX_RANGE       4000    // Max travel distance (steps)
// ---------------------------------*

// Stepper Driver (TMC2209)
#define DIR		11				// Direction pin
#define STEP	12				// Step pin
#define	STALL	13				// DIAG pin
#define	STALL_VALUE		80		// Stall threshold [0..255] (lower = more sensitive)
#define	DRIVER_ADDRESS	0b00	// Drivers address
//---------------------------------*

// Stepper Motor (NEMA 17)
#define LIMIT	4				// Limit switch pin
#define SPEED	10000			// Speed (steps/s)
#define ACCEL	80000			// Acceleration (steps/s^2)

TMC2209Stepper	driver(&SERIAL_PORT_2, R_SENSE, DRIVER_ADDRESS);	// Setup driver
SpeedyStepper	stepper;											// Create stepper motor
//---------------------------------*
 
//Variables
bool	stalled = false;			// Stall indication
//---------------------------------*

// END OF CONFIG+++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// FUNCTIONS:
	//Stall indication
	void Stall_Int() {
		stalled = true;
		Serial.println("***************************************************");
		Serial.println("********************! STALLED !********************");
		Serial.println("***************************************************");
		Serial.print("TSTEP: ");
		Serial.println(driver.TSTEP());
		Serial.print("SG: ");
		Serial.println(driver.SG_RESULT());
		}

// END OF FUNCTIONS++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SETUP:
void setup() {
	// DELETE LATER!!! INDICATOR LED
	pinMode(LED, OUTPUT);

	//Stepper Setup
	//---------------------------------
		// Pin Setup
		pinMode(LIMIT, INPUT);
		pinMode(ENABLE, OUTPUT);

		// Driver Setup
		SERIAL_PORT_2.begin(115200);

		driver.begin();						// Start driver
		driver.toff(4);						// Not used, but required to enable the motor
		driver.blank_time(24);              // Recommended blank time
		driver.I_scale_analog(false);       // false = extrenal current sense resistors
		driver.internal_Rsense(false);      // false = deactivates internal resistor (it can't handle necessary currents).
		driver.mstep_reg_select(true);      // Microstep through URART, not by Pins.
		driver.rms_current(CURRENT);        // Sets the current in milliamps.
		driver.SGTHRS(STALL_VALUE);         // Set the stall value from 0-255. Higher value will make it stall quicker.
		driver.microsteps(MIRCO_STEPS);     // Set microsteps.
		driver.TCOOLTHRS(TCOOLS);           // Min. speed for stall detection.
		driver.TPWMTHRS(0);                 // Disable StealthChop PWM.
		driver.semin(0);                    // Turn off CoolStep.
		driver.en_spreadCycle(false);       // Turn off SpreadCycle
		driver.pdn_disable(true);           // Enable UART

		// Setup Driver Interrupt for Motor Stall Detection
		pinMode(STALL, INPUT);
		//attachInterrupt(digitalPinToInterrupt(STALL), Stall_Int, RISING);

		// Motor Setup
		stepper.connectToPins(STEP, DIR);

		//Start Driver and Motor
		digitalWrite(ENABLE, LOW);			// Enable motor

		// Move Home
		stepper.setSpeedInStepsPerSecond(SPEED);
		stepper.setAccelerationInStepsPerSecondPerSecond(ACCEL);
		if (stepper.moveToHomeInSteps(-1, SPEED, MAX_RANGE, STALL) != true) {

			// this code is executed only if homing fails because it has moved farther 
			// than maxHomingDistanceInMM and never finds the limit switch, blink the 
			// LED fast forever indicating a problem
			while (true)
			{
				digitalWrite(LED, HIGH);
				delay(50);
				digitalWrite(LED, LOW);
				delay(50);
			}
		}
		stepper.setCurrentPositionInSteps(0);
	//---------------------------------*
}
// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// MAIN PROGRAM (LOOP):
void loop() {
	// Move to Position
	Serial.println("+ MOVE UP");
	stepper.moveToPositionInSteps(-4000);
	Serial.println("~ STOPPED");
	delay(2000);

	Serial.println("- MOVE DOWN");
	stepper.moveToPositionInSteps(0);
	Serial.println("~STOPPED");
	delay(2000);

}
/*
void loop1() {
	if (stalled == true) {
		Serial.println("********************* STALLED *********************");
		stalled = false;
	}
	// Only print if TSTEP has changed by +/- 10:
	static int previousTSTEP = 0; // Variable to store previous TSTEP value

	int currentTSTEP = driver.TSTEP(); // Get the current TSTEP value

	if (abs(currentTSTEP - previousTSTEP) >= 10) {
		Serial.print("TSTEP: ");
		Serial.println(currentTSTEP);
		previousTSTEP = currentTSTEP; // Update the previous TSTEP value
	}
	delay(1);
}
*/
// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++