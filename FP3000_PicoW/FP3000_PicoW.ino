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
 * [ ] - Make libraies local (past in repository and change from #include <LibraryFile.h>  to "LocalFile.h")
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
#include "FP3000.h"
#include <TMCStepper.h> //DELETE LATER
#include <SpeedyStepper4Purr.h> //DELETE LATER
#include <Wire.h>
#include <MCP23017.h>


// Debugging
#include <Arduino_DebugUtils.h>

// Local Functions
// [...]
// ---------------------------------*


// Variables CORE 0
	//TODO: CHECK IF NEEDED / CAN BE USED LOCALY
const int Feed_Range_c0 = 4600;	// Typical feeding distance

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

	//END CORE 0 ---------------------------------*

	// Variables CORE 1
		//TODO: CHECK IF NEEDED / CAN BE USED LOCALY
byte Mode_c1 = 0;			// Mode Core 1 (Pumps); start with IDLE
byte Power_c1 = true;			// Indication if motors and secondaries are powered

// Stepper Settings
/*
	   // General Stepper Driver Settings (change according to your connections to the stepper driver / your needs).
	   // Library settings (TMCStepper.h) can be foound at: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
		   // Driver (here default for TMC2209)
		   #define ENABLE				15			// Enable Pin
		   #define	SERIAL_PORT_2		Serial2		// HardwareSerial port (TX: 8, RX: 9)
		   #define R_SENSE				0.11f		// Sense resistor value of the driver fur current cal.
		   #define	DRIVER_ADDRESS_1	0b00		// Drivers address

		   #define CURRENT				400			// Max current (mA) supplied to the motor
		   #define	STALL_VALUE			50			// Stall threshold [0..255] (lower = more sensitive) - 30 is quite low, yet safe for stable error management (50 for accuracy)
		   #define MIRCO_STEPS			32			// Set microsteps (32 is a good compromise between CPU load and noise)
		   #define TCOOLS				400			// max 20 bits

		   // Stepper Motor (NEMA 17)
		   #define SPEED				10000		// Speed (steps/s) (10000 is good)
		   #define ACCEL				100000		// Acceleration (steps/s^2) (100000	is good)
		   #define STD_FEED_DIST		4600		// Standard range (steps) the slider should moves when feeding (4600 is good)
		   #define	PUMP_MAX_RANGE		6000		// Max range (steps) the slider can move inside the pump (6000 is good)

	   // Unique Motor Settings (change according to your connections to the stepper driver)
		   // Stepper Motor 0
		   #define MOTOR_0				0			// Uniuqe stepper motor number
		   #define	STEP_0				12			// Step pin
		   #define	DIR_0				11			// Direction pin
		   #define	LIMIT_0				4			// Limit switch pin
		   #define	DIAG_0				13			// DIAG pin for stall detection
		   #define DIR_TO_HOME			-1			// Direction to home (1 = CW, -1 = CCW)
*/
// General Stepper Driver Settings (change according to your connections to the stepper driver / your needs).
// Library settings (TMCStepper.h) can be foound at: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
	// Driver (here default for TMC2209)
#define DRIVER_ENABLE		2			// Enable Pin
#define	SERIAL_PORT_1		Serial1		// HardwareSerial port (TX: 0, RX: 1)
#define R_SENSE				0.11f		// Sense resistor value of the driver fur current cal.

#define CURRENT				400			// Max current (mA) supplied to the motor
#define	STALL_VALUE			50			// Stall threshold [0..255] (lower = more sensitive) - 30 is quite low, yet safe for stable error management (50 for accuracy)
#define MIRCO_STEPS			32			// Set microsteps (32 is a good compromise between CPU load and noise)
#define TCOOLS				400			// max 20 bits

// Stepper Motor (NEMA 17)
#define SPEED				10000		// Speed (steps/s) (10000 is good)
#define ACCEL				100000		// Acceleration (steps/s^2) (100000	is good)
#define STD_FEED_DIST		4600		// Standard range (steps) the slider should moves when feeding (4600 is good)
#define	PUMP_MAX_RANGE		6000		// Max range (steps) the slider can move inside the pump (6000 is good)

// Unique Motor Settings (change according to your connections to the stepper driver)
	// Stepper Motor 1
#define MOTOR_1				1			// Uniuqe stepper motor number
#define	STEP_1				7			// Step pin
#define	DIR_1				8			// Direction pin
// ! CHANGE LIMIT PIN TO EXPANDER PIN AFTER TESTING !
#define	LIMIT_1				28			// Limit switch pin (via expander MCP23017)
#define	DIAG_1				6			// DIAG pin for stall detection
#define	DRIVER_ADDRESS_1	0b10		// Drivers address (0b01: MS1 is LOW and MS2 is HIGH)			
#define DIR_TO_HOME			-1			// Direction to home (1 = CW, -1 = CCW)

// PurrPleaser Board Settings
#define MCP_ADDRESS			0x20		// Port expander address
#define SLC_PIN				17			// SLC pin
#define SDA_PIN				16			// SDA pin
#define	VV_EN				22			// 5V Enable Pin
#define	EXPANDER			false		// Use expander (true) or not (false)

MCP23017 myMCP = MCP23017(MCP_ADDRESS);


//---------------------------------*



// TESTING - DELETE LATER
uint32_t  FIFO_c0 = 0;			// FIFO message from Core 0 to Core 1
uint32_t  FIFO_c1 = 0;			// FIFO message from Core 1 to Core 0
uint32_t  FIFO_R_c0 = 0;		// FIFO message read from at Core 0
uint32_t  FIFO_R_c1 = 0;		// FIFO message read from at Core 1

// Flags
//[...]
//---------------------------------*

// Create Pumps
// Add pumps here if needed (e.g. 
FP3000 Pump_0(MOTOR_1, STD_FEED_DIST, DIR_TO_HOME, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_1);
//---------------------------------*


// END OF CONFIG+++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// FUNCTIONS:
//[...]

// END OF FUNCTIONS++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SETUP - CORE 0:
void setup() {

	rp2040.idleOtherCore(); // Stop Core 1

	//DELETE DELAY FOR SERIAL
	delay(2000);

	// Debugging
	Debug.setDebugLevel(DEBUG_LEVEL);

	//---------------------------------*

	// Setup Core 0 finished, allow Core 1 to start
	DEBUG_DEBUG("Setup Core 0 finished");
	rp2040.restartCore1();

}

// SETUP - CORE 1:
void setup1() {

	delay(1000); // Give Core 0 a head start
	pinMode(LED_BUILTIN, OUTPUT); // DELETE LATER OR IMPLEMENT IN ERROR HANDLING

	//Stepper Setup
	//---------------------------------
		// Pin Setup
	pinMode(DRIVER_ENABLE, OUTPUT);
	pinMode(VV_EN, OUTPUT);
	pinMode(DIAG_1, INPUT);
	digitalWrite(VV_EN, HIGH);


	// Driver Setup
	SERIAL_PORT_1.begin(115200);

	// Setip Expander
	Wire.setSCL(17);
	Wire.setSDA(16);
	Wire.begin();
	// TODO: IMPLEMENT ERROR HANDLING
	if (!myMCP.Init()) {
		DEBUG_DEBUG("I2C ERROR");
	}
	myMCP.setPortMode(0b10000000, A);  // (GPA&B need to OUTPUT)
	myMCP.setPortMode(0b11000011, B);  // (GPA&B need to OUTPUT)

	// Setup Pumps
	Pump_0.SetupPump(CURRENT, STALL_VALUE, MIRCO_STEPS, TCOOLS, STEP_1, DIR_1, LIMIT_1, DIAG_1, SPEED, ACCEL, PUMP_MAX_RANGE, EXPANDER);


	//TESTING/IMPLEMENT IF NEEDED
	Serial.print(F("\nTesting connection to Pump 1..."));
	byte conResult = Pump_0.Test_Connection();
	if (conResult) {
		Serial.println(F("failed!"));
		Serial.print(F("Likely cause: "));
		switch (conResult) {
		case 1: Serial.println(F("loose connection")); break; //status - 0xFFFFFFFF
		case 2: Serial.println(F("Likely cause: no power")); break; //status 2
		}
		Serial.println(F("Fix the problem and reset board."));
		abort();
	}
	else {
		Serial.println(F("OK")); //staus 0
	}






	//Start Pump 
	digitalWrite(DRIVER_ENABLE, LOW);			  // Enable Driver

	//---------------------------------*

	// Setup finished
	DEBUG_DEBUG("Setup Core 1 finished");
	digitalWrite(LED_BUILTIN, HIGH);  // DELETE LATER OR IMPLEMENT IN ERROR HANDLING
}


// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// MAIN PROGRAM (LOOP):
	// CORE 0
void loop() {
	// [...]

	/* FIFO 0 TEST - DELETE LATER
	FIFO_R_c0 = rp2040.fifo.pop();
	DEBUG_VERBOSE("FIFO_0: %d ", FIFO_R_c0);
	FIFO_c0++;
	*/

	// DIAG/STALL TEST - DELETE LATER
	if (digitalRead(DIAG_1) == LOW) {
		digitalWrite(LED_BUILTIN, LOW);
	}
	else {
		digitalWrite(LED_BUILTIN, HIGH);
	}


}

// CORE 1
void loop1() {

	// Main Pump Functions
	switch (Mode_c1) {
	case 0: // IDLE
		// [...]
		// DELETE LATER! DEBUG/TESTING
		while (1) {
			//delay(100);
			Pump_0.Test(true);
			//myMCP.setPin(0, B, LOW);
			//delay(100);
			Pump_0.Test(false);
			//myMCP.setPin(0, B, HIGH);
		}
		digitalWrite(DRIVER_ENABLE, HIGH);			  // Diable Driver


		//TODO: Function, power of motors, etc.
		//TODO: Other idle functions, such as communication and reading sensors if needed.
		break;
	case 1: // FEED
		// [...]
		//TODO: Function, Feeding (Prime, Approx., Accurate, Empty)
		break;
	case 2: // CALIBRATE
		// [...]
		//TODO: Function, Calibrate Scales
		//TODO: Function, Calibrate Slider [?]
		break;
	case 3: // EMGY
		// [...]
		//TODO: Function, Emergency Mode
		break;
	default:
		// [...]
		// TODO: Function, Error handling
		break;
	}

	//---------------------------------*

	/* FIFO 1 TEST - DELETE LATER
	rp2040.fifo.push(FIFO_c1);
	FIFO_c1++;
	//delay(10);
	*/
}

// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// Code Dump / Templates - DELTE:
		/*
		// DELETE - TESTING ONLY: Move Home
		bool home_test = false;
		byte home_result = 0;
		byte error_result = 0;
		while (1) {
			home_result = stepper_1.moveToHome(-1, SPEED, MAX_RANGE, true);

			if (home_result > 0) {
				if (home_result >= 2) {
					DEBUG_VERBOSE("Error occured");
					if ((stepper_1.ErrorHandling(1, home_result, SPEED, MAX_RANGE) == 0)) {
						//HOME AGAIN
					}
					else {
						//EMGY
					}
				}

				DEBUG_VERBOSE("error_result: %d", error_result);
			break;
			}
		} DEBUG_VERBOSE("home_result: %d", home_result);
		*/