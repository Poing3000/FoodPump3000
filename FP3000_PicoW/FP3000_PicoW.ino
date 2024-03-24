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
 * [x] - Prime (find endstop)
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
 *    [x] - Detect Calibration Error (dif. IR vs Stall)
 *    [ ] - Handle unexpected Stall
 *    [ ] - EMGY Mode
 * [ ] - Comunicate
 * [ ] - Handle/implement multicore. >> Comunication on Core 0, Motor on Core 1.
 * [x] - Implement multible pumps control.
 * [ ] - Make libraries local (past in repository and change from #include <LibraryFile.h>  to "LocalFile.h")
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
 * -- Status/Error/Warning sytnax ---
 * E		0			0
 * ^		^			^
 * Type		Device		Info/Problem
 * ==========================================================
 */


 // Parts for individual configuration are highlited below.
 // ----------------------------------------------------------
 // 
 // CONFIGURATION:
 // ----------------------------------------------------------
 // Libraries
	// Stepper
#include "FP3000.h"
#include <Wire.h>

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


// Stepper Settings
 
// General Stepper Driver Settings (change according to your connections to the stepper driver / your needs).
// Library settings (TMCStepper.h) can be foound at: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
	// Driver (here default for TMC2209)
#define DRIVER_ENABLE		2			// Enable Pin
#define	SERIAL_PORT_1		Serial1		// HardwareSerial port (TX: 0, RX: 1)
#define R_SENSE				0.11f		// Sense resistor value of the driver fur current cal.

#define CURRENT				400			// Max current (mA) supplied to the motor
#define	STALL_VALUE			0			// Stall threshold [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value. Set to 0 if you want stall values loaded from file.
#define HOME_STALL_VALUE	0			// Stall threshold for homing [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value. Set to 0 if you want stall values loaded from file.
#define MIRCO_STEPS			32			// Set microsteps (32 is a good compromise between CPU load and noise)
#define TCOOLS				400			// max 20 bits

// Stepper Motor (NEMA 17)
#define SPEED				10000		// Speed (steps/s) (10000 is good)
//#define SPEED				1000		// DELTE LATER! TESTING ONLY
#define ACCEL				100000		// Acceleration (steps/s^2) (100000	is good)
#define STD_FEED_DIST		4600		// Standard range (steps) the slider should moves when feeding (4600 is good)
#define	PUMP_MAX_RANGE		6000		// Max range (steps) the slider can move inside the pump (6000 is good)

// Unique Motor Settings (change according to your connections to the stepper driver)
// Stepper Motor 0
#define MOTOR_0				0			// Unique device number
#define	STEP_0				4			// Step pin
#define	DIR_0				5			// Direction pin
#define	LIMIT_0				99			// Limit switch pin (via expander MCP23017)
#define	DIAG_0				3			// DIAG pin for stall detection
#define	DRIVER_ADDRESS_0	0b00		// Drivers address (0b01: MS1 is LOW and MS2 is HIGH)			
#define DIR_TO_HOME_0			1			// Direction to home (1 = CW, -1 = CCW)

// Stepper Motor 1
#define MOTOR_1				1			// Unique device number
#define	STEP_1				7			// Step pin
#define	DIR_1				8			// Direction pin
#define	LIMIT_1				99			// Limit switch pin (via expander MCP23017)
#define	DIAG_1				6			// DIAG pin for stall detection
#define	DRIVER_ADDRESS_1	0b10		// Drivers address (0b01: MS1 is LOW and MS2 is HIGH)			
#define DIR_TO_HOME_1		-1			// Direction to home (1 = CW, -1 = CCW)

// Scale Settings
#define SCALE_1				2			// Unique device number
#define SCALE_NVM_1			1			// Memory Address (for permanent calibration data)
#define DATA_PIN_1			12			// Data pin for scale 0
#define CLOCK_PIN_1			14			// Clock pin for scale 0

// PurrPleaser Board Settings
#define MCP_ADDRESS			0x20		// Port expander address
#define MCP_INTA			15			// Interrupt pin A
#define MCP_INTB			21			// Interrupt pin B
#define SLC_PIN				17			// SLC pin
#define SDA_PIN				16			// SDA pin
#define	VV_EN				22			// 5V Enable Pin
#define	EXPANDER			true		// Use expander (true) or not (false)

// Other Settings
#define APP_OFFSET			4			// Offset in g for approx. feeding
 
//---------------------------------*

// Port Expander
MCP23017 mcp = MCP23017(MCP_ADDRESS);
//---------------------------------*


// TESTING - DELETE LATER
uint32_t  FIFO_c0 = 0;			// FIFO message from Core 0 to Core 1
uint32_t  FIFO_c1 = 0;			// FIFO message from Core 1 to Core 0
uint32_t  FIFO_R_c0 = 0;		// FIFO message read from at Core 0
uint32_t  FIFO_R_c1 = 0;		// FIFO message read from at Core 1
//---------------------------------*

// Create Pumps
// Add pumps here if needed (e.g. 
FP3000 Scale_0(MOTOR_0, STD_FEED_DIST, PUMP_MAX_RANGE, DIR_TO_HOME_0, SPEED, STALL_VALUE, HOME_STALL_VALUE, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_0, mcp, EXPANDER, MCP_INTA);
FP3000 Pump_1(MOTOR_1, STD_FEED_DIST, PUMP_MAX_RANGE, DIR_TO_HOME_1, SPEED, STALL_VALUE, HOME_STALL_VALUE, SERIAL_PORT_1, R_SENSE, DRIVER_ADDRESS_1, mcp, EXPANDER, MCP_INTA);
//---------------------------------*

// END OF CONFIG+++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// SETUP - CORE 0:
void setup() {

	// Debugging
	Debug.setDebugLevel(DEBUG_LEVEL);
	if (Debug.getDebugLevel() >= 0) {	// Give time to open serial monitor
		delay(200);					
	}
	//---------------------------------*

	// Setup Core 0 finished, allow Core 1 to start
	rp2040.fifo.push(1); // 1 = OK

	//---------------------------------*
}

// SETUP - CORE 1:
void setup1() {

	// Setup Codes
	enum SetupCodes : byte {
		NOT_STARTED,
		OK,
		ERROR,
		WARNING
	};

	while (rp2040.fifo.pop() != OK);

	uint8_t type = 'S';		// S = Status, by default
	uint8_t device = 99;	// 99 = no device, just a placeholder
	uint16_t info = OK;		// OK = no problem, by default

	delay(100); // Give Core 0 a head start, just in case.
	pinMode(LED_BUILTIN, OUTPUT); // DELETE LATER OR IMPLEMENT IN ERROR HANDLING

	//Stepper Setup
	//---------------------------------
	// Pin Setup
	pinMode(DRIVER_ENABLE, OUTPUT);
	pinMode(DRIVER_ENABLE, HIGH);
	pinMode(VV_EN, OUTPUT);
	pinMode(DIAG_1, INPUT);
	digitalWrite(VV_EN, HIGH);

	// TODO: UPADTE PIN SETUP FOR EXPANDER
	pinMode(15, INPUT);


	// Driver Setup
	SERIAL_PORT_1.begin(115200);

	// Setup Expander
	Wire.setSCL(17);
	Wire.setSDA(16);
	Wire.begin();
	if (!mcp.Init()) {	// Check if MCP23017 is connected
		type = 'E';
		device = 10;	// MCP23017
		info = 0;		// No MCP Error
		PackPushData(type, device, info);
	}
	mcp.setPortMode(0b10000000, A);					// set GPA IN-/OUTPUTS (GPA/B 7 needs to be OUTPUT))
	mcp.setPortMode(0b11000011, B);					// set GPB IN-/OUTPUTS
	mcp.setInterruptPinPol(HIGH);					// set INTA and INTB active-high
	mcp.setInterruptOnChangePort(0b00000111, A);	// set interrupt-on-change for all pins on port A
	mcp.getIntCap(A);								// clear interrupt capture on port A

	// Setup Pumps
	byte setupResult = NOT_STARTED;					// Return from setup functions
	digitalWrite(DRIVER_ENABLE, LOW);				// Enable Driver

		// Setup Scale 0
		setupResult = Scale_0.SetupScale(SCALE_NVM_1, DATA_PIN_1, CLOCK_PIN_1);
		if (setupResult != OK) {
			ReceiveWarningsErrors_c1(Scale_0, SCALE_1);
		}

		// Setup Motor 0
		setupResult = Scale_0.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_0, DIR_0, LIMIT_0, DIAG_0, ACCEL);
		if (setupResult != OK) {
			ReceiveWarningsErrors_c1(Scale_0, MOTOR_0);
		}

		// Setup Motor 1
		setupResult = Pump_1.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_1, DIR_1, LIMIT_1, DIAG_1, ACCEL);
		if (setupResult != OK) {
			ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
		}
	//---------------------------------*
	
	// Setup finished
	digitalWrite(LED_BUILTIN, HIGH);  // Visual indication that hardware setup is finished
}


// END OF SETUP++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// MAIN PROGRAM (LOOP):
	// CORE 0
void loop() {

	// TESTING - DELETE LATER
	// CHECKING FOR DATA FROM CORE 1
	while (!PopAndDebug_c0());

}

// CORE 1
void loop1() {
	
	// Variables
	//-------------------------------------------------------------
	// The Core 1 loop switches between different operating modes.
	// The mode is set by Core 0. Default is IDLE.
	enum Mode : byte {
		IDLE,
		FEED,
		CALIBRATE,
		AUTOTUNE,
		EMGY
	};
	static byte Mode_c1 = IDLE;
	//static byte Mode_c1 = CALIBRATE; // DELETE

	// Syntax for function returns
	enum ReturnCode : byte {
		BUSY,
		OK,
		ERROR,
		WARNING
	};
	static byte scaleReturn = BUSY;
	static byte pumpReturn = BUSY;

	// Feeding Modes
	enum FeedingMode : byte {
		PRIME,
		APPROX,
		ACCURATE,
		EMPTY
	};
	static byte feedMode = PRIME;

	// TODO ADAPT FEEDING AMOUNT / RECEICE FROM CORE 0
	// Feeding Amount
	float feedAmount = 10;

	// Calibration Status
	byte calStatus = 0;
	byte prevCalStatus = 1;
	//-------------------------------------------------------------

	// Operating Modes
	switch (Mode_c1) {
	case IDLE:

		// ===============================================================
		// IDLE Mode is the default mode, where the device is waiting for
		// commands from Core 0.
		// ===============================================================

		// Power Off unused devices
		Power_c1(false);


		// DELETE - TESTING
		Mode_c1 = FEED; // DELETE
		


		// Check sensors once in a while

		// Communicate with Core 0, receive operating mode.


		break;
	case FEED:

		// ===============================================================
		// This mode is to automatically dispense food until the desired
		// amount is reached. The amount is set by Core 0 and is measured
		// in grams by the selected scale. The feeding process works in
		// four steps:
		// 1. Prime: Go to start position (endstops) and tare the scale.
		// 2. Approx.: Move slider up and down to get an approximate
		//    amount of food, close to the desired amount (APP_OFFSET).
		// 3. Accurate: Move slider in a precise filling motion until
		//    the desired amount is reached.
		// 4. Empty: Do a final measurement and empty the scale dumper.
		// NOTE, the final scale reading is sent to Core 0. Whereat
		// Core 0 should save the data in order to compensate a given 
		// error in the next feeding process. (E.g. if the pump has
		// dispensed 2g too much, Core 0 should subtract 2g from the next
		// feeding command.)
		// ===============================================================

		//TODO: IMPLEMENT ERROR HANDLING - MAX CYCLES

		switch (feedMode) {
		case PRIME:
			// Turn on power
			Power_c1(true);

			// Prime Scale
			if (scaleReturn == BUSY) {
				scaleReturn = Scale_0.Prime();
				if (scaleReturn == ERROR || scaleReturn == WARNING) {
					ReceiveWarningsErrors_c1(Scale_0, MOTOR_0);
				}
			}


			// Wait for Scale to finishe, then Prime Pump
			else if (pumpReturn == BUSY) {
				pumpReturn = Pump_1.Prime();
				if (pumpReturn == ERROR || pumpReturn == WARNING) {
					ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
				}
			}

			// Check if priming is finished
			// If reset flags and go to next mode.
			if (scaleReturn != BUSY && pumpReturn != BUSY) {
				scaleReturn = BUSY;
				pumpReturn = BUSY;
				feedMode = APPROX;
			}
			break;
		case APPROX:
			// Do one cycle, then check if the desired amount
			// (APP_OFFSET) is reached. If not, do another cycle.
			if (Pump_1.MoveCycle() != BUSY) {
				if (Scale_0.Measure(2) >= feedAmount - APP_OFFSET) {
					feedMode = ACCURATE;
				}
				// Check if there was something to warn about (stall).
				ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
			}

			break;
		case ACCURATE:
			// Accurate
			if (Pump_1.MoveCycleAccurate() != BUSY) {
				if (Scale_0.Measure(3) >= feedAmount) {
					
					// Return slider back to home position and do final measurement
					if (Pump_1.MoveTo(0) != BUSY) {
						
						// Do Final Measurement and send data to Core 0
						// (Uses floatToUint16 to convert measured float to uint16_t)
						PackPushData('M', SCALE_1, floatToUint16(Scale_0.Measure(7)));
						
						// Go to next mode
						feedMode = EMPTY;
					}
				}
			}
			break;
		case EMPTY:
			// Return slider back to home position
			if (Pump_1.MoveTo(0) != BUSY) {

				// Empty Scale
				if(Scale_0.EmptyScale() != BUSY) {
					// Check if there was something to warn about (stall).
					ReceiveWarningsErrors_c1(Scale_0, MOTOR_0);
					ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);
					
					// Back to IDLE
					Mode_c1 = IDLE;
				}
			}
			break;
		}
		break;

	case CALIBRATE:

		// ===============================================================
		// Calibrates selected scales.
		// You can choose between verbose and silent calibration:
		// Verbose calibration: Shows calibration steps on Serial Monitor
		// Silent calibration will use the debug messeages / transmits
		// them to Core 0.
		// ===============================================================

		// Verbose calibration
		//Scale_0.CalibrateScale(true);
		
		// Silent calibration
		// -------------------------
		// CalStatus:
		// 0 - WAITING
		// 1 - TARE
		// 2 - PLACE_WEIGHT
		// 3 - CALIBRATING
		// 4 - SAVEING_CALIBRATION
		// 5 - FINISHED
		// 6 - CALIBRATION_ERROR
		// -------------------------
		while (calStatus <= 4) { // 5 = Calibration successful			
			// Calibrate
			calStatus = Scale_0.CalibrateScale(false);
			// Send calibration updates to Core 0.
			if (prevCalStatus != calStatus) {
				PackPushData('C', 0, calStatus);
				prevCalStatus = calStatus;
			}
			// (No need to implement error handling here, as it should be user detecable.)
		}
		
		// This is blocking code, back to IDLE
		Mode_c1 = IDLE;

		break;
	case AUTOTUNE:

		// ===============================================================
		// Autotunes stall detection for selected devices.
		// You can toggle "quick check" and "save to file":
		// Quick check: Will quickly check the stall detection, though
		// might not show the most accurate results (usually good enough).
		// Quick check set to false: Will be much slower but may give
		// more accurate results.
		// Save to file: Will save the stall values to NVM. Thus, they can
		// be loaded automatically on startup.
		// NOTE, stall values will be read from NVM if STALL_VALUE /
		// HOME_STALL_VALUE is set to 0. E.g. if only STALL_VALUE is set
		// to 0, only this will be read from NVM, but not for homeing.
		// ===============================================================

		// Turn on power
		Power_c1(false);
		
		// Autotune Stall
		// (true/true for quick check and save to file)
		Scale_0.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Scale_0, MOTOR_0);

		Pump_1.AutotuneStall(true, true);
		ReceiveWarningsErrors_c1(Pump_1, MOTOR_1);

		// Turn off power
		Power_c1(false);

	case EMGY:
		// [...]
		//TODO: Function, Emergency Mode
		break;
	default:
		// Unexpected Mode, go to EMGY
		Mode_c1 = EMGY;
		break;
	}

	// Send mode (Status) to Core 0, if changed
	static byte oldMode_c1 = 99; // force sending first status
	if (Mode_c1 != oldMode_c1) {
		PackPushData('S', 99, Mode_c1);
		oldMode_c1 = Mode_c1;
	}


	//---------------------------------*
}

// END OF MAIN PROGRAM+++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// SUPPORT FUNCTIONS:


// Feeding Function



// Function to receive warnings and errors
void ReceiveWarningsErrors_c1(FP3000 &device, byte deviceNumber) {

	byte type;
	static uint16_t info_W = 0; // default 0 = no warning
	static uint16_t info_E = 0; // default 0 = no error

	// Check Warnings
	type = 'W';
	info_W = device.CheckWarning();
	if (info_W != 0) {
		PackPushData(type, deviceNumber, info_W);
	}
	
	// Check Errors
	type = 'E';
	info_E = device.CheckError();
	if (info_E != 0) {
		PackPushData(type, deviceNumber, info_E);
	}
}

// Function to pack and push data
void PackPushData(uint8_t type, uint8_t device, uint16_t info) {
	uint32_t data = ((uint32_t)type << 24) | ((uint32_t)device << 16) | info;
	rp2040.fifo.push(data);
}

// Function to unpack data from a single uint32_t for FIFO transport
void unpackData(uint32_t data, char& type, uint8_t& device, uint16_t& info) {
	type = static_cast<char>((data >> 24) & 0xFF);
	device = (data >> 16) & 0xFF;
	info = data & 0xFFFF;
}

// Function to save float measurements to uint16_t
uint16_t floatToUint16(float value) {
	value = value * 100;
	uint16_t uValue = static_cast<uint16_t>(value);
	return uValue;
}

// Function convert data uint16_t to float
float uint16ToFloat(uint16_t value) {
	float fValue = static_cast<float>(value);
	fValue = fValue / 100.0;
	return fValue;
}

// Function to pop and debug data from Core 1
// Returns true if data was popped and provides debug messages
bool PopAndDebug_c0() {

	char type;
	uint8_t device;
	uint16_t info;
	uint32_t data;
	bool popped = false;

	// Status Codes Messeages
	// ==========================================================
	const char* STATUS_MESSAGES[] = {
	  "Standby",
	  "Feeding",
	  "Calibrating",
	  "EMERGENCY FEEDING"
	};

	// Error Codes Messeages
	// ==========================================================
	const char* ERROR_MESSAGES[] = {
	  "MCP Error",
	  "Driver connection error",
	  "Stepper unknown error",
	  "Stepper jammed",
	  "Scale connection error",
	  "File system error"
	};
	// =========================================================*

	// Warning Codes Messeages
	// ==========================================================
	const char* WARNING_MESSAGES[] = {
	  "No warning",
	  "Stepper sluggish",
	  "Endstop defective",
	  "Stall detected",
	  "Not calibrated",
	  "Stall value not set"
	};
	// =========================================================*

	// Calibration Codes Messeages
	// ==========================================================
	const char* CALIBRATION_MESSAGES[] = {
	  "20s time to remove all weight!",
	  "Taring..",
	  "20s time to place 20g!",
	  "Calibrating..",
	  "Saving calibration value to file..",
	  "Calibration successful.",
	  "Calibration failed."
	};
	// =========================================================*

	// Check if data is available
	int dataCount = rp2040.fifo.available();

	for (int i = 0; i < dataCount; i++) {
		if(rp2040.fifo.pop_nb(&data)) {
			// Unpack data from FIFO
			unpackData(data, type, device, info);

			// Debug message
			if (type == 'S') {
				DEBUG_DEBUG("%s", STATUS_MESSAGES[info]);
			}
			else if (type == 'M') {
				float finfo = uint16ToFloat(info);
				DEBUG_DEBUG("Scale (device#) %d: %.2fg", device, finfo);
			}
			else if (type == 'C') {
				DEBUG_DEBUG("Calibration Scale %d: %s", device, CALIBRATION_MESSAGES[info]);
			}
			else if (type == 'W') {
				DEBUG_WARNING("WARNING Device: %d, %s", device, WARNING_MESSAGES[info]);
			}
			else {
				DEBUG_ERROR("ERROR Device: %d, %s", device, ERROR_MESSAGES[info]);
				// TODO: Implement EMGY Mode
			}
			popped = true;
		}
		else {
			// This would be an unexpected error.
			DEBUG_WARNING("Error while popping data from Core 1");
		}
	}
	return popped;
}

// Power On/Off unused devices
void Power_c1(bool power) {

	// Default power state is ON since power is set ON at setup.
	static bool prevPower = true;

	if (power != prevPower) {
		if (power) {
			digitalWrite(DRIVER_ENABLE, LOW);	// Enable Driver
			digitalWrite(VV_EN, HIGH);			// Enable 5V
		}
		else {
			digitalWrite(DRIVER_ENABLE, HIGH);	// Disable Driver
			digitalWrite(VV_EN, LOW);			// Disable 5V
		}
		prevPower = power;
	}
}

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


		/*
		* 	printDeviceStatus("Scale", Scale_0);
		*   printDeviceStatus("Pump", Pump_1);
		* 
		* 
		void printDeviceStatus(const String &deviceName, FP3000 &device) {
			Serial.println(deviceName + " Error Status: " + String(device.CheckError()));
			Serial.println(deviceName + " Warning Status: " + String(device.CheckWarning()));
		}
		*/

		// DELETE TESING
		/*
		Serial.print("New Autotune Stall Result for Scale_0: ");
		Serial.println(Scale_0.AutotuneStall(true));
		Serial.print("New Autotune Stall Result for Pump_1: ");
		Serial.println(Pump_1.AutotuneStall(true));
		//Scale_0.AutotuneStall(true);
		//Pump_1.AutotuneStall(true);


				while (1) {


			//delay(100);
			Scale_0.MotorTest(false);
			Pump_1.MotorTest(true);
			//mcp.setPin(0, B, HIGH);
			//delay(100);
			Scale_0.MotorTest(true);
			Pump_1.MotorTest(false);
			//mcp.setPin(0, B, LOW);

		}
		digitalWrite(DRIVER_ENABLE, HIGH);			  // Disable Driver

		
		// Function to pack data into a single uint32_t for FIFO transport
		uint32_t packData(uint8_t type, uint8_t device, uint16_t info) {
		return ((uint32_t)type << 24) | ((uint32_t)device << 16) | info;
		}

		delay(250);
		Serial.print("UNITS: ");
		Serial.println(Scale_0.Measure());


		*/