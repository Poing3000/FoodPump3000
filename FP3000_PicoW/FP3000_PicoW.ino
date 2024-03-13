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
#define	STALL_VALUE			60			// Stall threshold [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value.
#define HOME_STALL_VALUE	60			// Stall threshold for homing [0..255] (lower = more sensitive) >> use AutotuneStall(bool quickCheck) to find the best value.
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
			ReceiveWarningsErrors_c1(Scale_0, SCALE_1, setupResult);
		}

		// Setup Motor 0
		setupResult = Scale_0.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_0, DIR_0, LIMIT_0, DIAG_0, ACCEL);
		if (setupResult != OK) {
			ReceiveWarningsErrors_c1(Scale_0, MOTOR_0, setupResult);
		}

		// Setup Motor 1
		setupResult = Pump_1.SetupMotor(CURRENT, MIRCO_STEPS, TCOOLS, STEP_1, DIR_1, LIMIT_1, DIAG_1, ACCEL);
		if (setupResult != OK) {
			ReceiveWarningsErrors_c1(Pump_1, MOTOR_1, setupResult);
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
	
	// The Core 1 loop switches between different operating modes.
	// The mode is set by Core 0. Default is IDLE.
	enum Mode : byte {
		IDLE,
		FEED,
		CALIBRATE,
		EMGY
	};
	static byte Mode_c1 = IDLE;
	//static byte Mode_c1 = CALIBRATE; // TESTING ONLY

	// Operating Modes
	switch (Mode_c1) {
	case IDLE:
		
		//Serial.print("UNITS: ");
		//Serial.println(Scale_0.Measure());
		//delay(250);


		// Power Off unused devices
		Power_c1(false);

		// Check sensors once in a while

		// Communicate with Core 0, receive operating mode.


		break;
	case FEED: // FEED
		// [...]
		//TODO: Function, Feeding (Prime, Approx., Accurate, Empty)
		break;
	case CALIBRATE: // CALIBRATE
		// [...]
		//TODO: Function, Calibrate Scales
		Scale_0.CalibrateScale();
		Mode_c1 = IDLE; // TESTING ONLY
		//TODO: Function, Calibrate Slider [?]
		break;
	case EMGY: // EMGY
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

// Function to receive warnings and errors
void ReceiveWarningsErrors_c1(FP3000 &device, byte deviceNumber, byte setupResult) {

	byte type;
	uint16_t info;
	if (setupResult == 3) {		// 3 = WARNING, 2 = ERROR
		type = 'W';
		info = device.CheckWarning();
	}
	else {
		type = 'E';
		info = device.CheckError();
	}
	PackPushData(type, deviceNumber, info);
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
	  "EMERGENCY FEEDING",
	  "Cal. Remove Weight",
	  "Cal. Place 20g",
	};


	// Error Codes Messeages
	// ==========================================================
	const char* ERROR_MESSAGES[] = {
	  "MCP Error",
	  "Driver connection error",
	  "Stepper unknown error",
	  "Stepper jammed",
	  "Scale connection error"
	};
	// =========================================================*

	// Warning Codes Messeages
	// ==========================================================
	const char* WARNING_MESSAGES[] = {
	  "No warning",
	  "Stepper sluggish",
	  "Endstop defective",
	  "Stall detected",
	  "Not calibrated"
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

// Function to pack and push data
void PackPushData(uint8_t type, uint8_t device, uint16_t info) {
	uint32_t data = ((uint32_t)type << 24) | ((uint32_t)device << 16) | info;
	rp2040.fifo.push(data);
}

/*
// Function to pack data into a single uint32_t for FIFO transport
uint32_t packData(uint8_t type, uint8_t device, uint16_t info) {
	return ((uint32_t)type << 24) | ((uint32_t)device << 16) | info;
}
*/

// Function to unpack data from a single uint32_t for FIFO transport
void unpackData(uint32_t data, char& type, uint8_t& device, uint16_t& info) {
	type = static_cast<char>((data >> 24) & 0xFF);
	device = (data >> 16) & 0xFF;
	info = data & 0xFFFF;
}

// Power On/Off unused devices
void Power_c1(bool power) {

	// Power ON
	if (power) {
		digitalWrite(DRIVER_ENABLE, LOW);	// Enable Driver
		digitalWrite(VV_EN, HIGH);			// Enable 5V
	}

	// Power OFF
	else {
		digitalWrite(DRIVER_ENABLE, HIGH);	// Disable Driver
		digitalWrite(VV_EN, LOW);			// Disable 5V
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


		*/