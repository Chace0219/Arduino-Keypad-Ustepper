#include <EEPROM.h>


// 
#include "LiquidCrystal_I2C.h"
#include "Keypad.h"
#include "FBD.h"
#include "FiniteStateMachine.h"
#include "nzs_controller.h"

// ID of the settings block
#define CONFIG_VERSION "ls8"
#define CONFIG_START 32

// Example settings structure
struct StoreStruct {
	// This is for mere detection if they are your settings
	char version[4];
	// The variables of your settings
	double mmPerRev;
} setting = {
	CONFIG_VERSION,
	// The default values
	40.0F
};

#define INVALIDPOS 0xFFFFFFFF
//// required varialbes
const uint32_t MOVINGSPEED = 150; // move speed
const uint32_t HOMINGSPEED = 100; // homing speed
const double GANTRYWIDTH = 49.94F;
const double RAILLEN = 250.0F; // correct?
const double RIGTHOFFSET = 1.65F; // 

const double STROKE = (RAILLEN - GANTRYWIDTH) - RIGTHOFFSET;

const uint8_t CTRLMODE = 2;
const uint32_t MAXCURR = 1500;
const uint32_t HOLDCURR = 700;
const uint32_t MICROSTEPS = 16;
const uint32_t STEPSROTATION = 400;

// end switch
const uint8_t LEFTSTOP = A0; // 
const uint8_t RIGHTSTOP = A1;

// 
const uint8_t ROWS = 4; // four rows
const uint8_t COLS = 4; // four columns
char keys[ROWS][COLS] = {
	{ '1','2','3','A' },
{ '4','5','6','B' },
{ '7','8','9','C' },
{ '.','0','#','D' }
};

static bool currMotorEn = true;

// 
uint8_t rowPins[ROWS] = { 37, 35, 33, 31 }; //connect to the row pinouts of the keypad
uint8_t colPins[COLS] = { 29, 27, 25, 23 }; //connect to the column pinouts of the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C screen(0x27, 20, 4);

//
static double restLength = 0;

// 
NszCommandProc stepper;

#define PLUS 0
#define MINUS 1
#define NONESIGN 2
#define EMPTYROW "                    "

// 
const uint8_t NORMAL = 0;
const uint8_t STEPPING = 1;
const uint8_t INVALID = 2;

// POSITIVE - right to left
// NEGATIVE - left to right
#define POSITIVE 1
#define NEGATIVE 0
static bool homeDir = POSITIVE;

#define METRIC false
#define IMPERAL true
static bool unit = METRIC;

// 
static double absPos = 0.0F;
static double relPos = 0.0F;

static uint32_t nLastCheckPosTime;

// 
static double predictAbsPos = 0.0F;
double getAbsolutePos();

// double map function 
float map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//
void displayStartup(bool init = false, uint8_t mode = NORMAL);

// absolute screen variables
uint8_t absoluteMoveSign = NONESIGN;
String absoluteMoveValue = "";
void displayAbsoluteMode(bool init = false, uint8_t status = NORMAL);

// releative screen variables
uint8_t relInputValueSign = NONESIGN;
String relInputValueStr = "";
void displayRelativeMode(bool init = false, uint8_t status = NORMAL);

String calInputValue = "";
void cal1Update();
void cal1Enter();

State startup(NULL);
State absolute(NULL); // ABSOLUTE Measure Mode
State relative(NULL); // RELATIVE Move Mode
State homing(NULL); // Homing Mode
State nudgeAdjust(NULL); // Nudge ADJUST Mode

State ca1Step(cal1Enter, cal1Update, NULL); // 
State ca2Step(NULL); // 
State ca3Step(NULL); // 
State ca4Step(NULL); // 

FiniteStateMachine screenMachine(startup);

void stepperIdleEnter();
void stepperIdleUpdate();
void stepperIdleExit();
State stepperIdle(stepperIdleEnter, stepperIdleUpdate, stepperIdleExit);
void stepperActiveEnter();
void stepperActiveUpdate();
void stepperActiveExit();
State stepperActive(stepperActiveEnter, stepperActiveUpdate, stepperActiveExit);

void stepperLeftEnter();
void stepperLeftUpdate();
void stepperLeftExit();
State stepperToLeft(stepperLeftEnter, stepperLeftUpdate, stepperLeftExit);

void stepperRightEnter();
void stepperRightUpdate();
void stepperRightExit();
State stepperToRight(stepperRightEnter, stepperRightUpdate, stepperRightExit);

void cal5mmEnter();
void cal5mmUpdate();
void cal5mmExit();
State stepperCal5mm(cal5mmEnter, cal5mmUpdate, cal5mmExit);
double cal5mmTarget = 5.0;

void calRevoEnter();
void calRevoUpdate();
State stepperCal40mm(calRevoEnter, calRevoUpdate, NULL);

FiniteStateMachine stepperMachine(stepperIdle);

void initLCD()
{
	screen.begin();
	screen.backlight();
}

void initPorts()
{
	pinMode(LEFTSTOP, INPUT_PULLUP);
	pinMode(RIGHTSTOP, INPUT_PULLUP);
}

void setup()
{
	delay(1000);
	Serial.begin(115200);
	Serial.println(F("program started"));
	initPorts();
	initLCD();

	// 
	stepper.init();

	stepper.setCtrlMode(CTRLMODE);
	stepper.setMaxCurrent(MAXCURR);
	stepper.setHoldCurrent(HOLDCURR);
	// stepper.setMicroStep(MICROSTEPS);
	stepper.setStepsPerRotation(STEPSROTATION);

	// 
	keypad.setHoldTime(3000);
	keypad.addEventListener(keypadEvent);
	displayStartup();

	syncRelPos();

	loadConfig();

	Serial.print(F("current mm per rev is "));
	Serial.println(setting.mmPerRev, 2);
	// stepperMachine.transitionTo(calibrate);
}


double getAbsolutePos()
{
	double absRad, absPos;
	absPos = INVALIDPOS;
	if (stepper.readPos(absRad))
	{
		Serial.print(F("current absolute position is "));
		absPos = (absRad / 360) * setting.mmPerRev;
		Serial.print(absPos, 2);
		Serial.println(F("mm."));
	}
	else
		Serial.println(F("nano zero stepper not responding"));

	return absPos;
}

TON leftStopTON(25);
Rtrg leftStopTrg;
TON rightStopTON(25);
Rtrg rightStopTrg;

TON RightStopKeepTON(3000);

void loop()
{
	leftStopTON.IN = digitalRead(LEFTSTOP) == false;
	leftStopTON.update();
	leftStopTrg.IN = leftStopTON.IN;
	leftStopTrg.update();

	rightStopTON.IN = digitalRead(RIGHTSTOP) == false;
	rightStopTON.update();
	rightStopTrg.IN = rightStopTON.IN;
	rightStopTrg.update();

	RightStopKeepTON.IN = digitalRead(RIGHTSTOP) == false;
	RightStopKeepTON.update();
	if (screenMachine.isInState(startup))
	{
		if (RightStopKeepTON.Q)
		{
			screenMachine.transitionTo(absolute);
			displayAbsoluteMode(true);
		}
	}

	stepperMachine.update();
	screenMachine.update();
	char newKey = keypad.getKey();
}

void displayStartup(bool init, uint8_t mode)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MAKERPARTS ROBOSTOP "));
	screen.setCursor(0, 1);
	screen.print(F("     VERSION 1.0    "));
	screen.setCursor(0, 2);
	screen.print(F("          "));

	if (mode == NORMAL)
	{
		screen.setCursor(0, 3);
		screen.print(F("PRESS 6 TO HOME -> "));
	}
	else if (mode == STEPPING)
	{
		screen.setCursor(0, 3);
		screen.print(F(" MOVING TO HOME -> "));
	}
}

void displayCal1Step()
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:CALIBRATION 1/4"));

	screen.setCursor(0, 1);
	screen.print(F("CURRENT "));
	
	String tempStr = "";
	if (setting.mmPerRev > 99.99F)
	{
		setting.mmPerRev = 99.99;
		saveConfig();
	}

	if (setting.mmPerRev < 10.0F)
		tempStr += " ";
	tempStr += String(setting.mmPerRev, 3);
	tempStr += "MM    ";
	screen.print(tempStr);

	screen.setCursor(0, 2);
	screen.print(F("PRESS ENTER TO HOME "));

	screen.setCursor(0, 4);
	screen.print(F("AND MOVE TO +5MM    "));
}

void displayCal2Step(uint8_t moving)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:CALIBRATION 2/4"));

	screen.setCursor(0, 1);
	screen.print(F("CALIPER SPACE FROM  "));

	screen.setCursor(0, 2);
	screen.print(F("GANTRY AND END PLATE"));

	if (moving)
	{
		screen.setCursor(0, 3);
		screen.print(F("   MOVING STEPPER   "));
	}
	else
	{
		screen.setCursor(0, 3);
		screen.print(F("HIT ENTER WHEN READY"));
	}
}

void displayCal3Step(bool init, bool invalid)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:CALIBRATION 3/4"));

	screen.setCursor(0, 1);
	screen.print(F("CALIPER AMOUNT MOVED"));

	screen.setCursor(0, 3);
	if(invalid)
		screen.print(F("INVALID INPUT VALUE"));
	else
		screen.print(F("HIT ENTER WHEN DONE"));
	
	screen.setCursor(0, 2);
	if (init)
		screen.print(EMPTYROW);
	screen.setCursor(0, 2);
	screen.print("ENTER AMOUNT:");
	screen.print(calInputValue);
}

void displayCal4Step()
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:CALIBRATION 4/4"));

	String tempStr = "";
	screen.setCursor(0, 1);
	screen.print(F("OLD AMOUNT:"));
	if (setting.mmPerRev > 10.0F)
		tempStr += " ";
	tempStr += String(setting.mmPerRev, 3);
	screen.print(tempStr);

	screen.setCursor(0, 1);
	screen.print(F("NEW AMOUNT:"));
	tempStr += String(setting.mmPerRev, 3);
	screen.print(calInputValue);

	screen.setCursor(0, 3);
	screen.print(F("HIT ENTER TO FINISH "));
}

void displayAbsoluteMode(bool init, uint8_t status)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	if (unit == METRIC)
		screen.print(F("MODE:ABS MEASURE MM "));
	else
		screen.print(F("MODE:ABS MEASURE IN "));

	screen.setCursor(0, 1);
	screen.print(F("ABS POS:"));

	// display absolute position
	String absPosStr = "";
	double value;
	if (unit == METRIC)
		value = absPos;
	else
		value = absPos / 25.4F;

	if (value > 99.0F) {}
	else if (value > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";

	if (unit == METRIC)
	{
		absPosStr += String(value, 2);
		absPosStr += "MM    ";
	}
	else
	{
		absPosStr += String(value, 3);
		absPosStr += "IN   ";
	}
	screen.print(absPosStr);

	Serial.println(absPosStr);


	screen.setCursor(0, 3);
	screen.print(F("PRESS ENTER TO MOVE "));

	if (init)
	{
		screen.setCursor(0, 2);
		screen.print(EMPTYROW);
		screen.setCursor(0, 2);
		if (absoluteMoveSign == PLUS)
			screen.print("+");
		else if (absoluteMoveSign == MINUS)
			screen.print("-");
		else
			screen.print("=");
		screen.print(absoluteMoveValue);
		screen.blink_on();
	}

	if (status == INVALID)
	{
		screen.setCursor(0, 2);
		// screen.print(F("BEYOND TRAVEL LIMIT "));
		screen.print(F("REST LENGTH:"));
		double value = restLength;
		if (unit != METRIC)
			value /= 25.4;
		if (value <= 100.0F)
			screen.print(" ");
		if (value <= 10.0F)
			screen.print(" ");
		screen.print(restLength, 2);
		if (unit == METRIC)
			screen.print("MM");
		else
			screen.print("IN");

	}
	else if (status == STEPPING)
	{
		screen.setCursor(0, 2);
		screen.print(F("    MOVING NOW ...  "));
	}

}

void displayRelativeMode(bool init, uint8_t status)
{
	screen.blink_off();

	screen.setCursor(0, 0);
	if (unit == METRIC)
		screen.print(F("MODE:REL MOVE MM    "));
	else
		screen.print(F("MODE:REL MOVE INCH  "));

	screen.setCursor(0, 1);
	screen.print(F("ABS POS:  "));
	// display absolute position
	String absPosStr = "";
	double value;
	if (unit == METRIC)
		value = absPos;
	else
		value = absPos / 25.4F;

	if (value > 99.0F) {}
	else if (value > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";
	if (unit == METRIC)
	{
		absPosStr += String(value, 2);
		absPosStr += "MM ";
	}
	else
	{
		absPosStr += String(value, 3);
		absPosStr += "IN";
	}
	screen.print(absPosStr);
	String relPosStr;
	screen.setCursor(0, 2);
	screen.print(F("REL POS: "));
	// relPos = 12.5;
	value = 0;
	if (unit == METRIC)
		value = relPos;
	else
		value = relPos / 25.4;

	if (abs(value) > 99.0F) {}
	else if (abs(value) > 9.0F)
		relPosStr += " ";
	else
		relPosStr += "  ";
	if (relPos >= 0.0)
		relPosStr += "+";
	if (unit == METRIC)
	{
		relPosStr += String(value, 2);
		relPosStr += "MM  ";
	}
	else
	{
		relPosStr += String(value, 3);
		relPosStr += "IN  ";
	}

	screen.print(relPosStr);

	if (status == NORMAL)
	{
		screen.setCursor(0, 3);
		screen.print(F("MOVING :            "));
		screen.setCursor(9, 3);
		if (relInputValueSign == PLUS)
			screen.print("+");
		else if (relInputValueSign == MINUS)
			screen.print("-");
		else
			screen.print("=");

		screen.print(relInputValueStr);
		screen.blink_on();
	}

	if (status == INVALID)
	{
		screen.setCursor(0, 3);
		// screen.print(F("BEYOND TRAVEL LIMIT "));
		screen.print(F("REST LENGTH:"));
		double value = restLength;
		if (unit != METRIC)
			value /= 25.4;
		if (value <= 100.0F)
			screen.print(" ");
		if (value <= 10.0F)
			screen.print(" ");
		screen.print(restLength, 2);
		if (unit == METRIC)
			screen.print("MM");
		else
			screen.print("IN");

	}
	else if (status == STEPPING)
	{
		screen.setCursor(0, 3);
		screen.print(F("MOVING : MOVING NOW "));
	}
}

void displayHomingMode()
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:HOMING         "));
	screen.setCursor(0, 1);
	screen.print(F("PRESS 4 TO HOME <-- "));
	screen.setCursor(0, 2);
	screen.print(F("PRESS 6 TO HOME --> "));
	screen.setCursor(0, 3);
	screen.print(F("PRESS 5 TO CENTRE   "));
}

void displayNudgeMode()
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:NUDGE ADJUST   "));
	screen.setCursor(0, 1);
	if (unit == METRIC)
		screen.print(F("#1 -0.1MM #3 +0.1MM "));
	else
		screen.print(F("#1 -1/32IN#3 +1/32IN"));

	screen.setCursor(0, 2);
	if (unit == METRIC)
		screen.print(F("#4 -1MM   #6 +1MM   "));
	else
		screen.print(F("#4 -1/16IN#6 +1/16IN"));

	screen.setCursor(0, 3);

	//
	double value;
	String strLine = "";
	strLine += "A:";

	//
	if (unit == METRIC)
		value = absPos;
	else
		value = absPos / 25.4F;

	if (value >= 100.0F)
		strLine += String(value, 1);
	else if (value >= 10.0F)
		strLine += String(value, 2);
	else
	{
		strLine += String(value, 3);
	}

	if (unit == METRIC)
		strLine += "MM";
	else
		strLine += "IN";

	strLine += " R:";
	if (unit == METRIC)
		value = relPos;
	else
		value = relPos / 25.4F;

	if (value < 0.0)
		strLine += "-";
	else
		strLine += "+";

	value = abs(value);
	if (value >= 100.0F)
		strLine += String(value, 1);
	else if (value >= 10.0F)
		strLine += String(value, 2);
	else
		strLine += String(value, 2);
	if (unit == METRIC)
		strLine += "MM";
	else
		strLine += "IN";
	screen.print(strLine);
}

// Take care of some special events.
void keypadEvent(KeypadEvent key) {
	static byte kpadState;
	kpadState = keypad.getState();
	switch (kpadState) {
	case PRESSED:
	{
		Serial.print(F("keypad pressed "));
		Serial.println((char)key);

		if (screenMachine.isInState(startup))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case '0':
				{
					// reset all variables
					absPos = 0.0F;
					relPos = 0.0F;
					stepper.setZero();
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
				}
				break;

				case '6':
				{
					stepperMachine.transitionTo(stepperToRight);
					displayStartup(false, STEPPING);
				}
				break;

				case 'A':
				{
					absoluteMoveSign = PLUS;
					absoluteMoveValue = "";
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
				}
				break;

				case '#':
				{
					Serial.println(F("entered into nudge mode"));
					displayNudgeMode();
					screenMachine.transitionTo(nudgeAdjust);
				}
				break;
				}

			}
			else if (stepperMachine.isInState(stepperActive))
			{
				switch ((char)key)
				{
				case 'C':
				{
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}
		}
		else if (screenMachine.isInState(absolute))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'A':
				{
					screenMachine.transitionTo(relative);
					displayRelativeMode(true);
				}
				break;

				case 'B':
				{
					if (absoluteMoveSign == NONESIGN)
						absoluteMoveSign = PLUS;
					else
						absoluteMoveSign++;
					displayAbsoluteMode(true);
				}
				break;
				case 'C':
				{
					absoluteMoveValue = "";
					displayAbsoluteMode(true);
				}
				break;

				case 'D':
				{
					Serial.println(absoluteMoveValue);
					double specifiedValue = absoluteMoveValue.toDouble();
					Serial.print(F("entered specified value is "));
					if (absoluteMoveSign == PLUS)
						Serial.print(F("+"));
					else if (absoluteMoveSign == MINUS)
						Serial.print(F("-"));
					else
						Serial.print(F(" "));
					Serial.println(specifiedValue, 3);

					if (unit != METRIC)
						specifiedValue = specifiedValue * 25.4F;


					// syncRelPos();
					double value = absPos;
					if (absoluteMoveSign == PLUS)
						value += specifiedValue;
					else if (absoluteMoveSign == MINUS)
						value -= specifiedValue;
					else
						value = specifiedValue;

					if (value >= 0.0F && value <= STROKE)
					{
						Serial.print(F("target absolute position is "));
						Serial.println(value, 2);
						displayAbsoluteMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
						predictAbsPos = value;
					}
					else
					{
						if (value < 0.0)
							restLength = absPos;
						else if(value > STROKE)
							restLength = STROKE - absPos;

						displayAbsoluteMode(false, INVALID);
					}
				}
				break;

				case '#':
				{
					Serial.println(F("entered into nudge mode"));
					displayNudgeMode();
					screenMachine.transitionTo(nudgeAdjust);
				}
				break;

				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					if (absoluteMoveValue == "0")
						absoluteMoveValue = String(key);
					else
						absoluteMoveValue += String(key);
					displayAbsoluteMode(true);
				}
				break;

				case '.':
				{
					if (absoluteMoveValue.length() > 0 && absoluteMoveValue.indexOf(".") == -1)
					{
						absoluteMoveValue += String(key);
						displayAbsoluteMode(true);
					}
				}
				break;

				default:
					break;
				}
			}
			else if (stepperMachine.isInState(stepperActive))
			{
				switch ((char)key)
				{
				case 'C':
				{
					stepper.stopMove();
					Serial.println(F("cancel button pressed on abs mode"));
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}

		}
		else if (screenMachine.isInState(relative))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'A':
				{
					screenMachine.transitionTo(homing);
					displayHomingMode();
				}
				break;

				case 'B':
				{
					if (relInputValueSign == NONESIGN)
						relInputValueSign = PLUS;
					else
						relInputValueSign++;
					displayRelativeMode(true);
				}
				break;
				case 'C':
				{
					relInputValueStr = "";
					displayRelativeMode(true);
				}
				break;

				case 'D':
				{
					double specifiedValue = relInputValueStr.toDouble();
					Serial.print(F("entered specified value is "));
					if (relInputValueSign == PLUS)
						Serial.print(F("+"));
					else if (relInputValueSign == MINUS)
						Serial.print(F("-"));
					else
						Serial.print(F(" "));
					Serial.print(specifiedValue, 3);
					if (unit == METRIC)
						Serial.println(F("mm"));
					else
						Serial.println(F("inch"));

					// 
					if (unit != METRIC)
						specifiedValue = specifiedValue * 25.4;

					double value = relPos;
					if (relInputValueSign == PLUS)
						value += specifiedValue;
					else if (relInputValueSign == MINUS)
						value -= specifiedValue;
					else
						value = specifiedValue;

					// 
					predictAbsPos = absPos;
					if (homeDir == POSITIVE)
						predictAbsPos += (value - relPos);
					else
						predictAbsPos -= (value - relPos);
					if (predictAbsPos >= 0.0F && predictAbsPos <= STROKE)
					{
						Serial.print(F("target absolute position is "));
						Serial.println(predictAbsPos);
						displayRelativeMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
					}
					else
					{ //
						if (value < 0.0)
							restLength = absPos;
						else if (value > STROKE)
							restLength = STROKE - absPos;

						displayRelativeMode(false, INVALID);
					}
				}
				break;

				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					if (relInputValueStr == "0")
						relInputValueStr = String(key);
					else
						relInputValueStr += String(key);
					displayRelativeMode(true);
				}
				break;

				case '#':
				{
					Serial.println(F("entered into nudge mode"));
					displayNudgeMode();
					screenMachine.transitionTo(nudgeAdjust);
				}
				break;

				case '.':
				{
					if (relInputValueStr.length() > 0 && relInputValueStr.indexOf(".") == -1)
					{
						relInputValueStr += String(key);
						displayRelativeMode(true);
					}
				}
				break;

				default:
					break;

				}
			}
			else if (stepperMachine.isInState(stepperActive))
			{
				switch ((char)key)
				{
				case 'C':
				{
					Serial.println(F("cancel button pressed on rel mode"));
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}

		}
		else if (screenMachine.isInState(homing))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'A':
				{
					stepper.enablePinMode(true);
					syncRelPos();
					screenMachine.transitionTo(ca1Step);
				}
				break;

				case '4':
				{
					stepper.enablePinMode(true);
					syncRelPos();
					Serial.println(F("it will go to left end"));
					stepperMachine.transitionTo(stepperToLeft);
				}
				break;

				case '5':
				{
					stepper.enablePinMode(true);
					syncRelPos();
					Serial.println(F("it will go to center positon"));
					predictAbsPos = (RAILLEN - GANTRYWIDTH) / 2;
					stepperMachine.transitionTo(stepperActive);
				}
				break;

				case '6':
				{
					stepper.enablePinMode(true);
					Serial.println(F("it will go to right end"));
					syncRelPos();
					stepperMachine.transitionTo(stepperToRight);
				}
				break;

				case '#':
				{
					stepper.enablePinMode(true);
					Serial.println(F("entered into nudge mode"));
					syncRelPos();
					displayNudgeMode();
					screenMachine.transitionTo(nudgeAdjust);
				}
				break;
				case 'C':
				{
					Serial.println(F("disabled motor on homing mode"));
					stepper.enablePinMode(false);
				}
				break;
				}
			}
			else
			{
				switch ((char)key)
				{
				case 'C':
				{
					Serial.println(F("cancel button pressed on homing mode"));
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}
		}
		else if (screenMachine.isInState(ca1Step))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'A':
				{
					syncRelPos();
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
				}
				break;

				case 'D':
				{
					Serial.println(F("it will go to right end in cal 1/4"));
					stepperMachine.transitionTo(stepperToRight);
				}
				break;
				}
			}
			else
			{
				switch ((char)key)
				{
				case 'C':
				{
					Serial.println(F("cancel button pressed on calibration mode"));
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}
		}
		else if (screenMachine.isInState(ca2Step))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'D':
				{
					Serial.println(F("it will move 360 degree to left"));
					stepperMachine.transitionTo(stepperCal40mm);
				}
				break;
				}
			}
			else
			{
				switch ((char)key)
				{
				case 'C':
				{
					Serial.println(F("cancel button pressed on calibration mode"));
					stepperMachine.transitionTo(stepperIdle);
				}
				break;
				}
			}
		}
		else if (screenMachine.isInState(ca3Step))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'C':
				{
					calInputValue = "";
					displayCal3Step(true, false);
				}
				break;

				case '0':
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					if (calInputValue == "0")
						calInputValue = String(key);
					else
						calInputValue += String(key);
					displayCal3Step(false, false);
				}
				break;

				case '.':
				{
					if (calInputValue.length() > 0 && calInputValue.indexOf(".") == -1)
					{
						calInputValue += String(key);
						displayCal3Step(false, false);
					}
				}
				break;

				case 'D':
				{
					double specifiedValue = calInputValue.toDouble();
					Serial.print(F("entered new calibration value is "));
					Serial.println(calInputValue);
					if (specifiedValue >= 0.0F && specifiedValue <= 99.99)
					{
						Serial.println(F("entered into calibration 4 step"));
						screenMachine.transitionTo(ca4Step);
						displayCal4Step();
					}
					else
					{
						displayCal3Step(false, true);
					}
				}
				break;
				}
			}
		}
		else if (screenMachine.isInState(ca4Step))
		{
			switch ((char)key)
			{
			case 'D':
			{
				Serial.println(F("saved setting in EEPROM"));
				setting.mmPerRev = calInputValue.toDouble();
				saveConfig();
			}
			break;

			case 'A':
			{
				syncRelPos();
				screenMachine.transitionTo(absolute);
				displayAbsoluteMode(true);
			}
			break;
			}
		}
		else if (screenMachine.isInState(nudgeAdjust))
		{
			if (stepperMachine.isInState(stepperIdle))
			{
				switch ((char)key)
				{
				case 'A':
				{
					screenMachine.transitionTo(absolute);
					absoluteMoveSign = NONESIGN;
					absoluteMoveValue = "";
					displayAbsoluteMode(true);
				}
				break;

				case '1':
				{
					double specifiedValue = -0.1;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("-0.1mm"));
					double value = absPos;
					value += specifiedValue;
					if (value >= 0.0F && value <= STROKE)
					{
						predictAbsPos = value;
						Serial.print(F("target step count is "));
						Serial.println(predictAbsPos);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;
				case '4':
				{
					double specifiedValue = -1.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("-1.0mm"));
					double value = absPos;
					value += specifiedValue;
					if (value >= 0.0F && value <= STROKE)
					{
						Serial.print(F("target step count is "));
						Serial.println(value);
						predictAbsPos = value;
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '3':
				{
					Serial.print(F("nudge mode entered specified value is "));
					double specifiedValue = 0.1;
					if (unit == METRIC)
					{
						specifiedValue = (double)25.4 / (double)32;
						Serial.println(F("+0.1mm"));
					}
					else
					{
						specifiedValue = (double)25.4 / (double)32;
						Serial.println(F("+1/32inch"));
					}
					double value = absPos;
					value += specifiedValue;
					if (value >= 0.0F && value <= STROKE)
					{
						Serial.print(F("target step count is "));
						Serial.println(value);
						predictAbsPos = value;
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '6':
				{
					double specifiedValue = 1.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+1.0mm"));
					double value = absPos;
					value += specifiedValue;
					if (value >= 0.0F && value <= STROKE)
					{
						Serial.print(F("target step count is "));
						Serial.println(value);
						predictAbsPos = value;
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				}
			}
		}
	}
	break;

	case HOLD:
	{
		Serial.print(F("keypad hold "));
		Serial.println((char)key);
		if (stepperMachine.isInState(stepperIdle))
		{
			if ((char)key == '0')
			{
				if (screenMachine.isInState(relative))
				{
					Serial.println(F("resetting relative position in relative mode"));
					relPos = 0.0F;
					displayRelativeMode(NORMAL);
				}
			}
			else if ((char)key == '*')
			{
				Serial.println(F("calibrating command has issued"));
				stepper.calibrate();
			}
			else if ((char)key == 'B')
			{
				Serial.println(F("display unit will be switched"));
				if (unit == METRIC)
					unit = IMPERAL;
				else
					unit = METRIC;
				if (screenMachine.isInState(startup))
					displayStartup();
				else if (screenMachine.isInState(absolute))
					displayAbsoluteMode(true);
				else if (screenMachine.isInState(relative))
					displayRelativeMode(true);
				else if (screenMachine.isInState(nudgeAdjust))
					displayNudgeMode();
			}
		}
	}
	break;
	}  // end switch-case
}


void stepperIdleEnter()
{
	// stepper.stopMove();
}

void stepperIdleUpdate() {}
void stepperIdleExit() {}
void stepperActiveEnter()
{
	nLastCheckPosTime = millis();
	double targetRad = (predictAbsPos / setting.mmPerRev) * 360.0F;
	stepper.moveToPosition(targetRad, MOVINGSPEED);
}

static uint32_t lastSteppingTime = millis();

void stepperActiveUpdate()
{
	// 
	if (predictAbsPos > absPos)
	{
		if (leftStopTON.Q)
		{
			Serial.println(F("left stop switch is triggered"));
			stepper.stopMove();

			delay(500);
			stepperMachine.transitionTo(stepperIdle);
		}
	}
	else
	{
		if (rightStopTON.Q)
		{
			stepper.stopMove();
			delay(500);
			Serial.println(F("right stop switch is triggered"));
			stepper.setZero();
			setAdjustPos(0);
			stepperMachine.transitionTo(stepperIdle);
		}
	}

	// 
	if (stepperMachine.timeInCurrentState() > 10000)
	{
		stepperMachine.transitionTo(stepperIdle);
	}
	else
	{
		if (millis() - nLastCheckPosTime > 250)
		{
			nLastCheckPosTime = millis();

			syncRelPos();

			if (abs(predictAbsPos - absPos) < 0.1)
			{
				stepperMachine.transitionTo(stepperIdle);
			}
			else
			{

				if (screenMachine.isInState(absolute))
					displayAbsoluteMode(false, STEPPING);
				else if (screenMachine.isInState(relative))
					displayRelativeMode(false, STEPPING);
				else if (screenMachine.isInState(nudgeAdjust))
					displayNudgeMode();
			}
		}
	}
}

void stepperActiveExit()
{
	syncRelPos();
	setAdjustPos(predictAbsPos);
	if (screenMachine.isInState(startup))
		displayStartup();
	else if (screenMachine.isInState(absolute))
		displayAbsoluteMode(true);
	else if (screenMachine.isInState(relative))
		displayRelativeMode(true);
	else if (screenMachine.isInState(nudgeAdjust))
		displayNudgeMode();
}

void setAdjustPos(double predictedPos)
{
	double value = predictedPos;
	if (homeDir == POSITIVE)
		relPos += (value - absPos);
	else
		relPos -= (value - absPos);
	absPos = value;
}


void syncRelPos()
{
	double value = getAbsolutePos();
	if (value == INVALIDPOS)
		value = getAbsolutePos();
	if (homeDir == POSITIVE)
		relPos += (value - absPos);
	else
		relPos -= (value - absPos);
	absPos = value;
}

void stepperLeftEnter()
{
	stepper.moveToPosition((RAILLEN / setting.mmPerRev) * 360, HOMINGSPEED);
}

void stepperLeftUpdate()
{
	if (leftStopTON.Q || leftStopTrg.Q)
	{
		Serial.println(F("left end switch detected"));
		Serial.println(F("entered into idle status"));
		stepperMachine.transitionTo(stepperIdle);
		stepper.stopMove();
		delay(500);
	}
}

void stepperLeftExit()
{
	stepper.stopMove();
	delay(500);
	// syncRelPos();
	setAdjustPos(STROKE);
}

void stepperRightEnter()
{

	stepper.moveToPosition((RAILLEN / setting.mmPerRev) * (-360.0), HOMINGSPEED);
}

void stepperRightUpdate()
{
	if (rightStopTON.Q || rightStopTrg.Q)
	{
		Serial.println(F("right end switch detected"));
		Serial.println(F("entered into idle status"));
		stepper.stopMove();
		delay(500);
		stepper.setZero();
		setAdjustPos(0);
		if (screenMachine.isInState(ca1Step))
			stepperMachine.transitionTo(stepperCal5mm);
		else
			stepperMachine.transitionTo(stepperIdle);
	}
}

void stepperRightExit()
{
	stepper.stopMove();
	delay(500);
	setAdjustPos(0);
}

void loadConfig() {
	if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
		EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
		EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
		for (unsigned int t = 0; t<sizeof(setting); t++)
			*((char*)&setting + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
	for (unsigned int t = 0; t<sizeof(setting); t++)
		EEPROM.write(CONFIG_START + t, *((char*)&setting + t));
}

void cal1Update()
{

}

void cal1Enter()
{
	displayCal1Step();
}

void cal5mmEnter()
{
	stepper.moveToPosition((cal5mmTarget / setting.mmPerRev) * 360.0F, MOVINGSPEED);
	nLastCheckPosTime = millis();
}

void cal5mmUpdate()
{
	if (millis() - nLastCheckPosTime > 250)
	{
		nLastCheckPosTime = millis();
		getAbsolutePos();
		if (abs(cal5mmTarget - absPos) < 0.1)
		{
			stepperMachine.transitionTo(stepperIdle);
		}
	}
}

void cal5mmExit()
{
	// stepper.setZero();
	getAbsolutePos();
	screenMachine.transitionTo(ca2Step);
	displayCal2Step(false);
}

void calRevoEnter()
{
	stepper.moveToPosition(360.0F, MOVINGSPEED);
	nLastCheckPosTime = millis();
	displayCal2Step(true);
}

void calRevoUpdate()
{
	if (millis() - nLastCheckPosTime > 250)
	{
		nLastCheckPosTime = millis();
		double absRad;
		absPos = INVALIDPOS;
		if (!stepper.readPos(absRad))
			Serial.println(F("nano zero stepper not responding"));
		else
		{
			if (abs(absPos - 360.0F) < 0.1)
			{
				stepperMachine.transitionTo(stepperIdle);
				screenMachine.transitionTo(ca3Step);
				displayCal3Step(true, false);
			}
		}
	}
}

