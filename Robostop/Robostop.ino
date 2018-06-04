
// 
#include "LiquidCrystal_I2C.h"
#include "Keypad.h"
#include "FBD.h"
#include "FiniteStateMachine.h"
#include "nzs_controller.h"

//// required varialbes
const uint32_t MOVINGSPEED = 75; // move speed
const uint32_t HOMINGSPEED = 75; // homing speed
const double GANTRYWIDTH = 50.0F;
const double RAILLEN = 200.0F;
const double MMPERREV = 40.0; // mm per 360 degree

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

State startup(NULL);
State absolute(NULL); // ABSOLUTE Measure Mode
State relative(NULL); // RELATIVE Move Mode
State homing(NULL); // Homing Mode
State nudgeAdjust(NULL); // Nudge ADJUST Mode
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


	// 
	keypad.setHoldTime(3000);
	keypad.addEventListener(keypadEvent);
	displayStartup();

	//
	relPos = 0.0;
	absPos = getAbsolutePos();

	// predictAbsPos = 100;
	// stepperMachine.transitionTo(stepperActive);

}


double getAbsolutePos()
{
	double absRad, absPos;
	if (stepper.readPos(absRad))
	{
		Serial.print(F("current absolute position is "));
		absPos = (absRad / 360) * MMPERREV;
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

TON RightStopKeepTON(2000);

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
		if (rightStopTON.Q)
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
		value = absPos / 2.54F;

	if (value > 99.0F) {}
	else if (value > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";

	absPosStr += String(value, 3);
	if (unit == METRIC)
		absPosStr += "MM   ";
	else
		absPosStr += "IN   ";
	screen.print(absPosStr);

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
		screen.print(F("BEYOND TRAVEL LIMIT "));
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
		value = absPos / 2.54F;

	if (value > 99.0F) {}
	else if (value > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";
	absPosStr += String(value, 2);
	if (unit == METRIC)
		absPosStr += "MM ";
	else
		absPosStr += "IN ";
	screen.print(absPosStr);

	String relPosStr;
	screen.setCursor(0, 2);
	screen.print(F("REL POS: "));
	// relPos = 12.5;
	value = 0;
	if (unit == METRIC)
		value = relPos;
	else
		value = relPos / 2.54;

	if (abs(value) > 99.0F) {}
	else if (abs(value) > 9.0F)
		relPosStr += " ";
	else
		relPosStr += "  ";
	if (relPos >= 0.0)
		relPosStr += "+";
	relPosStr += String(value, 2);
	if (unit == METRIC)
		relPosStr += "MM  ";
	else
		relPosStr += "INCH";
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
		screen.print(F("BEYOND TRAVEL LIMIT "));
		// screen.print(F("INVALID INPUT VALUE "));
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
		screen.print(F("#1 -0.1IN #3 +0.1IN "));

	screen.setCursor(0, 2);
	if (unit == METRIC)
		screen.print(F("#4 -1MM   #6 +1MM   "));
	else
		screen.print(F("#4 -1IN   #6 +1IN   "));

	screen.setCursor(0, 3);
	
	//
	double value;
	String strLine = "";
	strLine += "Abs:";
	
	//
	if (unit == METRIC)
		value = absPos;
	else
		value = absPos / 2.54F;

	if (value >= 100.0F)
		strLine += String(value, 1);
	else if(value >= 10.0F)
		strLine += String(value, 2);
	else
		strLine += String(value, 3);

	strLine += " Rel:";
	if (unit == METRIC)
		value = relPos;
	else
		value = relPos / 2.54F;

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
		strLine += String(value, 3);

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
						specifiedValue = specifiedValue * 2.54F;

					absPos = getAbsolutePos();

					double value = absPos;
					if (absoluteMoveSign == PLUS)
						value += specifiedValue;
					else if (absoluteMoveSign == MINUS)
						value -= specifiedValue;
					else
						value = specifiedValue;

					if (value >= 0.0F && value <= RAILLEN)
					{
						Serial.print(F("target absolute position is "));
						Serial.println(value, 2);
						displayAbsoluteMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
						predictAbsPos = value;
					}
					else
					{
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
						specifiedValue = specifiedValue * 2.54;

					absPos = getAbsolutePos();
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
						predictAbsPos += (value - absPos);
					else
						predictAbsPos -= (value - absPos);
					if (predictAbsPos >= 0.0F && predictAbsPos <= RAILLEN)
					{
						Serial.print(F("target absolute position is "));
						Serial.println(predictAbsPos);
						displayRelativeMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
					}
					else
					{ // 
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
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
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
					predictAbsPos = RAILLEN / 2;
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
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
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
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
					{
						Serial.print(F("target step count is "));
						Serial.println(value);
						predictAbsPos = value;
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '7':
				{
					double specifiedValue = -10.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("-10.0mm"));
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
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
					double specifiedValue = 0.1;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+0.1mm"));
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
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
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
					{
						Serial.print(F("target step count is "));
						Serial.println(value);
						predictAbsPos = value;
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '9':
				{
					double specifiedValue = 10.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+10.0mm"));
					double value = getAbsolutePos();
					value += specifiedValue;
					if (value >= 0.0F && value <= RAILLEN)
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
			else if ((char)key == '-' || (char)key == '+')
			{
				Serial.println(F("display unit will be switched"));
				if (unit == METRIC)
					unit = IMPERAL;
				else
					unit = METRIC;
				if (screenMachine.isInState(startup))
					displayStartup();
				else if (screenMachine.isInState(absolute))
					displayAbsoluteMode(true, NORMAL);
				else if (screenMachine.isInState(relative))
					displayRelativeMode(true, NORMAL);
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
	stepper.stopMove();
}

void stepperIdleUpdate() {}
void stepperIdleExit() {}
void stepperActiveEnter()
{
	nLastCheckPosTime = millis();
	double targetRad = (predictAbsPos / MMPERREV) * 360.0F;
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
			double value = getAbsolutePos();
			if (abs(predictAbsPos - value) < 1.0)
				stepperMachine.transitionTo(stepperIdle);
			else
			{
				syncRelPos();

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

	if (screenMachine.isInState(startup))
		displayStartup();
	else if (screenMachine.isInState(absolute))
		displayAbsoluteMode(true, NORMAL);
	else if (screenMachine.isInState(relative))
		displayRelativeMode(true, NORMAL);
	else if (screenMachine.isInState(nudgeAdjust))
		displayNudgeMode();
}

void syncRelPos()
{
	double value = getAbsolutePos();
	if (homeDir == POSITIVE)
		relPos += (value - absPos);
	else
		relPos -= (value - absPos);
	absPos = value;
}

void stepperLeftEnter()
{
	stepper.moveToPosition((RAILLEN / MMPERREV) * 360, HOMINGSPEED);
}

void stepperLeftUpdate()
{
	if (leftStopTON.Q || leftStopTrg.Q)
	{
		Serial.println(F("left end switch detected"));
		Serial.println(F("entered into idle status"));
		stepperMachine.transitionTo(stepperIdle);
		stepper.stopMove();

		absPos = getAbsolutePos();
		delay(500);
	}
}

void stepperLeftExit()
{
	stepper.stopMove();
	delay(500);
	syncRelPos();
}

void stepperRightEnter()
{
	
	stepper.moveToPosition((RAILLEN / MMPERREV) * (-360.0), HOMINGSPEED);
}

void stepperRightUpdate()
{
	if (rightStopTON.Q || rightStopTrg.Q)
	{
		Serial.println(F("right end switch detected"));
		Serial.println(F("entered into idle status"));
		stepperMachine.transitionTo(stepperIdle);
		stepper.stopMove();
		delay(500);
		stepper.setZero();
		absPos = getAbsolutePos();
	}
}

void stepperRightExit()
{
	stepper.stopMove();
	delay(500);
	syncRelPos();
}

