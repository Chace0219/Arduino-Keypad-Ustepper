#include "LiquidCrystal_I2C.h"
#include "Keypad.h"
#include "FBD.h"
#include "FiniteStateMachine.h"
#include "A4988.h"

// end switch
const uint8_t LEFTSTOP = A0; // 
const uint8_t RIGHTSTOP = A1;

const uint8_t ROWS = 4; //four rows
const uint8_t COLS = 4; //four columns
char keys[ROWS][COLS] = {
	{ '1','2','3','A' },
{ '4','5','6','B' },
{ '7','8','9','C' },
{ '.','0','#','D' }
};

uint8_t rowPins[ROWS] = { 37, 35, 33, 31 }; //connect to the row pinouts of the keypad
uint8_t colPins[COLS] = { 29, 27, 25, 23 }; //connect to the column pinouts of the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Set the LCD address to 0x27 for a 20 chars and 4 line display
LiquidCrystal_I2C screen(0x27, 20, 4);

const uint8_t DIR1 = 13;
const uint8_t STEP1 = 12;
const uint8_t ENA1 = 11;

const uint8_t DIR2 = 6;
const uint8_t STEP2 = 7;
const uint8_t ENA2 = 12;

const int STEPSPERREV = 200;
A4988 stepper(STEPSPERREV, DIR1, STEP1, ENA1);

#define PLUS 0
#define MINUS 1
#define NONESIGN 2
#define EMPTYROW "                    "

//
const uint8_t NORMAL = 0;
const uint8_t STEPPING = 1;
const uint8_t INVALID = 2;

const double RAILLEN = 200.0;
const double STEPSPERMILL = 114.29;
const double MAXSTEP = RAILLEN * STEPSPERMILL;

// POSITIVE - right to left
// NEGATIVE - left to right
#define POSITIVE 0
#define NEGATIVE 1
const bool HOMEDIR = POSITIVE; 
static int32_t absStepCnt = 0;
static double relPos = 0.0F;

#define METRIC false
#define IMPERAL true

static bool unit = METRIC;

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

// increment mode display variables
uint8_t nudgeValueSign = NONESIGN;
String nudgeValueStr = "";
void displayIncrementMode(bool init = false, uint8_t status = NORMAL);

State startup(NULL);
State absolute(NULL); // ABSOLUTE Measure Mode
State relative(NULL); // RELATIVE Move Mode
State increment(NULL); // Increment Move Mode
State homing(NULL); // Homing Mode
State nudgeAdjust(NULL); // Nudge ADJUST Mode
FiniteStateMachine screenMachine(startup);

static int32_t targetStepCnt = 0;
static int32_t movingSteps = 0;
void stepperIdleEnter();
void stepperIdleUpdate();
void stepperIdleExit();
State stepperIdle(stepperIdleEnter, stepperIdleUpdate, stepperIdleExit);
void stepperActiveEnter();
void stepperActiveUpdate();
void stepperActiveExit();
State stepperActive(stepperActiveEnter, stepperActiveUpdate, stepperActiveExit);

void stepperLeftUpdate();
void stepperLeftExit();
State stepperToLeft(NULL, stepperLeftUpdate, stepperLeftExit);

void stepperRightUpdate();
void stepperRightExit();
State stepperToRight(NULL, stepperRightUpdate, stepperRightExit);

FiniteStateMachine stepperMachine(stepperIdle);

void initLCD()
{
	screen.begin();
	screen.backlight();
}

void initMotor()
{
	// 6rpm
	stepper.begin(60);
	stepper.enable();
	//
	stepper.setMicrostep(16); // full microstep
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
	initMotor();
	keypad.setHoldTime(3000);
	keypad.addEventListener(keypadEvent);
	displayStartup();
	
	// targetStepCnt = 3000;
	// stepperMachine.transitionTo(stepperActive);
}

static uint32_t oldPosition = 0;

TON leftStopTON(50);
Rtrg leftStopTrg;
TON rightStopTON(50);
Rtrg rightStopTrg;

TON leftStopKeepTON(2000);

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

	if (leftStopTrg.Q)
		Serial.println(F("left end switch is triggered"));

	if (rightStopTrg.Q)
		Serial.println(F("right end switch is triggered"));

	leftStopKeepTON.IN = digitalRead(LEFTSTOP) == false;
	leftStopKeepTON.update();
	if (screenMachine.isInState(startup))
	{
		if (leftStopKeepTON.Q)
		{
			absStepCnt = MAXSTEP;
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
		screen.print(F("PRESS 4 TO HOME LEFT"));
	}
	else if (mode == STEPPING)
	{
		screen.setCursor(0, 3);
		screen.print(F(" MOVING TO LEFT POS "));
	}
}

void displayAbsoluteMode(bool init, uint8_t status)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:ABS MEASURE MM "));
	screen.setCursor(0, 1);
	screen.print(F("ABS POS:"));
	
	// display absolute position
	String absPosStr = "";
	double absPos = getAbsPos(absStepCnt, HOMEDIR);
	if (absPos > 99.0F) {}
	else if (absPos > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";

	absPosStr += String(absPos, 3);
	absPosStr += "MM   ";
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
		// screen.print(F("INVALID INPUT VALUE "));
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
	screen.print(F("MODE:REL MOVE MM    "));
	screen.setCursor(0, 1);
	screen.print(F("ABS POS:  "));

	// display absolute position
	String absPosStr = "";
	double absPos = getAbsPos(absStepCnt, HOMEDIR);
	if (absPos > 99.0F) {}
	else if (absPos > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";
	absPosStr += String(absPos, 2);
	absPosStr += "MM ";
	screen.print(absPosStr);

	String relPosStr;
	screen.setCursor(0, 2);
	screen.print(F("REL POS: "));
	// relPos = 12.5;
	if (abs(relPos) > 99.0F) {}
	else if (abs(relPos) > 9.0F)
		relPosStr += " ";
	else
		relPosStr += "  ";
	if (relPos >= 0.0)
		relPosStr += "+";
	relPosStr += String(relPos, 2);
	relPosStr += "MM  ";
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

void displayIncrementMode(bool init, uint8_t status)
{
	screen.blink_off();
	screen.setCursor(0, 0);
	screen.print(F("MODE:INCREM MOVE MM "));
	screen.setCursor(0, 1);
	screen.print(F("ABS POS: "));

	// display absolute position
	String absPosStr = " ";
	double absPos = getAbsPos(absStepCnt, HOMEDIR);
	if (absPos > 99.0F) {}
	else if (absPos > 9.0F)
		absPosStr += " ";
	else
		absPosStr += "  ";
	absPosStr += String(absPos, 2);
	absPosStr += "MM ";
	screen.print(absPosStr);

	String relPosStr = "";
	screen.setCursor(0, 2);
	screen.print(F("REL POS: "));
	// relPos = 12.5;
	if (abs(relPos) > 99.0F) {}
	else if (abs(relPos) > 9.0F)
		relPosStr += " ";
	else
		relPosStr += "  ";
	if (relPos >= 0.0)
		relPosStr += "+";
	relPosStr += String(relPos, 2);
	relPosStr += "MM  ";
	screen.print(relPosStr);

	if (init)
	{
		screen.setCursor(0, 3);
		screen.print(EMPTYROW);
	}

	if (status == NORMAL)
	{
		screen.setCursor(0, 3);
		screen.print(F("NUDGE:"));
		if (nudgeValueSign == PLUS)
			screen.print(F("+"));
		else
			screen.print(F("-"));
		screen.print(nudgeValueStr);
		screen.blink_on();
	}
	else if (status == INVALID)
	{
		screen.setCursor(0, 3);
		screen.print(F("NUDGE: INVALID INPUT"));
	}
	else if (status == STEPPING)
	{
		screen.setCursor(0, 3);
		screen.print(F("NUDGE:  MOVING NOW  "));
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
	screen.print(F("#1 -0.1MM #3 +0.1MM "));
	screen.setCursor(0, 2);
	screen.print(F("#4 -1MM   #6 +1MM   "));
	screen.setCursor(0, 3);
	screen.print(F("#7 -10MM  #9 +10MM  "));
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
					absStepCnt = 0;
					relPos = 0;
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
				}
				break;

				case '4':
				{
					stepperMachine.transitionTo(stepperToLeft);
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
			else if(stepperMachine.isInState(stepperActive))
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

					double absPos = getAbsPos(absStepCnt, HOMEDIR);
					if (absoluteMoveSign == PLUS)
						absPos += specifiedValue;
					else if (absoluteMoveSign == MINUS)
						absPos -= specifiedValue;
					else
						absPos = specifiedValue;
					if (absPos >= 0.0F && absPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(absPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						displayAbsoluteMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
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
					screenMachine.transitionTo(increment);
					nudgeValueSign = NONESIGN;
					nudgeValueStr = "";
					displayIncrementMode(true, NORMAL);
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
					Serial.println(specifiedValue, 3);

					double absPos = getAbsPos(absStepCnt, HOMEDIR);
					if (relInputValueSign == PLUS)
						absPos += specifiedValue;
					else if (relInputValueSign == MINUS)
						absPos -= specifiedValue;
					else
						absPos += (specifiedValue - relPos);
							
					if (absPos >= 0.0F && absPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(absPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						displayRelativeMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
					}
					else
					{
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
		else if (screenMachine.isInState(increment))
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
					if (nudgeValueSign == PLUS)
						nudgeValueSign = MINUS;
					else
						nudgeValueSign = PLUS;
					displayIncrementMode(false);
				}
				break;
				case 'C':
				{
					nudgeValueStr = "";
					displayIncrementMode(true);
				}
				break;

				case 'D':
				{
					double specifiedValue = nudgeValueStr.toDouble();
					Serial.print(F("entered specified value is "));
					if (nudgeValueSign == PLUS)
						Serial.print(F("+"));
					else
						Serial.print(F("-"));
					Serial.println(specifiedValue, 3);

					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					if (nudgeValueSign == PLUS)
						currAbsPos += specifiedValue;
					else
						currAbsPos -= specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						displayIncrementMode(false, STEPPING);
						stepperMachine.transitionTo(stepperActive);
					}
					else
						displayIncrementMode(false, INVALID);
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
					if (nudgeValueStr == "0")
						nudgeValueStr = String(key);
					else
						nudgeValueStr += String(key);
					displayIncrementMode(false);
				}
				break;

				case '.':
				{
					if (nudgeValueStr.length() > 0 && nudgeValueStr.indexOf(".") == -1)
					{
						nudgeValueStr += String(key);
						displayIncrementMode(false);
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
					Serial.println(F("cancel button pressed on increment mode"));
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
					screenMachine.transitionTo(absolute);
					displayAbsoluteMode(true);
				}
				break;

				case '4':
				{
					Serial.println(F("it will go to left end"));
					stepperMachine.transitionTo(stepperToLeft);
				}
				break;

				case '5':
				{
					Serial.println(F("it will go to center positon"));
					targetStepCnt = MAXSTEP / 2;
					stepperMachine.transitionTo(stepperActive);
				}
				break;

				case '6':
				{
					Serial.println(F("it will go to right end"));
					stepperMachine.transitionTo(stepperToRight);
				}
				break;

				case '#':
				{
					Serial.println(F("entered into nudge mode"));
					displayNudgeMode();
					screenMachine.transitionTo(nudgeAdjust);
				}
				break;
				case 'C':
				{
					Serial.println(F("cancel button pressed on homing mode"));
					stepperMachine.transitionTo(stepperIdle);
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
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;
				case '4':
				{
					double specifiedValue = -1.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("-1.0mm"));
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '7':
				{
					double specifiedValue = -10.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("-10.0mm"));
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '3':
				{
					double specifiedValue = 0.1;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+0.1mm"));
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '6':
				{
					double specifiedValue = 1.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+1.0mm"));
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
						stepperMachine.transitionTo(stepperActive);
					}
				}
				break;

				case '9':
				{
					double specifiedValue = 10.0;
					Serial.print(F("nudge mode entered specified value is "));
					Serial.println(F("+10.0mm"));
					double currAbsPos = getAbsPos(absStepCnt, HOMEDIR);
					currAbsPos += specifiedValue;
					if (currAbsPos >= 0.0F && currAbsPos <= RAILLEN)
					{
						targetStepCnt = posConvStep(currAbsPos, HOMEDIR);
						Serial.print(F("target step count is "));
						Serial.println(targetStepCnt);
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
			if (screenMachine.isInState(relative))
			{
				switch ((char)key)
				{
				case '#':
				{
					Serial.println(F("resetting relative position in relative mode"));
					relPos = 0.0F;
					displayRelativeMode(NORMAL);
					// displayRelativeMode(true, NORMAL);
				}
				break;
				}
			}
			else
			{
				switch ((char)key)
				{
				case '#':
				{
					Serial.println(F("resetting relative position in increment mode"));
					relPos = 0.0F;
					displayRelativeMode(true, NORMAL);
					screenMachine.transitionTo(relative);
					// displayRelativeMode(NORMAL);
				}
				break;
				}

			}
		}
	}
	break;
	}  // end switch-case
}


void stepperIdleEnter() {}
void stepperIdleUpdate() {}
void stepperIdleExit() {}
void stepperActiveEnter() 
{
	const int32_t MAXINTERVAL = 100;
	if (abs(targetStepCnt - absStepCnt) > MAXINTERVAL)
	{
		if (targetStepCnt - absStepCnt > MAXINTERVAL)
			movingSteps = MAXINTERVAL;
		else
			movingSteps = -MAXINTERVAL;
	}
	else
		movingSteps = targetStepCnt - absStepCnt;

	Serial.print(F("moving steps "));
	Serial.println(movingSteps);
	stepper.move(movingSteps);
	moveRelPos(movingSteps, HOMEDIR);
	absStepCnt += movingSteps;

	Serial.print(F("rel pos is "));
	Serial.print(relPos, 3);
	Serial.print(F(", absStepCnt "));
	Serial.println(absStepCnt);
}

static uint32_t lastSteppingTime = millis();

void stepperActiveUpdate()
{

	if (rightStopTON.Q && movingSteps < 0)
	{
		Serial.println(F("right stop switch is triggered"));
		absStepCnt = 0;
		stepperMachine.transitionTo(stepperIdle);
	}
	if (leftStopTON.Q && movingSteps > 0)
	{
		Serial.println(F("left stop switch is triggered"));
		absStepCnt = MAXSTEP;
		stepperMachine.transitionTo(stepperIdle);
	}

	if (stepperMachine.timeInCurrentState() > abs(movingSteps) / 5)
	{
		if (targetStepCnt == absStepCnt)
			stepperMachine.transitionTo(stepperIdle);
		else
		{
			if (targetStepCnt != absStepCnt)
				stepperMachine.immediateTransitionTo(stepperActive);
		}
	}
}

void stepperActiveExit()
{
	if (screenMachine.isInState(startup))
		displayStartup();
	else if (screenMachine.isInState(absolute))
		displayAbsoluteMode(true, NORMAL);
	else if (screenMachine.isInState(relative))
		displayRelativeMode(true, NORMAL);
	else if (screenMachine.isInState(increment))
		displayIncrementMode(true, NORMAL);
}

void stepperLeftUpdate()
{
	const uint32_t movingStep = 100;
	if ((millis() - lastSteppingTime) > movingStep / 5)
	{
		stepper.move(movingStep); // adjust this value 
		absStepCnt += movingStep;

		if (HOMEDIR == POSITIVE)
		{
			relPos += (double)movingStep / STEPSPERMILL;
		}
		else
		{
			relPos -= (double)movingStep / STEPSPERMILL;
		}
		Serial.print(absStepCnt);
		Serial.print(F(", "));
		Serial.print(relPos, 3);
		Serial.println();

		if (leftStopTON.Q)
		{
			Serial.println(F("left end switch detected"));
			absStepCnt = MAXSTEP;
			Serial.println(F("entered into idle status"));
			stepperMachine.transitionTo(stepperIdle);
		}
		lastSteppingTime = millis();
	}
}

void stepperLeftExit()
{

}

void stepperRightUpdate()
{
	const uint32_t movingStep = 100;
	if ((millis() - lastSteppingTime) > movingStep / 5)
	{
		stepper.move(-movingStep); // adjust this value 
		absStepCnt -= movingStep;

		if (HOMEDIR == POSITIVE)
		{
			relPos -= (double)movingStep / STEPSPERMILL;
		}
		else
		{
			relPos += (double)movingStep / STEPSPERMILL;
		}
		Serial.print(absStepCnt);
		Serial.print(F(", "));
		Serial.print(relPos, 3);
		Serial.println();

		if (rightStopTON.Q)
		{
			Serial.println(F("right end switch is detected"));
			absStepCnt = 0;
			Serial.println(F("entered into idle status"));
			stepperMachine.transitionTo(stepperIdle);
		}
		lastSteppingTime = millis();
	}
}

void stepperRightExit()
{

}

void moveRelPos(int32_t movingSteps, bool direction)
{
	if (direction == POSITIVE)
		relPos += movingSteps / STEPSPERMILL;
	else
		relPos -= movingSteps / STEPSPERMILL;
}

double getAbsPos(uint32_t absSteps, bool direction)
{
	double absPos;
	if (direction == POSITIVE)
	{
		absPos = map_double(absStepCnt, 0, MAXSTEP, 0.0, RAILLEN);
	}
	else
	{
		absPos = map_double(absStepCnt, 0, MAXSTEP, RAILLEN, 0.0);
	}
	return absPos;
}

uint32_t posConvStep(double absPos, bool direction)
{
	uint32_t stepCount;
	if (direction == POSITIVE)
	{
		stepCount = (uint32_t)map_double(absPos, 0.0, RAILLEN, 0.0, MAXSTEP);
	}
	else
	{
		stepCount = (uint32_t)map_double(absPos, 0.0, RAILLEN, MAXSTEP, 0.0);
	}
	return stepCount;
}