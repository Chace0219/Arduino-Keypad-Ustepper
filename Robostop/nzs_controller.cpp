

#include "nzs_controller.h"
#include <arduino.h>

NszCommandProc::NszCommandProc()
{

}

void NszCommandProc::init()
{
	COMMPORT.begin(NSZBAUDRATE);
}

void NszCommandProc::setCtrlMode(uint8_t mode)
{
	String commandStr = "ctrlmode ";
	commandStr += String(mode);
	// 
	// COMMPORT.println(PROMPT);
	// 
	COMMPORT.println(commandStr);
}

void NszCommandProc::moveToPosition(double angle, int32_t speed)
{
	String commandStr = "move ";
	commandStr += String(angle, 2);
	commandStr += " ";
	commandStr += String(speed);
	// Serial.println(commandStr);
	COMMPORT.println(commandStr);
}

void NszCommandProc::moveSteps(uint8_t direction, int32_t stepCnt)
{
	String commandStr = "step ";
	commandStr += String(direction);
	commandStr += " ";
	commandStr += String(stepCnt);

	// Serial.println(commandStr);
	COMMPORT.println(commandStr);
}

bool NszCommandProc::readPos(double &currPos)
{
	String commandStr = "readpos";
	// 
	// COMMPORT.println(PROMPT);
	// 
	COMMPORT.println(commandStr);
	uint32_t lastRespTime = millis();
	bool bReceiving = false;

	const uint32_t TIMEOUTMS = 500;
	const uint32_t INTERVALTIMOUT = 25;
	String response = "";
	while (true)
	{
		if (COMMPORT.available())
		{
			bReceiving = true;
			lastRespTime = millis();
			char ch = COMMPORT.read();
			response += String(ch);
		}
		if (bReceiving)
		{
			if (millis() - lastRespTime > INTERVALTIMOUT)
				// character interval timeout happened
				break;
		}
		else
		{
			if (millis() - lastRespTime > TIMEOUTMS)
			{
				Serial.println("timeout happens in communication between nsz");
				return false;
			}

		}
	}

	// encoder 182.25
	if (response.indexOf("encoder ") != -1)
	{
		String encoderValue;

		int16_t startPoint = response.indexOf("encoder");
		startPoint += 8;
		int16_t endPoint = response.indexOf("\n", startPoint);
		encoderValue = response.substring(startPoint, endPoint);
		currPos = encoderValue.toDouble();
		
		Serial.print("current postion in nsz ");
		Serial.println(currPos, 2);
	}
	else
	{
		Serial.println("invalid response");
		return false;
	}

	return true;
}

bool NszCommandProc::setMicroStep(uint8_t steps)
{
	String commandStr = "microsteps " +  String(steps);
	COMMPORT.println(commandStr);
}


void NszCommandProc::stopMove()
{
	String commandStr = "stop";
	COMMPORT.println(commandStr);

}

void NszCommandProc::setPos(double position)
{
	String commandStr = "setpos ";
	commandStr += String(position, 2);
	COMMPORT.println(commandStr);
}

void NszCommandProc::enablePinMode(bool enable)
{
	String commandStr = "enablepinmode ";
	if (enable)
		commandStr += "0";
	else
		commandStr += "1";
	COMMPORT.println(commandStr);
}

void NszCommandProc::setZero()
{
	String commandStr = "setzero";
	COMMPORT.println(commandStr);
}
