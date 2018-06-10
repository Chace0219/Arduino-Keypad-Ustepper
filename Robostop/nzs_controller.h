

#ifndef NSZ_CONTROLLER_H_
#define NSZ_CONTROLLER_H_

#include <arduino.h>

#define COMMPORT Serial3
#define NSZBAUDRATE 115200

// control mode constants
const uint8_t CONTROLOFF = 0;
const uint8_t OPENLOOP = 1;
const uint8_t SIMPLEPID = 2;
const uint8_t CURRENTPID = 3;
const uint8_t VELOCITYPID = 4;

const char PROMPT[] = ":>";

#define DEBUGOUTPUT
class NszCommandProc
{
public:
	NszCommandProc();

	void init();
	void setCtrlMode(uint8_t mode = SIMPLEPID);
	void moveToPosition(double angle, int32_t speed = 30);
	void moveSteps(uint8_t direction, int32_t stepCnt);
	bool readPos(double &currPos);
	bool setMicroStep(uint8_t steps);
	void stopMove();
	void setPos(double position);
	void setZero();
	void enablePinMode(bool enable);
	void calibrate();
	void setMaxCurrent(uint32_t value);
	void setHoldCurrent(uint32_t value);
	void setStepsPerRotation(uint32_t value);
private:

};



#endif 
