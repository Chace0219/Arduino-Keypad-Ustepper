

#include "Key.h"

// default constructor
Key::Key() {
	kchar = NO_KEY;
	kstate = IDLE;
	stateChanged = false;
}

// constructor
Key::Key(char userKeyChar) {
	kchar = userKeyChar;
	kcode = -1;
	kstate = IDLE;
	stateChanged = false;
}


void Key::key_update (char userKeyChar, KeyState userState, boolean userStatus) {
	kchar = userKeyChar;
	kstate = userState;
	stateChanged = userStatus;
}



/*
|| @changelog
|| | 1.0 2012-06-04 - Mark Stanley : Initial Release
|| #
*/
