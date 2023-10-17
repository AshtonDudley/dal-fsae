#include <Arduino.h>
#include "states/IState.h"
#include "states/ReadyToDrive.h"
#include "states/Run.h"
#include "StateManager.h"
	ReadyToDrive s0 = ReadyToDrive();
	Run s1 = Run();
	StateManager manager(s0);
void setup() {
	// put your setup code here, to run once:
	Serial.println("init");
	manager._currentState.print();
	manager.SwitchState(s1);
	manager._currentState.print();
	Serial.begin(9600);
}


void loop() {
	// put your main code here, to run repeatedly:

	Serial.print("State: ");
	manager._currentState.print();
	// delay(1);
}