// function to check inputs for failure scenario
bool failureCheck(){
  return false;
}

// function to check states, should be running at all times
void stateCheck() {
    // set all states as false
    // check accelerator for action
    if (failureCheck() == true) {}
    } else if (acceleratorVolts > 0) { // check for accelleration
      drivingState = True;
    } else if ((acceleratorVolts == 0) && (drivingState == True)) { // check for release of acceleration while in driving state
      regenState = True;
    } else if () { // 
      
    }
}
// function containing ready to drive code
void readyToDrive() {
  return;
}
// function containing regeneration code
void regeneration() {
  return;
}

// setup, runs before main loop
void setup () {
  Serial.begin(9600);
}

// main loop continuously runs
void loop () {
  // declare states
  bool regenState = false;
  bool readyToDriveState = false;
  bool failureState = false;

  // input simulating accellerator
  int acceleratorSerial = analogRead(A0); // gets analog reading from pin A0 connected to potentiometer
  float acceleratorVolts = acceleratorSerial * (5.0/1023.0); // simple unit conversion to volts from bits (1023 bit scale to 5V scale)
  Serial.println(readVolt); // prints value to serial output terminal
  

  // ready to drive state (idle: awaiting input)
  while ((readyToDriveState == true) && (failureState == false)) {
    // check for state switch
    stateCheck();
  }
  // driving state (currently in motion not)
  while ((drivingState = true) && (failureState == false)) {
    // check for state switch
    stateCheck();

    // regenerative breaking state
    while ((regenState = true) && (failureState == false)) {
      // check for state switch 
      stateCheck();
    }
  }


}