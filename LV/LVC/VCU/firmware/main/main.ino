//#include "ready_to_drive.h"
#define MAXTHROTTLEVOLT 4.5
#define MINTHROTTLEVOLT 0.5

// converts analog inuput to voltage
float serial_to_volt(int serial, float MAX_VOLT, int ANALOG_IN) {
  float volt = serial * MAX_VOLT/ANALOG_IN;
  return volt;
}

// check if a value is within a range
bool in_range(float VALUE, float MAX_VALUE, float MIN_VALUE) {
  if ((VALUE > MIN_VALUE) && (VALUE < MAX_VALUE)) {
    return true;
  } else {
    return false;
  }
}

void check_state() {
  // run necessary checks to determine state and set current state
  int current_state = NULL;

  // return current state
  return current_state;
}

// ready to drive sound function
void readyToDriveSound(int pin_out) {
  // conficgure pin as output
  pinMode(pin_out, OUTPUT);
  
  // turn on and off noise for 1 second (1000 ms)
  digitalWrite(pin_out, HIGH);
  delay(1000);
  digitalWrite(pin_out, LOW);
  delay(1000);
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
/*
  // assign current state class to current state
  class current_state = check_state();
*/
  // define inputs and outputs
  int APPS_1_in = A1;
  int APPS_2_in = A2;
  int BPSF_in = A3;
  int BPSB_in = A4;

  // gets analog reading from voltage inputs
  int APPS_1_Serial_in = analogRead(APPS_1_in);
  int APPS_2_Serial_in = analogRead(APPS_2_in);
  int BPSF_Serial_in = analogRead(BPSF_in);
  int BPSB_Serial_in = analogRead(BPSB_in);

  BPSF_Serial_in = 0;
  BPSB_Serial_in = 0;

  // convert serial input to volts scale
  float APPS_1_VOLTS_in = serial_to_volt(APPS_1_Serial_in, 5.0, 1023.0);
  float APPS_2_VOLTS_in = serial_to_volt(APPS_2_Serial_in, 5.0, 1023.0);
  float BPSF_VOLTS_in = serial_to_volt(BPSF_Serial_in, 5.0, 1023.0);
  float BPSB_VOLTS_in = serial_to_volt(BPSB_Serial_in, 5.0, 1023.0);

  // check if current and throttle signals have not gone out of bounds
  if (in_range(APPS_1_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT)) {
    Serial.println(APPS_1_VOLTS_in);
  }
  if (in_range(APPS_2_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT)) {
    Serial.println(APPS_2_VOLTS_in);
  }
  if (in_range(BPSF_VOLTS_in, 4.5, 0.5)) {
    Serial.println("BPSF_in");
  }
  if (in_range(BPSB_VOLTS_in, 4.5, 0.5)) {
    Serial.println("BPSB_in");
  }

  // check that the throttle signal maintain 5% offset from eachother
  
  float throttle_tolterance = MAXTHROTTLEVOLT*0.05;
  if ((APPS_1_VOLTS_in <= (APPS_2_VOLTS_in + throttle_tolterance)) && (APPS_1_VOLTS_in >= (APPS_2_VOLTS_in - throttle_tolterance))) {
    Serial.println("In range");
    return 0;
  }

  // power up state loop
  void POWER_UP() {

  }

  // wait state loop
  void WAIT() {

  }

  // ready to drive state loop
  void RTD() {
    // signal ready to drive by triggering ready to drive sound
    readyToDriveSound(13);
  }

  // fault state loop
  void FAULT() {

  }

  // run state loop
  void RUN() {

  }
}