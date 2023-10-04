#include "ready_to_drive.h"

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

  // convert serial input to volts scale
  float APPS_1_VOLTS_in = APPS_1_Serial_in * (5.0/1023.0);
  float APPS_2_VOLTS_in = APPS_2_Serial_in * (5.0/1023.0);
  float BPSF_VOLTS_in = BPSF_Serial_in * (5.0/1023.0);
  float BPSB_VOLTS_in = BPSB_Serial_in * (5.0/1023.0);

  APPS_2_VOLTS_in = 0.0;
  BPSF_VOLTS_in = 0.0;
  BPSB_VOLTS_in = 0.0;

  if ((0.5 <= APPS_1_VOLTS_in) && (4.5 >= APPS_1_VOLTS_in)) {
    Serial.println("APPS_1_in");
  }
  if ((0.5 <= APPS_2_VOLTS_in) && (4.5 >= APPS_2_VOLTS_in)) {
    Serial.println("APPS_2_in");
  }
  if ((0.5 <= BPSF_VOLTS_in) && (4.5 >= BPSF_VOLTS_in)) {
    Serial.println("BPSF_in");
  }
  if ((0.5 <= BPSB_VOLTS_in) && (4.5 >= BPSB_VOLTS_in)) {
    Serial.println("BPSB_in");
  }
  
}