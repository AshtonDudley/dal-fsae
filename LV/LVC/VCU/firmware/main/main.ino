//#include "ready_to_drive.h"
#define MAXTHROTTLEVOLT 4.5
#define MINTHROTTLEVOLT 0.5

// converts analog inuput to voltage
float serial_to_volt(int serial, float MAX_VOLT, int ANALOG_IN) {
  float volt = serial * MAX_VOLT/ANALOG_IN;
  return volt;
}

float convert_range(float old_value, float old_max, float old_min, float new_max, float new_min) {
  return (((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min);
}

// check if a value is within a range
bool in_range(float VALUE, float MAX_VALUE, float MIN_VALUE) {
  if ((VALUE > MIN_VALUE) && (VALUE < MAX_VALUE)) {
    return true;
  } else {
    return false;
  }
}

// ready to drive sound function
void readyToDriveSound(int pin_out) {
  // configure pin as output
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
  // define inputs
  int APPS_1_in = A1; // accellerator pedal 1
  int APPS_2_in = A2; // accellerator pedal 2
  int BPSF_in = A3; // break pressure signal front 
  int BPSB_in = A4; // break pressure signal back
  int TQS_in = NA; // tourque system in (for faults)
  int FWRD_in = D10; // forward drive trigger from dash
  int RVRS_in = D11; // reverse drive trigger from dash
  
  // define outputs
  int FWRD_out = D5; // forward motor signal out
  int RVRS_out = D6; // reverse motor signal out


  // gets analog reading from inputs
  int APPS_1_Serial_in = analogRead(APPS_1_in);
  int APPS_2_Serial_in = analogRead(APPS_2_in);
  int BPSF_Serial_in = analogRead(BPSF_in);
  int BPSB_Serial_in = analogRead(BPSB_in);
  int FWRD_Serial_out = analogRead(FWRD_out);
  int RVRS_Serial_out = analogRead(RVRS_out);
  
  // gets digital reading from inputs
  int TQS_Digital_in = digitalRead(TQS_in); 
  int FWRD_Digital_in = digitalRead(FWRD_in);
  int RVRS_Digital_in = digitalRead(RVRS_in);

  // set variables withouth input (for testing)
  BPSF_Serial_in = 0;
  BPSB_Serial_in = 0;

  // convert serial input to volts scale
  float APPS_1_VOLTS_in = serial_to_volt(APPS_1_Serial_in, 5.0, 1023.0);
  float APPS_2_VOLTS_in = serial_to_volt(APPS_2_Serial_in, 5.0, 1023.0);
  float BPSF_VOLTS_in = serial_to_volt(BPSF_Serial_in, 5.0, 1023.0);
  float BPSB_VOLTS_in = serial_to_volt(BPSB_Serial_in, 5.0, 1023.0);
  float FWRD_VOLTS_in = serial_to_volt(FWRD_Serial_out, 5.0, 1023.0);
  float RVRS_VOLTS_in = serial_to_volt(RVRS_Serial_out, 5.0, 1023.0);

  // Tourque system fault check (not aloud to drive when there is a fault)
  if (TQS_Digital_in == LOW) {
    
    // check that the throttle signal maintain 5% offset from eachother: APPS implausibility check
    float throttle_tolterance = MAXTHROTTLEVOLT*0.05;
    bool thottlesMatch = false;
    //
    if ((APPS_1_VOLTS_in <= (APPS_2_VOLTS_in + throttle_tolterance)) && (APPS_1_VOLTS_in >= (APPS_2_VOLTS_in - throttle_tolterance))) {
      throttlesMatch = true;
    } else {
      throttlesMatch = false;
    }

    // check current throttle signals are within operating range
    if (in_range(APPS_2_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT) && in_range(APPS_1_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT)) { // throttle signals within range to drive motors
      // check if forward or reverse is active
      if ((FWRD_Digital_in == HIGH) && thottlesMatch) {
        FWRD_Serial_out = convert_range((APPS_1_VOLTS_in + APPS_2_VOLTS_in)/2, MAXTHROTTLEVOLT, MINTHROTTLEVOLT, 0, 1024); // thottle out 1-5 volts to the forward signal
      } else if ((RVRS_Digital_in == HIGH) && thottlesMatch) {
        FVRS_Serial_out = convert_range((APPS_1_VOLTS_in + APPS_2_VOLTS_in)/2, MAXTHROTTLEVOLT, MINTHROTTLEVOLT, 0, 1024); // thottle out 1-5 volts to the reverse signal
      }
    }
  }

  // check break signal within operating range 
  if (in_range(BPSF_VOLTS_in, 4.5, 0.5)) {
    Serial.println("BPSF_in");
  }
  if (in_range(BPSB_VOLTS_in, 4.5, 0.5)) {
    Serial.println("BPSB_in");
  }
}