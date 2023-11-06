//#include "ready_to_drive.h"
#define MAXTHROTTLEVOLT 4.5
#define MINTHROTTLEVOLT 0.5
#define MAXBREAKVOLT 4.5
#define MINBREAKVOLT 0.5
#define MINREGENVOLT 0
#define MAXREGENVOLT 0.49

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
void readyToDriveSound(const int pin_out) {
  // configure pin as output
  pinMode(pin_out, OUTPUT);
  
  // turn on and off noise for 1 second (1000 ms)
  digitalWrite(pin_out, HIGH);
  delay(1000);
  digitalWrite(pin_out, LOW);
  delay(1000);
}

// define inputs
const int APPS_1_in = A1; // accellerator pedal 1
const int APPS_2_in = A2; // accellerator pedal 2
const int BPSF_in = A3; // break pressure signal front 
const int BPSB_in = A4; // break pressure signal back
const int TQS_in = 100; // tourque system in (for faults)
const int FWRD_in = 10; // forward drive trigger from dash
const int RVRS_in = 11; // reverse drive trigger from dash
const int RTDButton_in = 9; // ready to drive button from dash

// define outputs
const int FWRD_out = 5; // 
const int RVRS_out = 6; // reverse motor signal out
const int RTDMotorControl_out = 4; // motor controller ready to drive, initializes motor controller
const int ACCEL_out = 7; // outputs for tourque output
const int REGEN_out = 8; 


// setup, runs before main loop
void setup () {
  Serial.begin(9600);

  // set pinModes for digital inputs
  pinMode(TQS_in, INPUT);
  pinMode(FWRD_in, INPUT);
  pinMode(RVRS_in, INPUT);
  pinMode(RTDButton_in, INPUT);

  // set pinModes for digital outputs
  pinMode(FWRD_in, INPUT);
  pinMode(RVRS_in, INPUT);
  pinMode(FWRD_out, OUTPUT);
  pinMode(RVRS_out, OUTPUT);
  pinMode(RTDMotorControl_out, OUTPUT);
  pinMode(ACCEL_out, OUTPUT);
  pinMode(REGEN_out, OUTPUT);
}


void loop () {
  // gets analog reading from inputs
  int APPS_1_Serial_in = analogRead(APPS_1_in);
  int APPS_2_Serial_in = analogRead(APPS_2_in);
  int BPSF_Serial_in = analogRead(BPSF_in);
  int BPSB_Serial_in = analogRead(BPSB_in);
  
  // gets digital reading from inputs
  int TQS_Digital_in = digitalRead(TQS_in); 
  int FWRD_Digital_in = digitalRead(FWRD_in);
  int RVRS_Digital_in = digitalRead(RVRS_in);
  int RTDButton_Digital_in = digitalRead(RTDButton_in);

  // convert serial input to volts scale
  float APPS_1_VOLTS_in = serial_to_volt(APPS_1_Serial_in, 5.0, 1023.0);
  float APPS_2_VOLTS_in = serial_to_volt(APPS_2_Serial_in, 5.0, 1023.0);
  float BPSF_VOLTS_in = serial_to_volt(BPSF_Serial_in, 5.0, 1023.0);
  float BPSB_VOLTS_in = serial_to_volt(BPSB_Serial_in, 5.0, 1023.0);




  // start up mode

  // check break is applied within opperating range
  if (in_range(BPSF_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) && in_range(BPSB_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT)) {
    // check ready to drive button is pushed
    if (RTDButton_Digital_in == HIGH) {
      // TRIGGER READY TO DRIVE MODE
      digitalWrite(RTDMotorControl_out, HIGH); // initialize motor controller
    }
  }
  
  
  
  
  
  
  // ready to drive mode

  // Tourque system fault check (not aloud to drive when there is a fault)
  if (TQS_Digital_in == HIGH) {

    // if (either?) break is applied tourque production must stop: APPS/Break Pedal Probability check
    bool breaksEngaged = false;
    if (in_range(BPSF_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT) || in_range(BPSB_VOLTS_in, MAXBREAKVOLT, MINBREAKVOLT)) {
      breaksEngaged = true; // this is applied as a constraint while trying to supply power to the motors
    }

    // check that the throttle signal maintain 5% offset from eachother: APPS implausibility check
    float throttle_tolterance = MAXTHROTTLEVOLT*0.05; // value which is 5% of max powering voltage
    bool throttlesMatch = false;
    if ((APPS_1_VOLTS_in <= (APPS_2_VOLTS_in + throttle_tolterance)) && (APPS_1_VOLTS_in >= (APPS_2_VOLTS_in - throttle_tolterance))) {
      throttlesMatch = true;
    } else {
      throttlesMatch = false;
    }

    // check current throttle signals are within operating range
    if (in_range(APPS_2_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT) && in_range(APPS_1_VOLTS_in, MAXTHROTTLEVOLT, MINTHROTTLEVOLT)) { // throttle signals within range to drive motors (might be able to change to checking just one if they are inside throttle tollerance)
      // check if forward or reverse is active, that throttles match, and if breaks are engauged
      if ((FWRD_Digital_in == HIGH)) {
        digitalWrite(FWRD_out, HIGH); // tell motor controller to drive forward
      } else if ((RVRS_Digital_in == HIGH)) {
        digitalWrite(RVRS_out, HIGH); // tell motor controller to drive in reverse
      }
      if (throttlesMatch && breaksEngaged == false) {
        digitalWrite(ACCEL_out, HIGH); // signal motors to supply tourque
      }
    } else if (in_range(APPS_2_VOLTS_in, MAXREGENVOLT, MINREGENVOLT) && in_range(APPS_1_VOLTS_in, MAXREGENVOLT, MINREGENVOLT)) {
      digitalWrite(REGEN_out, HIGH); // signal motors to supply regen tourque output? not sure about this one
    }
  }
}