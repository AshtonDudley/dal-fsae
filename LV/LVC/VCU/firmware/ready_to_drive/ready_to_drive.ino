#include "ready_to_drive.h"

void readyToDriveSound(bool allChecksComplete, char pin_out) {
  if (allChecksComplete == true) {
    digitalWrite(pin_out, HIGH);
    delay(1500);
    digitalWrite(pin_out, LOW);
  }
}