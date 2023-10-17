#pragma once

#include <Arduino.h>
#include "IState.h"

class ReadyToDrive : public IState {
public:
    ReadyToDrive() {};
    void print() override;
    void loop() override;
};