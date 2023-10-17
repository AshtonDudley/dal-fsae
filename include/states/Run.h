#pragma once

#include <Arduino.h>
#include "IState.h"

class Run : public IState {
public:
    Run() {};
    void print() override;
    void loop() override;
};