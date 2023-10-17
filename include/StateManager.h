#pragma once

#include "states/IState.h"

class StateManager {
public:
    StateManager(IState& state) : _currentState(state) {}
    bool SwitchState(IState&);
    IState& _currentState;
private:
};