#include "StateManager.h"

bool StateManager::SwitchState(IState& state) {
    // if (_currentState != state) {
    _currentState = state;
    // }
    return true;
}