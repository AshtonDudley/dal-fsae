#pragma once

class IState {
public:
    virtual void loop() = 0;
    virtual void print() = 0;
};