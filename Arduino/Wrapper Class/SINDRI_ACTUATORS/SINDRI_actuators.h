/*
 * Actuators.h - Class definitions of all SINDRI actuator types
 * 
*/

#ifndef ACTUATORS_H
#define ACTUATORS_H

// Third Party Libraries
#include <math.h>

// Include Arduino helper 
#include <Arduino.h>

// Xylem Pump
// Class Definition
class Pump
{
    public:
        Pump(int pin, float SAT_FLOW);
        unsigned long time;
        void begin();
        void motor_write(float flowrate);

    private:
        int _pin;
        float _MAX_FLOW = 3.8; // LPM
        float _SAT_FLOW;
};

class Switch
{
    public:
        Switch(int pin){};
};

class Valve
{
    public:
        Valve(int pin){};
};

#endif /// ACTUATORS_H
