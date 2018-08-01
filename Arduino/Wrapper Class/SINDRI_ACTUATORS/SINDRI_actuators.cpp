#include "SINDRI_actuators.h"


Pump::Pump(int pin, float SAT_FLOW)
{
    _pin = pin;
    _SAT_FLOW = SAT_FLOW;
};

void Pump::begin()
{
    Serial.print("Pump at pin: ");
    Serial.print(_pin);
    Serial.print(" initialized!\n");
};

void Pump::motor_write(float flowrate)
{
    float clipped = constrain(flowrate, 0, _SAT_FLOW);
    float max_pwm = 255*(_SAT_FLOW);
    float pwm_sig = max_pwm - (max_pwm/_SAT_FLOW);
    analogWrite(_pin, (int)pwm_sig);
};
