/* 
 * Sensors.h - Class definitions for all SINDRI sensor types
 * Created by Luke Roberto Mar 25th, 2018.
*/

#ifndef SENSORS_H
#define SENSORS_H

// Standard library
#include <StandardCplusplus.h>
#include <deque>
#include <math.h>

// Third party libraries
#include "DHT.h"
#include "max6675.h"

// Include arduino helper functions
#include "Arduino.h"

// SPT25-20-0100A Pressure transmitter
// Class Definition
class Pressure_sensor
{
    public: 
        Pressure_sensor(int pin);
        unsigned long time;
        float current_reading = 0.0;

        void begin();
        void update_sensor(bool filter);
        void log_data();

    private:
        int _pin;
        std::deque<float> _buffer;
};

// Sparkfun MQ-8 H2 Gas Sensor
// Class Definition
class H2_sensor
{
    public:
        H2_sensor(int pin);
        unsigned long time;
        float current_reading = 0.0;
        
        void begin();
        void update_sensor();
        void log_data();

    private:
        int _pin;
};

// DHT Humidity/Temperature Sensor
// Class Definition
class Humidity_sensor
{
    public:
        Humidity_sensor(int pin);
        unsigned long time;
        float current_reading = 0.0;
        
        void begin();
        void update_sensor();
        void log_data();

    private:
        int _pin;
        DHT dht_sensor;
};

// O2 Sensor
// Class Definition
class O2_sensor
{
    public:
        O2_sensor(int pin);
        unsigned long time;

        float current_reading = 0.0;
        void begin();
        void update_sensor();
        void log_data();
    
    private:
        int _pin;
};

// Thermistor
// Class Definition

class Thermistor
{
    public:
        Thermistor(int pin, float R);
        unsigned long time;
        float current_reading = 0.0;

        void begin();
        void update_sensor();
        void log_data();

    private:
        int _pin;
        float _R;
        float _A = 0.24929;
        float _B = -0.049126;
        float _C = 0.00020905;
};

// Thermocouples
// Class Definition
class Thermocouple
{
    public:
        Thermocouple(int DO, int CLK, int CS);
        unsigned long time;
        float current_reading = 0.0;
        
        void begin();
        void update_sensor();
        void log_data();

    private:
        MAX6675 thermocouple;
        int _CS;
};

#endif // SENSORS_H
