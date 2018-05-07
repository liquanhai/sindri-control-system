#include "SINDRI_sensors.h"

// Pressure Sensor
Pressure_sensor::Pressure_sensor(int pin) : _buffer()
{
    _pin = pin;
}

void Pressure_sensor::begin()
{
    time = millis();

    Serial.print("Pressure sensor at pin: ");
    Serial.print(_pin);
    Serial.print(" Initialized!\n");
}

void Pressure_sensor::update_sensor(bool filter)
{
    float R = 267.0; // Ohms

    float val = analogRead(_pin);
    float V = val*(5.0/1024.0); // map the voltage between 0 and 5V
    float I = V/R; // Convert to current using the loop resistance

    current_reading =  0.0625*I*1e5 - 0.2;
    
    if(_buffer.size() < 5){
        // Add the new value to the buffer
        _buffer.push_front(current_reading);
    
    } else {
        // Removes oldest value, adds newest to beginning
        _buffer.pop_back();
        _buffer.push_front(current_reading);
    }

    if(filter){
        float total = 0;
        for(int i = 0; i < _buffer.size(); i++){
            total += _buffer[i];
        }

        float ave = total/(float)_buffer.size();
        current_reading = ave;
    }

    time = millis();
}

void Pressure_sensor::log_data()
{
    Serial.print(_pin);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");
}

// H2 Sensor
H2_sensor::H2_sensor(int pin)
{
    _pin = pin;
}

void H2_sensor::begin()
{
    time = millis();

    Serial.print("H2 sensor at pin: ");
    Serial.print(_pin);
    Serial.print(" Initialized!\n");
}

void H2_sensor::update_sensor()
{
    current_reading = analogRead(_pin); 
    time = millis();
}

void H2_sensor::log_data()
{
    Serial.print(_pin);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");
}

// Humidity Sensor
// Setup:
// Pin 1 to 5V
// Pin 2 to _pin
// Pin 4 to GND
// 10K between 2 and 1
Humidity_sensor::Humidity_sensor(int pin):
    dht_sensor(pin, DHT22)
{
    _pin = pin;    
}

void Humidity_sensor::begin()
{
    // Initialize wrapper class
    dht_sensor.begin();
    
    time = millis();

    Serial.print("Humidity sensor at pin: ");
    Serial.print(_pin);
    Serial.print(" Initialized!\n");
}

void Humidity_sensor::update_sensor()
{
    delay(2000); // Takes time to measure new reading
    float h = dht_sensor.readHumidity();
    if (!isnan(h))
    {
        current_reading = h;       
    }
    time = millis();
}

void Humidity_sensor::log_data()
{
    Serial.print(_pin);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");
}

// O2 Sensor
O2_sensor::O2_sensor(int pin)
{
    _pin = pin;
}

void O2_sensor::begin()
{
    time = millis(); 

    Serial.print("O2 Sensor at pin: ");
    Serial.print(_pin);
    Serial.print(" Initialized!\n");
}

void O2_sensor::update_sensor()
{
    float val = analogRead(_pin);
    float voltage = ((val/1024.0)*5.0)/201.0*10000;
    current_reading = voltage/7.43;
    time = millis();
}

void O2_sensor::log_data()
{
    Serial.print(_pin);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");

}

// Thermistors
Thermistor::Thermistor(int pin, float R)
{
    _pin = pin;
    _R   = R; 
}

void Thermistor::begin()
{
    time = millis();
    
    Serial.print("Thermistor at pin: ");
    Serial.print(_pin);
    Serial.print(" Initialized!\n");
}

void Thermistor::update_sensor()
{
    float analog_in  = analogRead(_pin);
    float measured_v = analog_in*(5.0/1023.0);

    float current_R = _R*((5.0-measured_v)/measured_v);

    float inv_T = _A + _B*log(current_R) + _C*pow(log(current_R),3);
    current_reading = 1/inv_T;
    
    time = millis();
}

void Thermistor::log_data()
{
    Serial.print(_pin);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");
}

// Thermocouples
Thermocouple::Thermocouple(int DO, int CLK, int CS):
    thermocouple(CLK, CS, DO)
{
    _CS = CS;
}

void Thermocouple::begin()
{
    time = millis();

    Serial.print("Thermocouple at pin: ");
    Serial.print(_CS);
    Serial.print(" Initialized!\n");
}

void Thermocouple::update_sensor()
{
    current_reading = thermocouple.readCelsius();
    time = millis();
}

void Thermocouple::log_data()
{
    Serial.print(_CS);
    Serial.print(",");
    Serial.print(current_reading);
    Serial.print(",");
    Serial.print(time);
    Serial.print("\n");
}

