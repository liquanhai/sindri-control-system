// Code to collect data from Sparkfun MQ-8 Hydrogen gas sensor
// Can detect hydrogen gas concentrations anywhere from 100-10000ppm


// Input:  5.0V
// Output: 2.5-4.0V (in 1000ppm H2)


int sensorVal;

void setup() {
    Serial.begin(9600);
    Serial.println("H2 Sensor Test!");
}

void loop() {

    sensorValue = analogRead(0); // Analog Pin 0
    Serial.println(SensorValue, DEC);
    delay(100);
}

