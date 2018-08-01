// Code to read from SPT25-20-0100A Pressure Sensor

// Specs: Input:  9-36VDC
//        Output: 0-20mA


// Set up as a simple analogRead operation

int pressureSensor = A0;


void setup() {
    Serial.begin(9600);
    Serial.println("Pressure Sensor Test!");
}


void loop() {

    int val = analogRead(pressureSensor);
    Serial.println("Pressure Sensor Value: %i", val)
}

