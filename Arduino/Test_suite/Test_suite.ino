#include <SINDRI_sensors.h>
#include <SINDRI_actuators.h>

// Define the sensor and actuator locations

int pressure_pin = 0;
int temp1_pin = 1;
int temp2_pin = 2;
int temp3_pin = 3;

int pump_pin = 3;
int slnd_pin = 4;

// Define sensors
Thermistor t1(temp1_pin,2200);
Thermistor t2(temp2_pin,2200);
Thermistor t3(temp3_pin,2200);

Pressure_sensor p1(pressure_pin);

void setup() {
  // Set solenoid pin to output
  pinMode(slnd_pin,OUTPUT);

  // Start serial
  Serial.begin(9600);
  Serial.println("Sensor and Actuator Test");
  t1.begin();
  t2.begin();
  t3.begin();

  p1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.print("\nTemp1: ");
  t1.update_sensor();
  t1.log_data();

  Serial.print("Temp2: ");
  t2.update_sensor();
  t2.log_data();

  Serial.print("Temp3: ");
  t3.update_sensor();
  t3.log_data();

  Serial.print("\nPres1: ");
  p1.update_sensor(0);
  p1.log_data();

  delay(1500);
  
//  Serial.print("Motor set full\n");
//  analogWrite(pump_pin, 255);
//  delay(2000);
//
//  Serial.print("Motor off\n");
//  analogWrite(pump_pin,0);

//  Serial.print("Solenoid On\n");
//  digitalWrite(slnd_pin, HIGH);
//
//  delay(4000);
//
//  Serial.print("Solenoid off\n");
//  digitalWrite(slnd_pin,LOW);
//
//  delay(4000);
   
}
