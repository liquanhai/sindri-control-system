#include <SINDRI_actuators.h>
#include <SINDRI_sensors.h>

Pressure_sensor psensor(A10);
Pump pmp(9,2.0);


float nominal =0;
bool filtering = 1;
float satFlow = 2.0;
void motor_write(int pin, float flowrate)
{
  float clipped = constrain(flowrate, 0, satFlow);
//  Serial.println(clipped);
  float max_pwm = 255*(satFlow/3.8);
  float pwm_sig = max_pwm - (max_pwm/satFlow)*(satFlow-clipped);
//  Serial.println(pwm_sig);
//  Serial.println((int)pwm_sig);
  analogWrite(pin,(int)pwm_sig);
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Test");

  Serial.println("Calibrating");

//  delay(5000);

  for(int i = 0; i < 100; i++)
  {
    psensor.update_sensor(0);
    nominal += psensor.current_reading;
    delay(20);
  }

  nominal /= 100.0;
  Serial.println("Done Calibrating");
  Serial.println("Nominal: ");
  Serial.println(nominal);
  analogWrite(3,0);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  psensor.update_sensor(filtering);
//
  Serial.println(psensor.current_reading - nominal);
  analogWrite(3,0);
  Serial.println("Wrote Solenoid ON");
  delay(1000);
  analogWrite(3,255);
  delay(1000);
}
