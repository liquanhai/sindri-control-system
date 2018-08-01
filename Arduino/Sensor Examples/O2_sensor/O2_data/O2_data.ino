#include <math.h>

const int buzzerPin = 3; // Connect Buzzer module to Digital 3
float WarningValue = 19.5; // The minimum state of O2 concentration

void setup()
{
  Serial.begin(9600); // Start Serial Connection
}

void loop() 
{
  // long unsigned a;
  float sensorValue;
  float sensorVoltage;
  float Value_O2;

  sensorValue = analogRead(A0); // A0 should be the white output of the O2 sensor

  sensorVoltage = ((sensorValue/1024)*5.0)/201*10000;

  Value_O2 = sensorVoltage/7.43;

  Serial.print("Concentration of O2 is ");
  Serial.print(Value_O2, 1);
  Serial.print("\n");

  // Set off buzzer if concentration crosses threshold
  if (Value_O2 <= WarningValue)
  {
    digitalWrite(3, HIGH); 
  }
  else digitalWrite(3,LOW);
  
  delay(1000);
}

