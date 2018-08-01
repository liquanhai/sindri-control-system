#include <SINDRI_sensors.h>
#include <SINDRI_actuators.h>
#include <elapsedMillis.h>

// Code to test the reaction control 

// SPT25 Read 
// H2 Read
// Xylem Pump PWM Control

// Pins
int PTPIN = 0;
int TCPIN1 = 10;
int TCPIN2 = 20;
int TCPIN3 = 30;
int PWM_PIN = 3;
int H2PIN = 1;

//Pump definition
float satFlow = 2;
Pump m(PWM_PIN,satFlow);

//Thermocouple definition
Thermocouple thermocouple(TCPIN1,TCPIN2,TCPIN3);

//Pressure sensor definition
Pressure_sensor psensor(PTPIN);

//H2 sensor definition
H2_sensor h2sensor(H2PIN);

// Constants
float nominal = 0; // Calibrated pressure reading
float minMotorSpeed = 1;
float desiredFlowRate = 0.24;
bool filtering = 0;

// Variables for loop control
elapsedMillis loopTime; // loopTime used to force precise deltaT between starts
elapsedMillis sinceStart; // used to track time since start 
elapsedMillis onOffTracker; //On off delay to approximate slow motor speeds
uint32_t loopCounter;
uint32_t numSkip = 20;  // Number of loops to skip between host updates.
bool controlSwitch = false; // switch to start controls
float timeDelay = 150000; //time to start controlling
float startingRun = 10000; //time pump runs during warmup
float flowRateStarting = 1.0; //starting flowrate
float onDelay = 500; //on off delay time
float offDelay = 1500;

// PID Loop
float P_desired = 8;
float K_p = 0.15;
float K_d = 350;
float K_i = 0;
float Sum;
float summax = 100;

#define pastSize 3 //Past errors to keep 

float pastError[pastSize+1];  // Should be larger array than past_size.
unsigned int deltaT = 100;    // Sample period in microseconds.
float oneOverMdt = 1.0/(deltaT*pastSize); // Divide deltas by interval.

// Variables for warm up
float pastTemperature; //saves past temperature
float dTTreshold; //threshold for temperature stability


void setup() {
  // Start Serial
  Serial.begin(9600);
  Serial.println("Reaction Control Test!");

  // Test Pump 
//  Serial.println("Testing Pump... ");
//  m.motor_write(0.5);
////  delay(1000);
//  m.motor_write(0);
//  delay(10000);
  m.motor_write(0);
  // Calibrate the sensor
  Serial.println("Calibrating... ");

  for(int i = 0; i < 100; i++)
  { 
    psensor.update_sensor(filtering);
    nominal += psensor.current_reading;
    delay(20);
  }
  nominal /= 100.0;
  sinceStart = 0;
  printOut(100,"Nominal pressure correction: ",nominal);
  Serial.println("Done calibrating");
}

void loop() {
  //updatePump();
  //warmUp();
  minSignal();
}

//Pump update
void updatePump(){
  if (not controlSwitch) {return;}
  if (int(loopTime) < int(deltaT)) {return;}// looptime is an elapsedtime var.

  // Read Sensor value
  psensor.update_sensor(filtering);
  h2sensor.update_sensor();

  printOut(loopCounter,"H2 Reading: ",h2sensor.current_reading);
  
  float P_current = psensor.current_reading-nominal;

  printOut(loopCounter,"Pressure Reading: ",P_current);
  
  float error = P_desired-P_current;

  printOut(loopCounter,"Pressure Error: ",error);
  
  float errorDelta = (error-pastError[pastSize-1])*oneOverMdt;

  Sum = fmax(fmin(error*deltaT + Sum,summax),-1*summax);
  
  // Update previous errors for next time.
  for (int i = pastSize; i > 0; i--) pastError[i] = pastError[i-1];
  pastError[0] = error;

  printOut(loopCounter,"Pressure Error Change: ",errorDelta);
  
  // Set the motor speed
  float flowRate = desiredFlowRate + K_p*error+K_d*errorDelta+K_i*Sum;

  setMotorSpeed(flowRate); 
 
  printOut(loopCounter,"Motor Speed Written: ",flowRate);

  // updates timing
  loopTime = 0;
  if (loopCounter >= numSkip) {
    loopCounter = 0;
  } else {
    loopCounter += 1;
  }
}

// Runs warm up controls

void warmUp(){
  if (controlSwitch) {return;}
  if (int(loopTime) < int(deltaT)) {return;}// looptime is an elapsedtime var.
  loopTime = 0;

  thermocouple.update_sensor();
  
  float T_current = thermocouple.current_reading;
  
  printOut(loopCounter,"Current Temperature: ",T_current);

  float flowRate = flowRateStarting;

  if (sinceStart > startingRun) flowRate = 0;
  
  setMotorSpeed(flowRate); 
  printOut(loopCounter,"Motor Speed Written: ",flowRate);
  printOut(loopCounter,"Time since start: ",sinceStart);
  
  psensor.update_sensor(filtering);
  float currentPressure = psensor.current_reading-nominal;
  printOut(loopCounter,"Pressure: ",currentPressure);

  if (currentPressure > P_desired) controlSwitch = true; 
  
  // updates timing
  loopTime = 0;
  if (loopCounter >= numSkip) {
    loopCounter = 0;
  } else {
    loopCounter += 1;
  }
}

//prints an ouput
void printOut(int counter,String title,float value){
  if (counter >= numSkip) {
    Serial.print(title);
    Serial.print(value);
    Serial.print(" \n");
  }
}

float speedSet;
float newSwitch = 1;

void minSignal(){
  setMotorSpeed(speedSet);
  printOut(1000,"Motor Speed Written: ",speedSet);

  psensor.update_sensor(filtering);
  float currentPressure = psensor.current_reading-nominal;
  printOut(1000,"Pressure: ",currentPressure);

  if (speedSet >= 1.5) newSwitch = -1;
  if (speedSet <= 0.8) newSwitch = 1;
  speedSet += newSwitch*0.1;
  delay(1000);
}

void setMotorSpeed(float motorSpeed){
  float flowRate = 0;
  if (motorSpeed >= minMotorSpeed) {
    flowRate = motorSpeed;
  }
  else {
    flowRate = motorSpeed*(onDelay+offDelay)/onDelay;
    if (onOffTracker > onDelay) flowRate = 0;
  }

  if (onOffTracker > offDelay+onDelay) onOffTracker = 0;
  
  m.motor_write(flowRate);
}

