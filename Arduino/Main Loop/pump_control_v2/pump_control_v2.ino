#include <SINDRI_sensors.h>
#include <SINDRI_actuators.h>
#include <elapsedMillis.h>

// Code to test the reaction control 

// SPT25 Read 
// H2 Read
// Xylem Pump PWM Control

// Pins
int PTPIN = 0;
int TCPIN1 = 1;
int TCPIN2 = 2;
int TCPIN3 = 3;
int PWM_PIN = 3;
int H2PIN = 10;
int slnd_pin = 4;


//Pump definition
float satFlow = 2;
Pump m(PWM_PIN,satFlow);

//Thermocouple definition
Thermistor t1(TCPIN1,2200);
Thermistor t2(TCPIN2,2200);
Thermistor t3(TCPIN3,2200);

//Pressure sensor definition
Pressure_sensor psensor(PTPIN);

//H2 sensor definition
H2_sensor h2sensor(H2PIN);

// Constants
float nominal = 0; // Calibrated pressure reading
float minMotorSpeed = 1;
float desiredFlowRate = 1.5;
bool filtering = 1;

// Variables for loop control
elapsedMillis loopTime; // loopTime used to force precise deltaT between starts
elapsedMillis sinceStart; // used to track time since start 
elapsedMillis onOffTracker; //On off delay to approximate slow motor speeds
uint32_t loopCounter;
uint32_t numSkip = 20;  // Number of loops to skip between host updates.
bool controlSwitch = false; // switch to start controls
float timeDelay = 70000; //time to start controlling
float startingRun = 4000; //time pump runs during warmup
float flowRateStarting = 1.0; //starting flowrate
float onDelay = 500; //on off delay time
float offDelay = 2500;

// PID Loop
float P_desired = 8;
float K_p = 0;
float K_d = 300;
float K_i = 0.00002;
float K_ms = 0;
float Sum;
float summax = 50000;
float MS_Sum;
float MS_summax = 2500;

#define pastSize 10 //Past errors to keep 

float pastError[pastSize+1];  // Should be larger array than past_size.
unsigned int deltaT = 100;    // Sample period in microseconds.
float oneOverMdt = 1.0/(deltaT*pastSize); // Divide deltas by interval.
float pastSpeed = 0;

// Variables for warm up
float pastTemperature; //saves past temperature
float dTTreshold; //threshold for temperature stability

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
  // Set solenoid pin to output
  pinMode(slnd_pin,OUTPUT);
  // Start Serial
  Serial.begin(9600);
  Serial.println("Reaction Control Test!");

  // Test Pump 
//  Serial.println("Testing Pump... ");
//  m.motor_write(0.5);
////  delay(1000);
//  m.motor_write(0);
//  delay(10000);
  motor_write(9,0);

  for(int i = 0; i < 5; i++)
  { 
    Serial.print("Starting in: ");
    Serial.println(5-i);
    delay(1000);
  }
  
  // Calibrate the sensor
  Serial.println("Calibrating... ");

  for(int i = 0; i < 100; i++)
  { 
    psensor.update_sensor(0);
    nominal += psensor.current_reading;
    delay(20);
  }
  nominal /= 100.0;
  //nominal = 29.68;
  printOut(100,"Nominal pressure correction: ",nominal);
  Serial.println("Done calibrating");
  sinceStart = 0;

  // Open the relief valve
  digitalWrite(slnd_pin, LOW);
}

void loop() {
  updatePump();
  warmUp();
  //minSignal();
  //H2Test();

  // Check if pressure bad
  if(psensor.current_reading - nominal > 30)
  {
    Serial.println("Pressure too high, opening valve");
    controlSwitch = false;
    digitalWrite(slnd_pin, HIGH);
  }
}

//Pump update
void updatePump(){
  if (not controlSwitch) {return;}
  if (int(loopTime) < int(deltaT)) {return;}// looptime is an elapsedtime var.

  // Read Sensor value
  psensor.update_sensor(filtering);
  h2sensor.update_sensor();
  t1.update_sensor();
  t2.update_sensor();
  t3.update_sensor();
  printOut(loopCounter,"Time Since Start: ",sinceStart);
  printOut(loopCounter,"H2 Reading: ",h2sensor.current_reading);
  printOut(loopCounter,"T1 Reading: ",t1.current_reading);
  printOut(loopCounter,"T2 Reading: ",t2.current_reading);
  printOut(loopCounter,"T3 Reading: ",t3.current_reading);
  
  float P_current = psensor.current_reading-nominal;

  printOut(loopCounter,"Pressure Reading: ",P_current);
  
  float error = P_desired-P_current;

  printOut(loopCounter,"MW from Pressure Error: ",error*K_p);
  
  float errorDelta = (error-pastError[pastSize-1])*oneOverMdt;

  Sum = fmax(fmin(error*deltaT + Sum,summax),-1*summax);
  
  // Update previous errors for next time.
  for (int i = pastSize; i > 0; i--) pastError[i] = pastError[i-1];
  pastError[0] = error;

  printOut(loopCounter,"MW from Pressure Error Change: ",errorDelta*K_d);
  printOut(loopCounter,"MW from Pressure Error Sum: ",Sum*K_i);
 
  // Set the motor speed

  MS_Sum = MS_Sum*0.95;
  
  MS_Sum = fmax(fmin(pastSpeed*deltaT + MS_Sum,MS_summax),0);

  float flowRate = desiredFlowRate + K_p*error+K_d*errorDelta+K_i*Sum-K_ms*MS_Sum;

  pastSpeed = flowRate;

  printOut(loopCounter,"MW from Motor Speed Sum: ",-1*MS_Sum*K_ms);
  
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

  t1.update_sensor();
  t2.update_sensor();
  t3.update_sensor();
  
  
  float T_current = t1.current_reading;
  
  printOut(loopCounter,"Current Chamber Temperature: ",T_current);

  printOut(loopCounter,"Temp 2: ", t2.current_reading);
  printOut(loopCounter,"Temp 3: ", t3.current_reading);
  

  float flowRate = flowRateStarting;

  if (sinceStart > startingRun) flowRate = 0;
  
  setMotorSpeed(flowRate); 
  printOut(loopCounter,"Motor Speed Written: ",flowRate);
  printOut(loopCounter,"Time since start: ",sinceStart);
  
  psensor.update_sensor(filtering);
  float currentPressure = psensor.current_reading-nominal;
  printOut(loopCounter,"Pressure: ",currentPressure);

  if (currentPressure > P_desired) controlSwitch = true; 
  if (sinceStart > timeDelay) controlSwitch = true; 
  
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

void H2Test(){
  h2sensor.update_sensor();
  printOut(100000,"H2 Reading: ",h2sensor.current_reading);
  
  delay(500);
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
  
  motor_write(PWM_PIN,flowRate);
}

