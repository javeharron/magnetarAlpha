//Copyright (c) 2023, John Simonis and The Ohio State University
//This code was written by John Simonis for the Magnetar research project at The Ohio State University.

#include <Wire.h> //Standard wire library for I2C. 
#include <TFLI2C.h> //This is a library for the TF-Luna lidar module which is used in this project. https://github.com/budryerson/TFLuna-I2C

TFLI2C LidarSensor; //This initializes the TF-Luna lidar sensor using the I2C Lidar library.

int16_t LidDist; //This variable is used to store the Lidar distance data.
int Voltage;
int CalibCnt = 0;

const int Capacitance = 1700; //Capacitance in uF.
const int Resistance = 20000; //Resistance in Ohms.
const int TargetEnergy = 1;
const int16_t I2CAddress = 0x10;

const byte OkayLight = 3; //Digital pin for the Okay/green light.
const byte BadLight = 4; //Digital pin for the Bad/red light.
const byte ChargePin = 5; //Digital pin for controlling the charge cycle of the capacitor.
const byte DischargePin = 6; //Digital pin for discharging the capacitor 
const byte FirePin = 7; //Digital pin for triggering the firing of the coil gun.
const byte TriggerPin = 8; //Digital pin for reading the trigger button value.
const byte CalibPin = 9; //Digital pin for calibrating coil gun.
const byte ReadPin = A0; //Analog pin for reading the voltage value on the capacitor bank.

float Dist,TOF,Velocity,VoltageNec; //Dynamic floating point numbers for distance, time of flight, and velocity.

const float DragCoef = 0.67; //Drag coefficient for air drag calculations.
const float Resolution = 0.001; //Resolution for air drag curve calculations.
const float Density = 1.225; //Density of air (SI units).
const float Area = 0.00011844; //Cross sectional area (m^2).
const float Mass = 0.005; //Mass in kg.
const float Efficiency = 0.01; //% Efficiency of coil. 

float InverseDrag(float Distance){ //This function inverts normal air drag to ensure that a target travels at an initial velocity that degrades into a specified target velocity.
  float Ctr = 0; //Ctr for inverse drag function.
  float Temp = 0; //Temp value for TOF.
  float Veloc = sqrt(TargetEnergy/(Mass*0.5)); //Calculates ideal velocity from target energy.
  while(Temp<Distance){ //Iteratively calculates the starting velocity necessary to get to the distance at target energy.
      Veloc += ((0.5 * Density * pow(Veloc,2) * Area * DragCoef) / Mass * Resolution);
      Ctr += Resolution;
      Temp = Veloc * Ctr;
  }
  return Veloc;
}

float Drag(float Veloc, float TOF){ //This is the normal air drag function included for debug purposes and ballistic table generation.
  for (float i = 0; i <= TOF; i += Resolution) {
      Veloc -= ((0.5 * Density * pow(Veloc,2) * Area * DragCoef) / Mass * Resolution);
  }
  return Veloc;
}

int ReadVoltage(){ //Due to the amount of times voltage needs to be read this function is necessary.
  digitalWrite(DischargePin, 0); //Most of these writes are purely sanity checks for safety.
  digitalWrite(ChargePin, 0);
  digitalWrite(OkayLight, 0);
  digitalWrite(FirePin, 0);
  return map(analogRead(A0), 0, 1023, 0, 380);
}

void setup() {
  Serial.begin(115200); //Opens a serial communication line over 115200 baud for communication and debugging.

  pinMode(OkayLight, OUTPUT);//Configures all pins to necessary specifications.
  pinMode(BadLight, OUTPUT);
  pinMode(ChargePin, OUTPUT);
  pinMode(DischargePin, OUTPUT);
  pinMode(FirePin, OUTPUT);
  pinMode(TriggerPin, INPUT_PULLUP);
  pinMode(CalibPin, INPUT_PULLUP);
  pinMode(ReadPin, INPUT);

  Voltage = ReadVoltage(); //Reads the voltage across the capacitor bank.

  while(Voltage > 0){ //Discharges the capacitors if there is any voltage before main loop can be run
    digitalWrite(BadLight, 1);
    digitalWrite(DischargePin, 1);
    delay(1000);
    digitalWrite(DischargePin, 0);
    Voltage = ReadVoltage();
  }
}

void loop() {
  if(!digitalRead(CalibPin) ==1){ //This is a seperate button for calibrating distance, because the lidar sensor is noisy this allows users to calibrate on a target at a specified distance for charging
    Voltage = ReadVoltage();
    LidarSensor.getData(LidDist,I2CAddress);
    Dist = LidDist/100; //Converts the distance from cm to m.
    Velocity = InverseDrag(Dist);
    CalibCnt++;
  }
  if(CalibCnt > 0 && !digitalRead(CalibPin) == 0){ //If the distance is calibrated we can start the charging process
    VoltageNec = sqrt((0.5*pow(Velocity,2)*Mass)/((Capacitance/1000000)*0.5*Efficiency)); //This is the amount of Voltage necessary to arrive at the target at the desired energy.
    Voltage = ReadVoltage(); //Measure the current voltage for sanity.
    while(Voltage >= VoltageNec + 2 && Voltage <= VoltageNec - 2){ //While we are outside the target voltage range charge the capacitors accordingly in 5ms pulses
      if(Voltage > VoltageNec + 2){
        digitalWrite(DischargePin,1);
        delay(5);
        Voltage = ReadVoltage();
      }
      else{
        digitalWrite(ChargePin,1);
        delay(5);
        Voltage = ReadVoltage();
      }
    }
    while(Voltage <= VoltageNec + 2 && Voltage >= VoltageNec - 2) //While we are in the target voltage range alert the user and prepare to fire and recharge if necessary.
      digitalWrite(OkayLight, 1);
      if(!digitalRead(TriggerPin) == 1){
        digitalWrite(FirePin,1);
        delay(5);
        digitalWrite(FirePin,0);
      }
      else{
        Voltage = ReadVoltage();
      }
  }
}
