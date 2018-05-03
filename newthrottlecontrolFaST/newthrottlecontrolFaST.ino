#include <PID_v1.h>
#include "ETC.h"
//AlphaX Throttle Control
//Licensed under the GPL v3
/*
Alpha X ThrttoleController
Copyright (C) Norman Paulino

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
void loadCalibration();

// Inputs
const int POT_THROTTLE = A0; // Servo Position Input
const int POT_THROTTLE2 = A1; // Servo Position Input2
const int POT_PEDAL = A2; // Pedal Sensor Input
const int POT_PEDAL2 = A3; // Pedal Sensor Input
const int POT_IDLE = A4; //Requested Idle Input

// Pins H bridge
const int HB_DIRECTION = 8; // H bridge Direction
const int HB_PWM = 11;     // H bridge PWM (speed)

// Throttle constraints, max voltages in both direction
 int MinTPSV;
 int MaxTPSV;
 int MaxAPP = 1023;
 int MinAPP = 0;
 unsigned int throttleRest;

//PID Control variables
double kP = 2.3;
double kI = 1;
double kD = .9;
//Idle constants
double idlekP = 1.8;
double idlekI = .08;
double idlekD = 0.9;

//customvars
unsigned long timeDiag = 500;
bool diagEnabled = false;
bool calFlag = 0;
int throttleCorr = 0;
int idleSetPoint = 0;

//Define the PID controller
double Input, Output, SetPoint;
PID throttlePID(&Input, &Output, &SetPoint, kP, kI, kD, DIRECT);
//double idleOutput, idleSetPoint;
//PID idlePID(&Input, &idleOutput, &idleSetPoint, idlekP, idlekI, idlekD, DIRECT);

//Operating States flag 
char state = 's'; //s = start, t = transient, i = idle, c = constant (steady state)

void setup() {
  // put your setup code here, to run once:
 //Enable PID control
 Serial.begin(115200);
 Serial.println("Box OK!");
  throttlePID.SetMode(AUTOMATIC);
  //idlePID.SetMode(AUTOMATIC);
  calFlag = EEPROM.read(8);
  if (calFlag){
    loadCalibration();
    Serial.println("Calibration Loaded");
  }
  pinMode(HB_DIRECTION, OUTPUT);
  
}

void loop() {
  if (Serial.available()){
    byte serialRead = Serial.read() - 48;
    Serial.println(serialRead);
    switch(serialRead){
      case 1:
        //Calibrate TPS
        MaxTPSV = FindMaxTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        MinTPSV = FindMinTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        throttleRest = FindRestingTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        break;
      case 2:
      //CalibrateAPP
        MaxAPP = FindMaxAPP(POT_PEDAL);
        MinAPP = FindMinAPP(POT_PEDAL);
        break;
       case 3:
        diagEnabled = !diagEnabled;
        break;
       case 4:
        calFlag = burnCalibration(MinTPSV, MaxTPSV, MinAPP, MaxAPP,throttleRest);
        break;
       case 5:
        calFlag = clearCalibration();
        break;
      default:
        break;
    }
  }
  
  //Read all inputs
    int inTPS = analogRead(POT_THROTTLE); //read for example 48, meaning throttle at rest, max is 758
    int inAPP = analogRead(POT_PEDAL); //read for example 70, app is 0 (no demand), max is 940
    inAPP = map(inAPP,MinAPP, MaxAPP, MinTPSV, MaxTPSV);
    idleSetPoint = MinAPP + 10; //analogRead(POT_IDLE);

   //Do all


   //Print states every 500ms
    if ((millis() - timeDiag) > 500){
      if (diagEnabled){
        Serial.print("TPS % = "); Serial.println(Input);
        //Serial.print("MaxTPS % = "); Serial.println(MaxTPSV);
        //Serial.print("MinTPS % = "); Serial.println(MinTPSV);
        Serial.print("APP% = "); Serial.println(SetPoint);
        //Serial.print("MaxAPP % = "); Serial.println(MaxAPP);
        //Serial.print("MinAPP % = "); Serial.println(MinAPP);
        Serial.print("State: "); Serial.println(state);
      }
      //Serial.print("Idle% = "); Serial.println(idleSetPoint);
      timeDiag = millis();
    }
    
    if (abs(inTPS - inAPP) < 70){
      state = 'c'; //Sets to steady state
    }
    else if (abs(inTPS - inAPP) >= 70){
      //Serial.println(inTPS -inAPP);
      state = 't';  //sets to transient mode
    }
    if ((inAPP <= idleSetPoint) && (inTPS < MinTPSV + 20 )){
      state = 'i';  //sets idle mode
    }
   /* if (inAPP > MaxAPP -15){
      state = 'c'; //WOT mode
    }*/
    //convert inputs to doubles
    Input = inTPS;
    SetPoint = inAPP;
    switch (state){
      case 'c':
      throttlePID.SetTunings(2, 1, 0.9);
      if (inTPS < inAPP){
          digitalWrite(HB_DIRECTION, 0);            //sets ETC Direction to forward
          throttlePID.SetOutputLimits(0,100);
          throttlePID.Compute();                    //compute PID based correction
          analogWrite(HB_PWM,(100 + Output));  //uses the base duty and adds a correction facttor with PID
          //Serial.println("Steady State Forward");
        }
      else {
         digitalWrite(HB_DIRECTION, 1);                   //set ETC Direction Backward
         analogWrite(HB_PWM,30);  //makes throttle go backward with open loop values
         //Serial.println("Steady State Reverse");
      }
      break;
      case 't':
      //Transient function that forces the TP to open at max rate, and close at a relatively fast rate to avoid slamming
      if (inTPS < inAPP){ //opening
            digitalWrite(HB_DIRECTION, 0);
            analogWrite(HB_PWM,200);
            //Serial.println("Open Transient");
          }
        else {
           digitalWrite(HB_DIRECTION, 1); //closing
             if ((inAPP < throttleRest-50)) {//(SetPoint < 3)){
              analogWrite(HB_PWM, 10);
             Serial.println("Closing transient - slow");
             }
             else{
             analogWrite(HB_PWM, 170);
             Serial.println("Closing transient");
             }    
        }
        //Serial.println("Transient mode");
        break;
        case 'i':
          throttlePID.SetTunings(1.8,0.1, 0.9);
          if (inTPS < idleSetPoint){
            digitalWrite(HB_DIRECTION, 0);                //sets ETC Direction to forward
            throttlePID.SetOutputLimits(0,50);
            throttlePID.Compute();                            //compute PID based correction
            analogWrite(HB_PWM,(50 + Output));  //uses the base duty and adds a correction facttor with PID
            //Serial.println("Idle opening");
          }
          else {
           digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
           //idlePID.Compute();   
           analogWrite(HB_PWM,(20 /*+ idleOutput*/));  //makes throttle go backward with open loop values
           //Serial.println("Idle closing");
          }
        //Serial.println("Idle control");
        break;
        default: //Assumes an error in the char assignation and closes the throttle
          digitalWrite(HB_DIRECTION, 1);
          analogWrite(HB_PWM,0);
         break;
    }
}

void loadCalibration(){
  byte high = EEPROM.read(0);
  byte low = EEPROM.read(1);
  MinTPSV = word(high,low);
  Serial.println(word(high,low));
  high = EEPROM.read(2);
  low = EEPROM.read(3);
  MaxTPSV = word(high,low);
  high = EEPROM.read(4);
  low = EEPROM.read(5);
  MinAPP = word(high,low);
  high = EEPROM.read(6);
  low = EEPROM.read(7);
  MaxAPP = word(high,low);
  calFlag = EEPROM.read(8);
  high = EEPROM.read(9);
  low = EEPROM.read(10);
  throttleRest = word(high,low);
}

