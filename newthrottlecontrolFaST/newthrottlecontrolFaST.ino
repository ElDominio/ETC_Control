#include <EnableInterrupt.h>

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
void idleRequestPWM();
void serialRead(byte);

// Inputs
const int POT_THROTTLE = A0; // Servo Position Input
const int POT_THROTTLE2 = A1; // Servo Position Input2
const int POT_PEDAL = A3; // Pedal Sensor Input
const int POT_PEDAL2 = A2; // Pedal Sensor Input
const int POT_IDLE = A4; //Requested Idle Input
const byte idlePWMPin = 2;

// Pins H bridge
const byte HB_DIRECTION = 8; // H bridge Direction
const byte HB_PWM = 11;     // H bridge PWM (speed)

// Throttle constraints, max voltages in both direction
 int MinTPS;
 int MinTPS2;
 int MaxTPS;
 int MaxTPS2;
 int MaxAPP = 1023;
 int MaxAPP2 = 1023;
 int MinAPP = 0;
 int MinAPP2 = 0;
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
byte idleSetPoint = 0;
unsigned long idlePWMtime = 0;
bool idleCount = true;
byte dutyCyclein = 0;
byte throttleMode = 0;

//Define the PID controller
double Input, Output, SetPoint;
PID throttlePID(&Input, &Output, &SetPoint, kP, kI, kD, DIRECT);
//double idleOutput, idleSetPoint;
//PID idlePID(&Input, &idleOutput,  = true&idleSetPoint, idlekP, idlekI, idlekD, DIRECT);

//Operating States flag 
char state = 's'; //s = start, t = transient, i = idle, c = constant (steady state)

void setup() {
  // put your setup code here, to run once:
 //Enable PID control
 Serial.begin(115200);
 Serial.println("Box OK!");
  throttlePID.SetMode(AUTOMATIC);
  throttlePID.SetSampleTime(1);
  //idlePID.SetMode(AUTOMATIC);
  calFlag = EEPROM.read(8);
  if (calFlag){
    loadCalibration();
    Serial.println("Calibration Loaded");
  }
  pinMode(HB_DIRECTION, OUTPUT);
  enableInterrupt(idlePWMPin, idleRequestPWM, CHANGE);
  
}

void loop() {
  if (Serial.available()){
    byte serialRead = Serial.read() - 48;
    Serial.println(serialRead);
    serialCommands(serialRead);
  }
  
  //Read all inputs
    int inTPS = analogRead(POT_THROTTLE); //read for example 48, meaning throttle at rest, max is 758
    int rawAPP = analogRead(POT_PEDAL); //read for example 70, app is 0 (no demand), max is 940
    int inAPP = map(rawAPP,MinAPP, MaxAPP, MinTPS, MaxTPS);
    
    switch(throttleMode){
      case 0:
       break;
      case 1:
       if (inAPP < 300){
        map(inAPP, MinAPP, 300, MinAPP, 250);
       }
       else if( (inAPP > 300) && (inAPP < 600)){
        map(inAPP, 300, 600, 250, 700);
       }
       else{
        map(inAPP, 600, MaxAPP, 700, MaxAPP);
       }
       break;
    }

    if (abs(inTPS - inAPP) < 70){
      state = 'c'; //Sets to steady state
    }
    else if (abs(inTPS - inAPP) >= 70){
      //Serial.println(inTPS -inAPP);
      state = 't';  //sets to transient mode
    }
    if ((rawAPP <= MinAPP + 3)/* && (inTPS < MinTPS + 60 )*/){
      state = 'i';  //sets idle mode
      idleSetPoint = 100 + dutyCyclein; //analogRead(POT_IDLE);
    }


    
     if ((millis() - timeDiag) > 500){
      int inTPS2 = analogRead(POT_THROTTLE2);
      int rawAPP2 = analogRead(POT_PEDAL2);
      if (diagEnabled){
        /*Serial.print("TPS2 % = "); Serial.println(inTPS2);
        Serial.print("APP % = "); Serial.println(rawAPP);
        Serial.print("APP2% = "); Serial.println(rawAPP2);
        Serial.print("MaxAPP % = "); Serial.println(MaxAPP);
        Serial.print("MinAPP % = "); Serial.println(MinAPP);
        Serial.print("MaxTPS % = "); Serial.println(MaxTPS);
        Serial.print("MinTPS % = "); Serial.println(MinTPS);
        Serial.print("MaxAPP % = "); Serial.println(MaxAPP);
        Serial.print("MinAPP % = "); Serial.println(MinAPP);*/

        Serial.print("TPS % = "); Serial.println(inTPS);
        Serial.print("Calibrated APP% = "); Serial.println(inAPP);
        Serial.print("State: "); Serial.println(state);
        Serial.print("Idle% = "); Serial.println(idleSetPoint);
        Serial.print("Input idle DC = ");Serial.println(dutyCyclein);
        Serial.print("PID Output = "); Serial.println(Output);
      }
      //Serial.print("Idle% = "); Serial.println(idleSetPoint);
      timeDiag = millis();
    }
    
   /* if (inAPP > MaxAPP -15){
      state = 'c'; //WOT mode
    }*/
    //convert inputs to doubles
    Input = inTPS;
    SetPoint = inAPP;
    switch (state){
      case 'c':
      throttlePID.SetTunings(1.1, .07, 0.03);
      //  throttlePID.SetSampleTime(2);
      if (inTPS < inAPP){
          digitalWrite(HB_DIRECTION, 0);            //sets ETC Direction to forward
          throttlePID.SetControllerDirection(DIRECT);
          throttlePID.SetOutputLimits(30,200);
          //Serial.print("PID Output = "); Serial.println(Output);
          throttlePID.Compute();                    //compute PID based correction
          analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction factor with PID
          //Serial.println("Steady State Forward");
        }
      else {
           digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
           /*throttlePID.SetTunings(1.4, 1, 0.7);
           throttlePID.SetControllerDirection(REVERSE);
           throttlePID.SetOutputLimits(20,120);
           throttlePID.Compute();   */
           if (abs(inTPS - inAPP) < 5){
            analogWrite(HB_PWM,20);  //makes throttle go backward with open loop values
            //Serial.println("Idle backslow");
           }
           else{
            analogWrite(HB_PWM,80);  //makes throttle go backward with open loop values
            //Serial.println("Idle back");
           }
           throttlePID.Compute();                    //compute PID based correction

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
             if ((inTPS < throttleRest -50)) {
              analogWrite(HB_PWM, 20);
             //Serial.println("Closing transient - slow");
             }
             else{
               analogWrite(HB_PWM, 170);
               //Serial.println("Closing transient");
             }    
        }
        //Serial.println("Transient mode");
        break;
        case 'i':
           // throttlePID.SetSampleTime(1);
          SetPoint = idleSetPoint;
          
          if (inTPS < idleSetPoint){
            throttlePID.SetTunings(1.1, .07, 0.0);
            throttlePID.SetControllerDirection(DIRECT);
            digitalWrite(HB_DIRECTION, 0);                //sets ETC Direction to forward
            throttlePID.SetOutputLimits(5,60);
            throttlePID.Compute();                            //compute PID based correction
            analogWrite(HB_PWM,(Output));  //uses the base duty and adds a correction facttor with PID
            //Serial.println("Idle opening");
          }
          else {
           digitalWrite(HB_DIRECTION, 1);                       //set ETC Direction Backward
           /*throttlePID.SetTunings(1.4, 1, 0.7);
           throttlePID.SetControllerDirection(REVERSE);
           throttlePID.SetOutputLimits(20,120);
           throttlePID.Compute();   */
           if (abs(inTPS - idleSetPoint) < 5){
            analogWrite(HB_PWM,60);  //makes throttle go backward with open loop values
            //Serial.println("Idle backslow");
           }
           else{
            analogWrite(HB_PWM,105);  //makes throttle go backward with open loop values
            //Serial.println("Idle back");
           }
           //Serial.println("Idle closing");
         //  Serial.println(Output);
         throttlePID.Compute();
          }
        //Serial.println("Idle control");
        break;
        default: //Assumes an error in the char assignation and closes the throttle
          digitalWrite(HB_DIRECTION, 1);
          analogWrite(HB_PWM,0);
         break;
    }
}

void serialCommands(byte serialRead){
  switch(serialRead){
      case 1:
        //Calibrate TPS
        MaxTPS = FindMaxTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        MaxTPS2 = FindMaxTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE2);
        MinTPS = FindMinTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        MinTPS2 = FindMinTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE2);
        throttleRest = FindRestingTPS(HB_PWM, HB_DIRECTION, POT_THROTTLE);
        break;
      case 2:
      //CalibrateAPP
        MaxAPP = FindMaxAPP(POT_PEDAL);
        MaxAPP2 = FindMaxAPP(POT_PEDAL2);
        MinAPP = FindMinAPP(POT_PEDAL);
        MinAPP2 = FindMinAPP(POT_PEDAL2);
        break;
       case 3:
        diagEnabled = !diagEnabled;
        break;
       case 4:
        calFlag = burnCalibration(MinTPS, MaxTPS, MinAPP, MaxAPP, MinTPS2, MaxTPS2, MinAPP2, MaxAPP2,throttleRest);
        break;
       case 5:
        calFlag = clearCalibration();
        break;
       case 6:
        throttleMode++;
        if (throttleMode > 1){ throttleMode = 0;}
        Serial.print("Throttlemode changed to = ");Serial.println(throttleMode);
        break;
      default:
        break;
    }
}

void loadCalibration(){
  byte high = EEPROM.read(0);
  byte low = EEPROM.read(1);
  MinTPS = word(high,low);
  high = EEPROM.read(2);
  low = EEPROM.read(3);
  MaxTPS = word(high,low);
  high = EEPROM.read(4);
  low = EEPROM.read(5);
  MinAPP = word(high,low);
  high = EEPROM.read(6);
  low = EEPROM.read(7);
  MaxAPP = word(high,low);
  high = EEPROM.read(9);
  low = EEPROM.read(10);
  throttleRest = word(high,low);
  calFlag = EEPROM.read(8);
  
  high = EEPROM.read(11);
  low = EEPROM.read(12);
  MinTPS2 = word(high,low);
  high = EEPROM.read(13);
  low = EEPROM.read(14);
  MaxTPS2 = word(high,low);
  high = EEPROM.read(15);
  low = EEPROM.read(16);
  MinAPP2 = word(high,low);
  high = EEPROM.read(17);
  low = EEPROM.read(18);
  MaxAPP2 = word(high,low);
}

void idleRequestPWM(){
  //Serial.println("INTERRUPT! ");
  if (digitalRead(idlePWMPin)){
    if (idleCount){
      idlePWMtime = millis();
      idleCount = false;
    }
  }
  else{
    if (!idleCount){
      dutyCyclein = millis() - idlePWMtime;
      idleCount = true;
    }
  }
}
