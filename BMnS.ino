// #include <AUnit.h>
// #include <AUnitVerbose.h>

#include <Wire.h>
#include "I2C_Anything.h"

#define DEBUG

#define Cell_Count 10

#if (Cell_Count == 12)
const uint8_t analogInput[Cell_Count] = { A0, A2, A4, A6, A8, A10, A12, A14, A1, A3, A13, A15 } ; //Analog Inputs
const int R1[12] = { 0, 680, 680, 680, 470, 470, 680, 680, 680, 680, 680, 680 }; //exact measurment of each resistor Resistor #1  
const int R2[12] = { 680, 680, 300, 220, 100, 100, 100, 100, 100, 68, 68, 68 }; //exact measurment of each resistor Resistor #2
#endif
#if (Cell_Count == 10)
const uint8_t analogInput[Cell_Count] = { A0, A2, A4, A6, A8, /*A10,*/ A12, A14, A1, A3, A13/*, A15*/ } ; //Analog Inputs
const int R1[Cell_Count] = { 0, 680, 680, 680, 470, /*470,*/ 680, 680, 680, 680, 680/*, 680*/ }; //exact measurment of each resistor Resistor #1  
const int R2[Cell_Count] = { 680, 680, 300, 220, 100, /*100,*/ 100, 100, 100, 68, 68/*, 68*/}; //exact measurment of each resistor Resistor #2
#endif

#define relay 2 //Pin for Relay/MOSFET

/* Scale Calculator */
// New Steps/mm = (Old/Current steps/mm) x [100 / (measured distance filament traveled)]
#define VScale1  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale2  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale3  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale4  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale5  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale6  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale7  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale8  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale9  1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale10 1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale11 1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 
#define VScale12 1/*ARDUINO_VOLTAGE/REAL_VOLTAGE*/ 

const float Scale[12] = {VScale1, VScale2, VScale3, VScale4, VScale5, VScale6, VScale7, VScale8, VScale9, VScale10, VScale11, VScale12}; 

float Cell_Voltage[Cell_Count] = {};

const byte SLAVE_ADDRESS = 8;


void setup(){
   for ( int i = 0; i < Cell_Count; i++ ){
      pinMode(analogInput[i], INPUT); 
   }
   pinMode(relay, OUTPUT);
   digitalWrite(relay, HIGH);

   Wire.begin(); //I2C Bus Begin

   #ifdef DEBUG
   Serial.begin(115200);
   #endif
}

void loop(){
   for ( int i = 0; i < Cell_Count; i++ ){
      int value = analogRead(analogInput[i]);
      analogToVoltage(value, i);
   }

   #ifdef DEBUG
      for ( int i = 0; i < Cell_Count; i++ ){
         Serial.println("Cell "+ String(i) + " = " + String(Cell_Voltage[i]));
      }
      Serial.println();
   #endif
   for (int i = 0; i < Cell_Count; i++)
   {  
   Wire.beginTransmission (SLAVE_ADDRESS);
   I2C_writeAnything (Cell_Voltage[i]);
   I2C_writeAnything(i);
   Wire.endTransmission ();
     
   delay (200);
   }  // end of for
   // delay(1000);
}

float analogToVoltage(int value, int cellIndex) {
   int i = cellIndex;

   float vout;
   float vin;

   float vin_c;

   vout = (value * 5.0) / 1024.0; // see text
   #ifdef DEBUG
      // Serial.println("VOUT " + String(vout));
   #endif

   vin = vout / ((R2[i] * 1.0) / (R1[i] + R2[i])); 
   //Serial.println(R1[i] + R2[i]);
   //Serial.println((R2[i]*1.0) / (R1[i] + R2[i]));

   #ifdef DEBUG
      // Serial.println("VIN " + String(vin));
   #endif
   Cell_Voltage[i] = vin * Scale[i];
   
   


   #ifdef DEBUG
      // Serial.print("INPUT V"+String(i+1)+"= ");
      // Serial.println(vin_c, 4);
      // Serial.println(value); //raw analog input 
   #endif

   return; //vin_c;
}