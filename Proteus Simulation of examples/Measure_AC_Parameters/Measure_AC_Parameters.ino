/*
 * Power Analyzer v-1
 * Developed By: Shuvangkar Chandra Das
 * Published by: www.eeetechbd.com
 * Youtube channel: www.youtube.com/c/shuvangkarshuvo
 * Website: www.eeetechbd.com
 * Linkedin Id: https://www.linkedin.com/in/shuvangkar/
 * For more info visit my website and youtube channel
 */
#include "PowerAnalyzer.h"
#define VOLT_PIN 0                     //Voltage signal is connected with Anaog pin 0
#define CURRENT_PIN 1                  //current Signal ic connected with Analog Pin 1
Measure measure(VOLT_PIN,CURRENT_PIN); //Creating the object
void setup() 
{
   Serial.begin(9600);
}

void loop()
{
  float Volt = measure.voltage();
  float Current = measure.current();
  float Power = measure.power();
  float PF = measure.PowerFactor();
  float Angle = measure.PhaseAngle();

  Serial.print(Volt);Serial.print("    ");
  Serial.print(Current);Serial.print("    ");
  Serial.print(Power);Serial.print("    ");
  Serial.print(PF,4);Serial.print("    "); //PF wil be shown till 4 decimal point
  Serial.println(Angle);
  
}
