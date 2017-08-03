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
float V,I,P,pf;
void setup() 
{
  Serial.begin(9600); 
}

void loop()
{
  measure.all(V,I,P,pf);     //This function calculate all parameters within 44ms
  Serial.print(V);Serial.print("    ");
  Serial.print(I);Serial.print("    ");
  Serial.print(P);Serial.print("    ");
  Serial.print(pf,4);Serial.print("    ");//Print power factor in four decimal precision
  float angle = acos(pf);    //Calculating Phase Angle in Radian
  angle  = angle*(180.0/3.1416); //Converting Radian into degree
  Serial.println(angle,4);
}
