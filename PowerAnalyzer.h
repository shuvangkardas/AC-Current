/*
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the GNU General Public License.
  * The software is published by www.eeetechbd.com.
  * The library is devloped by Shuvangkar Chandra Das(Shuvangkar Shuvo) 
  * founder of eeeTechBd
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
  */
#ifndef POWERANALYZER_h
#define POWERANALYZER_h
#include "Arduino.h"
#define N 200

class Measure
{
 
  //Library interface description 
  public:
    Measure(int VoltPin,int CurrentPin);
    float RMS(int channel);
    float power();
    float voltage();
    float current();
    float PowerFactor();
    void all(float &voltage, float &current,float &power, float &PF);
    float PhaseAngle();
    void ADC_Initialize();
    unsigned int ADC_Read(uint8_t channel);
    
   private:
      int _VoltPin;
      int _CurrentPin; 
};

#endif

