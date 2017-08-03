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

#include "PowerAnalyzer.h"
#include "HardwareSerial.h"
#include "Arduino.h"
#include "math.h"

//Measure measure;

Measure::Measure(int VoltPin,int CurrentPin)
{
    ADC_Initialize();
    _VoltPin = VoltPin;
   _CurrentPin = CurrentPin;
   //offset_calculation(VoltagePin);
}


/* In this section we will measure 3 basic parameters such as voltage, current, power*************/

float Measure::RMS(int channel)
{
 int adc_value[N] = {0},offset = 0;
 //long int first_millis = millis();
 unsigned long int offset_sum = 0;
  for(int i=0;i<N;i++)
  {
    adc_value[i]= ADC_Read(channel);
    offset_sum +=adc_value[i];
    
  }
  offset =(int)(offset_sum/N);
 //long int second_millis = millis();
 //Serial.print(second_millis-first_millis);Serial.print("    ");
  float rms=0;
  long int sq_sum = 0;
  //Serial.print(offset);Serial.print("   ");
  for(int i=0;i<N;i++)
  {
    adc_value[i] = adc_value[i]-offset;
    //sq_sum = sq_sum+pow(adc_value[i],2.00); 
    sq_sum = sq_sum+(long int)adc_value[i]*(long int)adc_value[i];
  }
  sq_sum = sq_sum/N;
  rms = sqrt(sq_sum);
  rms = ((5.00*rms)/1023.00);
  //Serial.print(Irms);Serial.print("    ");
  //long int thirdmillis = millis();
  //Serial.print(thirdmillis-second_millis);Serial.print("    ");
  return rms;
}

float Measure::voltage()
{
  float value = RMS(_VoltPin);
  return value;
  
}
float Measure::current()
{
  float value = RMS(_CurrentPin);
  return value;
}
float Measure::PowerFactor()
{
  float V,I,P,pf;
  all(V,I,P,pf);
  return pf;
  
}

float Measure::PhaseAngle()
{
  float V,I,P,pf;
  all(V,I,P,pf);
  float angle  = acos(pf)*(180.0/3.1416);
  return angle; 
}

float Measure::power()
{
  int volt_adc[N] = {0},current_adc[N] = {0}, volt_offset = 0,current_offset = 0;
  long int first_millis = millis();
  unsigned long int volt_offset_sum = 0,current_offset_sum = 0;
  for(int i=0;i<N;i++)
  {
    volt_adc[i]= ADC_Read(_VoltPin);
    //volt_adc[i]= ADC_Read(1);
    current_adc[i] = ADC_Read(_CurrentPin);
    //current_adc[i] = ADC_Read(2);
    volt_offset_sum +=volt_adc[i];
    current_offset_sum +=current_adc[i];
  }
  volt_offset =(int)(volt_offset_sum/N);
  current_offset =(int)(current_offset_sum/N);
  //Serial.print(current_offset);Serial.print("   ");
  //Serial.print(volt_offset);Serial.print("  ");
  
  long int second_millis = millis();
  //Serial.print(second_millis-first_millis);Serial.print("    ");
  float Power=0;
  long int mul_sum = 0;
  //Serial.print(offset);Serial.print("   ");
  for(int i=0;i<N;i++)
  {
    volt_adc[i] = volt_adc[i]-volt_offset;
    current_adc[i] = current_adc[i] - current_offset;
    //sq_sum = sq_sum+pow(adc_value[i],2.00); 
    mul_sum = mul_sum+((long int)volt_adc[i]*(long int)current_adc[i]);
  }
  mul_sum = mul_sum/N;
  Power = ((5.00*mul_sum)/1023.00);
  Power = ((5.00*Power)/1023.00);
  //long int thirdmillis = millis();
  //Serial.print(thirdmillis-second_millis);Serial.print("    ");
  return Power;
  
}

void Measure::all(float &voltage, float &current,float &power, float &PF)
{
  int volt_adc[N] = {0},current_adc[N] = {0}, volt_offset = 0,current_offset = 0;
  //long int first_millis = millis();
  unsigned long int volt_offset_sum = 0,current_offset_sum=0;
  for(int i=0;i<N;i++)
  {
    volt_adc[i]= ADC_Read(_VoltPin);
    current_adc[i] = ADC_Read(_CurrentPin);
    volt_offset_sum +=volt_adc[i];
    current_offset_sum +=current_adc[i];
    
  }
  volt_offset =(int)(volt_offset_sum/N);
  current_offset =(int)(current_offset_sum/N);
  //long int second_millis = millis();
  //Serial.print(second_millis-first_millis);Serial.print("    ");
  float real_power=0,Vrms = 0,Irms=0,pf = 0;
  static float parameter[4] = {0};
  long int mul_sum = 0,volt_sq_sum = 0, current_sq_sum = 0;
  //Serial.print(offset);Serial.print("   ");
  for(int i=0;i<N;i++)
  {
    volt_adc[i] = volt_adc[i]-volt_offset;
    current_adc[i] = current_adc[i] - current_offset;
    volt_sq_sum = volt_sq_sum + ((long int)volt_adc[i]*(long int)volt_adc[i]);
    current_sq_sum = current_sq_sum+ ((long int)current_adc[i]*(long int)current_adc[i]);
    mul_sum = mul_sum+((long int)volt_adc[i]*(long int)current_adc[i]);
  }
  //Real power
  mul_sum = mul_sum/N;
  real_power = ((5.00*mul_sum)/1023.00);
  real_power = ((5.00*real_power)/1023.00);
 // parameter[2] = real_power;
  
  //Serial.print(real_power);Serial.print("    ");
  //Vrms Calculation
  volt_sq_sum = volt_sq_sum/N;
  Vrms = sqrt(volt_sq_sum);
  Vrms = ((5.00*Vrms)/1023.00);
  //parameter[0]= Vrms;
  
  //Serial.print(Vrms);Serial.print("    ");
  //Irms Calculation
  current_sq_sum = current_sq_sum/N;
  Irms = sqrt(current_sq_sum);
  Irms = ((5.00*Irms)/1023.00);
  //parameter[1] = Irms;

  //Serial.print(Irms);Serial.print("    ");
  //PF Calculation
  pf = real_power/(Vrms*Irms);
  //parameter[3] = pf;
  
  //long int thirdmillis = millis();
  //Serial.print(thirdmillis-second_millis);Serial.print("    ");
  //return parameter;
  voltage = Vrms;
  current = Irms;
  power = real_power;
  PF = pf;

}
/* End of Measuring Current Voltage power*****************************************************/



void Measure::ADC_Initialize()
{
/*
ADPS2 ADPS1 ADPS0 Division Factor
  0     0     0       2
  0     0     1       2
  0     1     0       4
  0     1     1       8
  1     0     0       16
  1     0     1       32
  1     1     0       64
  1     1     1       128
 */
  ADCSRA |= 1<<ADEN; //ADC enable. ADC doesn't consume power when ADEN is cleared
  //ADCSRA |=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // prescaler=128 means X_cpu/128=7.8125KHz
  ADCSRA |=(1<<ADPS2)|(1<<ADPS1); //
  // ADCSRA |= 1<<ADPS2;
  ADMUX |= 1<<REFS0; //Reference: AVCC (AVCC must be connected with VCC)
  
}
unsigned int Measure::ADC_Read(uint8_t channel)
{
  int adc_value = 0;
  //ADMUX |= (ADMUX&0b11100000)|(channel&0b00011111); // First five bits of ADMUX for selecting channel
  ADMUX = (ADMUX&0xf0)|(channel&0x0f);
  ADCSRA |=1<<ADSC; //Conversion will start writing this bit to one.
  while(!(ADCSRA&(1<<ADIF))); //ADIF will set when conversion complete and loop breaks.
  ADCSRA|= 1<<ADIF; //ADIF must be cleared(1) to trigger a new conversion next time
  adc_value = ADCW;
  return(adc_value);
  
}

