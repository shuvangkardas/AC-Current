/* Power Analyzer v-1
 * Developed By: Shuvangkar Chandra Das
 * Published by: www.eeetechbd.com
 * Youtube channel: www.youtube.com/c/shuvangkarshuvo
 * Website: www.eeetechbd.com
 * Linkedin Id: https://www.linkedin.com/in/shuvangkar/
 * For more info visit my website and youtube channel
 */
 /********************************************************************
  * We know that arduino can measure only voltage in the range of 0-5V
  * So we have to convert the higher voltage(220) into lower voltage using 
  * voltage divider and Transformer.To get the actual voltage, after calculating analog 
  * value, we will multiply the the voltage with the transfer ratio of 
  * voltage divider or transformer.
  *********************************************************************/
  /*To calculate current a current transformer(CT) is used. Arduino cannot read current
   *directly from CT as it produces current. So a sensing resistor converter the current   
   *into voltage. 
   */
#include "PowerAnalyzer.h"
#define VOLT_PIN 0                     //Voltage signal is connected with Anaog pin 0
#define CURRENT_PIN 1                  //current Signal ic connected with Analog Pin 1
#define volt_transfer_ratio ((100.0+10.0)/10.0) // Vin/Vadc = R1/(R1+R2),Here R1 = 10k, R2 = 100k
#define current_transfer_ratio (1000.00/200.00) //CT ratio = 1000, Sensing Resistor = 200 Ohm
Measure measure(VOLT_PIN,CURRENT_PIN); //Creating the object
float V,I,P,pf;
void setup() 
{
 Serial.begin(9600); 
}

void loop()
{
  float volt = 0, current = 0, power = 0,power_factor = 0,real_power = 0;
  for(int i = 0;i<20;i++)                //Averaging the 20 values to get better result. 
  {
    measure.all(V,I,P,pf);
    volt  = volt+ V*volt_transfer_ratio; //Calculating Actual voltage and then summing to calculate average
    current =current+ I*current_transfer_ratio; //Calculating Actual Current and Summing 
    power = power+ abs(P*volt_transfer_ratio*current_transfer_ratio);
    power_factor = power_factor+abs(pf);
  }
  volt = volt/20.0;
  current = current/20.0;
  power = power/20.0;
  power_factor = power_factor/20.0;
  Serial.print(volt);Serial.print("    ");
  Serial.print(current);Serial.print("    ");
  Serial.print(power);Serial.print("    ");
  Serial.println(power_factor);
}
