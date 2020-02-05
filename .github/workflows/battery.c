////////////////////////////////////////
// Module: battery.c
//
////////////////////////////////////////

#include "datatype.h"

#define CHARGER_CONNECT_STATUS_D7   PIN_D7

uint8 AlertLowBattery;
uint8 LowBatteryLedState;
uint8 BatteryTooLow;
uint8 ChargerConnected;
int16 v;

////////////////////////////////////////
uint16 AdditionlValueInCharge = 50;
uint16 BatteryVoltagereadingCapacityTable[2][21] = 
{
   { 0,   5,   10,  15,  20,  25,  30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95,  100 },
   { 849, 858, 880, 896, 906, 913, 915, 918, 912, 917, 919, 926, 928, 933, 938, 958, 858, 858, 965, 965, 965 }
};

////////////////////////////////////////
void ConstructBatteryModule() 
{
   setup_adc(ADC_CLOCK_INTERNAL);
   setup_adc_ports(sAN22, VSS_VDD);
   set_adc_channel(22);
}

////////////////////////////////////////
void BatteryManagement() 
{
   static int16 lastVbat;
//   static uint32 ChargerConnectedTime = 0;
//   uint32 timeDiff = 0;
//   static uint8 LastChargerConnected = 0;
   
   v = read_adc();

   if (lastVbat != v) 
   {
      fprintf(STDOUT, "Battery reading = %ld", v);
      fprintf(STDOUT, new_line_resp);
      
      lastVbat = v;
      BatteryTooLow = FALSE;
      AlertLowBattery = FALSE;
   
      if (v < 767) 
      {
         AlertLowBattery = TRUE;
         LowBatteryLedState = FALSE;
      
         if (v < 500)
            BatteryTooLow = TRUE;
      }
   }
   
   ChargerConnected = input(CHARGER_CONNECT_STATUS_D7);
}

////////////////////////////////////////
uint8 BatteryCapacity()
{
   uint8 i;
   int16 chargerConnectDelta;
   uint8 retPercentage = 0;
   static int16 ValNoCharge = 0;
   
   ChargerConnected = input(CHARGER_CONNECT_STATUS_D7);
   
   if( !ChargerConnected )
      ValNoCharge = v;
      
   else 
      chargerConnectDelta = v - ( AdditionlValueInCharge * ChargerConnected );

   for( i = 20; i > 0; i-- )
      if(  chargerConnectDelta >= BatteryVoltagereadingCapacityTable[1][i] )
      {
         retPercentage = BatteryVoltagereadingCapacityTable[0][i+1];   
         break;
      }

   return retPercentage;
}

////////////////////////////////////////


























































