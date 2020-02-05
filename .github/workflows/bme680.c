/**
 * @brief       BME680.cpp
 * @details     Low power gas, pressure, temperature & humidity sensor.
 *              Function file.
 *
 *
 * @return      N/A
 *
 * @author      Manuel Caballero
 * @date        21/July/2018
 * @version     21/July/2018    The ORIGIN
 * @pre         This is just a port from Bosh driver to mBed ( c++ )
 * @warning     N/A
 * @pre         This code belongs to Nimbus Centre ( http://www.nimbus.cit.ie ).
 */
/**\mainpage
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File     bme680.c
 * @date    19 Jun 2018
 * @version 3.5.9
 *
 */
 
/*! @file bme680.c
 @brief Sensor driver for BME680 sensor */
#include "BME680.h"
#include "datatype.h"

 /*
BME680::BME680 ( PinName sda, PinName scl, uint32_t freq )
    : _i2c          ( sda, scl )
{
    _i2c.frequency  ( freq );
}
 
 
BME680::~BME680()
{
}
 */

extern uint16  breathVolume;
extern struct  testInfoPara    testStat;


float flowLPM = 0.0;

//const int dP = 0, FlowRate = 1;

float polynomialfit(int32_t dPressure)
{
   float flow;
   float deltaPressure = (float)dPressure;
   
   flow = (0.0000000001*(deltaPressure*deltaPressure*deltaPressure)) - (0.000001*(deltaPressure*deltaPressure)) + (0.0042*deltaPressure) + 1.7646;
//   flow = ((0.0000000002*(deltaPressure*deltaPressure*deltaPressure)) - (0.000002*(deltaPressure*deltaPressure)) + (0.0059*deltaPressure) + 0.0152);
//   flow = ((0.0000000002*(deltaPressure*deltaPressure*deltaPressure)) - (0.000002*(deltaPressure*deltaPressure)) + (0.0069*deltaPressure) + 0.0152);
//   fprintf(STDOUT, "********** yaman dP %lu, flowLPM1 = %.2f \r\n", dPressure,flow);
   return flow;
}

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
float GetNewVolume(uint32_t dPressure, uint32_t timeIntervalMs)
{
  float newVolume_ml = 0.0;
  
  flowLPM = polynomialfit(dPressure);
  newVolume_ml = (flowLPM*(float)timeIntervalMs)*(1.0/60.0);
  return newVolume_ml;
}

///////////////////////////////////////////////////////////////////////////////
int8_t Compute_Breath_Volume(struct bme680_dev *dev)
{
   uint32_t    timerCtr = 0;
   uint32_t    startPressure;                                     // = 34120;//34132+10;//99837+5;
   uint32_t    startCount = 5;
   uint32_t    lowPressureLastCount = 0;
   uint32_t    inVolumeAccumulation = 0;
   uint32_t    accumulatedTimeMs = 0;   
   uint32_t    pressure1 = 0;
   uint32_t    timeStamp1 = 0, deltaT = 0, deltaP = 0, prevdeltaP = 0;
   uint32_t    temperature_received,pressure_received, _Pressure,humidity_received,gas_received, _T1,_T2,_T3,_tfine,temperature_comp;
   uint32_t    temperatureXLSB, temperatureLSB, temperatureMSB,pressureXLSB, pressureLSB, pressureMSB;
   uint32_t    _Temperature,_P1,_P2,_P4,_P8,_P9,_P5,_P10;
   int32_t     _P3,_P6,_P7;       
   
   uint32_t    timer_current, timer_diff, timer_previous = 0;
   
   uint8_t     timerFlag = OFF;
   uint8       osrs_h, osrs_t, osrs_p, filter, value,control_reg;
   uint8       numberOfAttempts = 1;      
   uint8       temp, temp1, pressureChangeNotDetected= 0;      
   uint8       earlyWarningFlag = ON;
   
//   uint32_5    lowPressureLastCountLimit = 3;//6;
//   uint32_t    prevPressure1 = 0, prevPressure2 = 0;
//   uint32_t    pressure_comp,humidity_comp, gas_comp,gas_resistance_MSB, gas_resistance_LSB, humidityMSB, humidityLSB;   
//   uint8       ctr, initSensorCmds[4],*regPtr = NULL;  

   const uint32_t dPressureThresheldStart = 100;                //20;//20;//12;
   const uint32_t dPressureThresheldEnd = 100;                 //30;//6;
      
   static uint32_t prevTimeStamp1 = 0;
   
   float temperature;
   float AccumulatedVolume1_ml = 0.0;
   
   float deltaVol_1 = 0.0;
   float tmpVol = 0;
   
   //   float humi, AccumulatedVolume2_ml = 0.0;;
 
   enum  BreathTestState breathTestState = BREATH_TEST_NOT_RUN;
   
//   struct      bme680_field_data data[N_MEAS];
   uint8       status = MYSTATUS_OK;
   uint32_t    count;
   uint8 rslt = BME680_OK;
   
   temp = temp1 = 0;
   get_mem_page( dev );
   
#ifdef DEBUG   
   fprintf(STDOUT, "Current mem_page before writing control registers = %02X\r\n", dev->mem_page);
   fprintf(STDOUT, "***************************************************************************\r\n");
#endif   
   
   delay_ms(100);
   
   osrs_h = BME680_OS_NONE;                       //oversampling none
   osrs_t = BME680_OS_8X << 5;                    //oversampling x 8
   osrs_p = BME680_OS_4X << 2;                    //oversampling x 4
   control_reg = osrs_t | osrs_p;

#ifdef DEBUG
   fprintf(STDOUT, "**********control_reg = %02X\r\n", control_reg);
   
   //set oversampling
   fprintf(STDOUT, "*********************************************************************\r\n");
   fprintf(STDOUT, "********************    MODE   SELECT  ******************************\r\n");
   fprintf(STDOUT, "*********************************************************************\r\n");

   //Read values of oversampling register before writing
   temp = Read_Byte(BME680_SPI_CTRL);               //(0x74)
   temp = Read_Byte(BME680_SPI_CTRL_HUM);           //(0x72)
#endif   

   /********************/
   /* Set Oversampling */
   /********************/
   //Set oversampling values for temperature, pressure, and humidity
   Write_Byte( BME680_SPI_CTRL, control_reg ); //0x74
   Write_Byte( BME680_SPI_CTRL_HUM, osrs_h );  //0x72

#ifdef DEBUG   
   //Read values of oversampling register after writing
   temp = Read_Byte(0x74); //BME680_SPI_CTRL
   temp = Read_Byte(0x72); //BME680_SPI_CTRL_HUM
   
   //Read values of filter register before writing
   temp = Read_Byte(BME680_SPI_CONFIG); //0x75
#endif
   /********************/
   /* Set Filter       */
   /********************/
   filter = 0x3 << 2;
   //filter = 0x5 << 2;
   
   //Set filter values for temperature and pressure
   Write_Byte(BME680_SPI_CONFIG, filter ); //BME680_SPI_CONFIG

#ifdef DEBUG
   //Read values of filter register after writing
   Read_Byte(0x75); //BME680_SPI_CONFIG
#endif   
   
#if 0
/* Enable Gas Measurment and Heater Set-Point Index */
   run_gas = 0x1 << 4;
   nb_conv = 0x1;
   ctrl_gas = run_gas | nb_conv;
   
   //Read values of run_gas register before writing
   Read_Byte(BME680_SPI_CTRL_GAS_1);
   
   //Set the enable bit for gas measurements
   Write_Byte(BME680_SPI_CTRL_GAS_1, ctrl_gas );  

   //Read values of run_gas register after writing
   Read_Byte(BME680_SPI_CTRL_GAS_1);
   
/* Set heat-on time */

   //Read values of gas wait register before writing
   Read_Byte(BME680_SPI_GAS_WAIT);
   
   //Set the time to wait for heating
   Write_Byte(BME680_SPI_GAS_WAIT, 0x59 );  

   //Read values of gas_wait register after writing
   Read_Byte(BME680_SPI_GAS_WAIT);
   
/* Set hot plate temperature setting res_heat_x */
/* first compute the value of the destination register */
   par_g1 = 0xED;
   par_g2 = 0xEB;
   par_g3 = 0xEE;
   res
   var1 = (((int32_t)amb_temp * par_g3) / 10) << 8;
   var2 = (par_g1 + 784) * (((((par_g2 + 154009) * target_temp * 5) / 100) + 3276800) / 10);
   var3 = var1 + (var2 >> 1);
   var4 = (var3 / (res_heat_range + 4));
   var5 = (131 * res_heat_val) + 65536;
   res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
   res_heat_x = (uint8_t)((res_heat_x100 + 50) / 100);
   
   //Read values of res_heat register before writing
   Read_Byte(BME680_SPI_RES_HEAT_X);
   
   //Set the target temperature for the hot plate
   Write_Byte(BME680_SPI_RES_HEAT_X, 0x59 );  

   //Read values of res_heat register after writing
   Read_Byte(BME680_SPI_RES_HEAT_X);
  
#endif   

   /* Fset mode to force mode*/
   //Read values of ctrl_meas register before writing
   value = Read_Byte(BME680_SPI_CTRL); //0x74
   
   //Set the mode to force mode
   Write_Byte(BME680_SPI_CTRL, value + 0x1 ); //0x74  

#ifdef DEBUG
   //Read values of ctrl_meas register after writing
   temp = Read_Byte(0x74); //BME680_SPI_CTRL
   
   //retreive raw data output
   fprintf(STDOUT, "***************************************************************************\r\n");
   fprintf(STDOUT, "***************    Capture Calibration Coefficants  ***********************\r\n");
   fprintf(STDOUT, "***************************************************************************\r\n");
#endif

   if( dev->intf == BME680_SPI_INTF )
   {
      rslt = set_mem_page( 0x50, dev );
      
      if( rslt != BME680_OK ) 
         fprintf(STDOUT, " Failed set mem page xxxx \r\n\r\n");
   }
   
   /**************************************/
   /* Set up Calib Temperature data t1-t3*/
   /**************************************/
   _T1 = ( Read_Byte(0xEA) << 8 );
   _T1 |= Read_Byte(0xE9);
   dev->calib.par_t1 = _T1;

   _T2 = ( Read_Byte(0x8B) << 8 );
   _T2 |= Read_Byte(0x8A);
   dev->calib.par_t2 = _T2;
      
   _T3 = Read_Byte(0x8C);
   dev->calib.par_t3 = _T3;
   
   /************************************/
   /* Set up Calib Pressure data p1-p10*/
   /************************************/
      
   _P1 = ( Read_Byte(0x8F) << 8 );
   _P1 |= Read_Byte(0x8E);
   //_P1 = 0xFFFF0000 | _P1;
   dev->calib.par_p1 = _P1;

   _P2 = ( Read_Byte(0x91) << 8 );
   _P2 |= Read_Byte(0x90);
   _P2 = 0xFFFF0000 | _P2;
   dev->calib.par_p2 = _P2;
   
   _P3 = Read_Byte(0x92);
   dev->calib.par_p3 = _P3;

   _P4 = ( Read_Byte(0x95) << 8 );
   _P4 |= Read_Byte(0x94);
   //_P4 = 0xFFFF0000 | _P4;
   dev->calib.par_p4 = _P4;

   _P5 = ( Read_Byte(0x97) << 8 );
   _P5 |=  Read_Byte(0x96);
   _P5 = 0xFFFF0000 | _P5;
   dev->calib.par_p5 = _P5;
   
   _P6 = Read_Byte(0x99);
   dev->calib.par_p6 = _P6;

   
   _P7 = Read_Byte(0x98);
   dev->calib.par_p7 = _P7;

   _P8 = ( Read_Byte(0x9D) << 8 );
   _P8 |= Read_Byte(0x9C);
   _P8 = 0xFFFF0000 | _P8;
   dev->calib.par_p8 = _P8;

   _P9 = ( Read_Byte(0x9F) << 8 );
   _P9 |= Read_Byte(0x9E);
   _P9 = 0xFFFF0000 | _P9;
   dev->calib.par_p9 = _P9;

   _P10 = Read_Byte(0xA0);
   _P10 = 0xFFFF0000 | _P10;
   dev->calib.par_p10 = _P10;

#ifdef DEBUG
   fprintf(STDOUT, "**********   _T1 = %Lu\r\n\r\n", _T1);
   fprintf(STDOUT, "**********   _T2 = %Lu\r\n\r\n", _T2);
   fprintf(STDOUT, "**********   _T3 = %Lu\r\n\r\n", _T3);
   fprintf(STDOUT, "**********   _P1 = %Lu\r\n\r\n", _P1);
   fprintf(STDOUT, "**********   _P2 = %Lu\r\n\r\n", _P2);
   fprintf(STDOUT, "**********   _P3 = %Lu\r\n\r\n", _P3);
   fprintf(STDOUT, "**********   _P4 = %Lu\r\n\r\n", _P4);
   fprintf(STDOUT, "**********   _P5 = %Lu\r\n\r\n", _P5);
   fprintf(STDOUT, "**********   _P6 = %Lu\r\n\r\n", _P6);
   fprintf(STDOUT, "**********   _P7 = %Lu\r\n\r\n", _P7);
   fprintf(STDOUT, "**********   _P8 = %Lu\r\n\r\n", _P8);
   fprintf(STDOUT, "**********   _P9 = %Lu\r\n\r\n", _P9);
   fprintf(STDOUT, "**********   _P10 = %Lu\r\n\r\n", _P10);
#endif


#ifdef HUMUDITY_ON
   /***********************************/
   /* Set up Calib Humidity data h1-h7*/
   /***********************************/
   if( getHumidity )
   {
      temp16 =  ( Read_Byte(0xE3) << 8 );
      temp16 |=  Read_Byte(0xE2);
      temp16 = ( temp16 | temp1 ) >> 4;
      
      dev->calib.par_h1 = temp16;
      dev->calib.par_h1 = 638;

      temp16 =  ( Read_Byte(0xE1) << 8);
      temp16 |=  Read_Byte(0xE2);
      temp16 = ( temp16 | temp1 ) >> 4;
      dev->calib.par_h2 = temp16;
      dev->calib.par_h2 = 1036;
   
      temp = Read_Byte(0xE4);
      dev->calib.par_h3 = temp;
      dev->calib.par_h3 = 0;
      
      temp = Read_Byte(0xE5);
      dev->calib.par_h4 = temp;
      dev->calib.par_h4 = 45;      
      
      temp = Read_Byte(0xE6);
      dev->calib.par_h5 = temp;
      dev->calib.par_h5 = 20;      
      
      temp = Read_Byte(0xE7);
      dev->calib.par_h6 = temp;
      dev->calib.par_h6 = 120;
      
      temp = Read_Byte(0xE8);
      dev->calib.par_h7 = temp; 
      dev->calib.par_h7 = -100;
   }
      
 #endif
 
 #ifdef GAS_ON
       /******************************/
       /* Set up Calib Gas data g1-g3*/
       /******************************/
       
//      temp = Read_Byte(0xED);
//      dev->calib.par_gh1 = temp;
        
//      temp16 = ( Read_Byte(0xEC) << 8 );
//      temp16 |= Read_Byte(0xEB);
//      dev->calib.par_gh2 = temp16;

//      temp = Read_Byte(0xEE);
//      dev->calib.par_gh3 = temp;
#endif


   rslt = set_mem_page( 0x1, dev );
      
   if( rslt != BME680_OK ) 
      fprintf(STDOUT, " Failed set mem page \r\n");
 
   //Read values of ctrl_meas register after writing
   //Read values of ctrl_meas register before writing
   delay_ms(100);

#ifdef DEBUG
   value = Read_Byte(BME680_SPI_CTRL);

   //retrieve raw data output
   fprintf(STDOUT, "***************************************************************************\r\n");
   fprintf(STDOUT, "********************    Raw Data Output      ******************************\r\n");
   fprintf(STDOUT, "***************************************************************************\r\n");
#endif   
   
   count = 0;
   timer_previous = get_ticks();

   while(TRUE)
   {
      //Set the mode to force mode
      Write_Byte(BME680_SPI_CTRL, value+ 0x1 );  //0x74

#ifdef DEBUG
      //Read values of ctrl_meas register after writing
      temp = Read_Byte(BME680_SPI_CTRL); //0x74
#endif

      //********************************************************************************//
      //********************************************************************************//
      //***************************    Raw Data Output   *******************************//
      //********************************************************************************//

      //**************************************//
      //           Read temperature           //
      //**************************************//
      temperatureXLSB = Read_Byte(0x24); //BME680_SPI_TEMP_XLSB
      temperatureLSB = Read_Byte(0x23);  //BME680_SPI_TEMP_LSB
      temperatureMSB = Read_Byte(0x22);  //BME680_SPI_TEMP_MSB
      temperature_received = ( (temperatureMSB << 12 ) | (temperatureLSB << 4 ) | (temperatureXLSB >> 4 ) );
   
      //**************************************//
      //           Read Pressure              //
      //**************************************//
      pressureXLSB = Read_Byte(0x21); //BME680_SPI_PRESS_XLSB
      pressureLSB = Read_Byte(0x20);  //BME680_SPI_PRESS_LSB
      pressureMSB = Read_Byte(0x1f);  //BME680_SPI_PRESS_MSB
      timeStamp1 = get_ticks();
      pressure_received = ( (pressureMSB << 12 ) | (pressureLSB << 4 ) | (pressureXLSB >> 4 ) );

   
#ifdef HUMIDITY_AND_GAS_RESISTANCE   
      //**************************************//
      //          Read Humidity               //
      //**************************************//
      humidityLSB = Read_Byte(BME680_SPI_HUM_LSB);
      humidityMSB = Read_Byte(BME680_SPI_HUM_MSB);

      fprintf(STDOUT, "**********   Humidity MSB_LSB = %02X %02X \r\n", humidityMSB, humidityLSB);
      humidity_received = ( (humidityMSB << 8 ) | humidityLSB );
      fprintf(STDOUT, "**********   humidity_receive %08LX \r\n", humidity_received);
   
      humidity_comp = calc_humidity( humidity_received, dev);
      fprintf(STDOUT, "**********   Humidity %Lu %% \r\n\r\n", humidity_comp/1000);
      
      //**************************************//
      //          Read Gas resistance         //
      //**************************************//
      gas_resistance_LSB = Read_Byte(BME680_SPI_GAS_R_LSB);
      gas_resistance_MSB = Read_Byte(BME680_SPI_GAS_R_MSB);

      fprintf(STDOUT, "**********   Gas MSB_LSB = %02X %02X \r\n", gas_resistance_MSB, gas_resistance_LSB);
      gas_received = ( (gas_resistance_MSB  << 8 ) | gas_resistance_LSB );
      fprintf(STDOUT, "**********   Gas_receive %08LX \r\n", gas_received);

      gas_range = ( Read_Byte( BME680_FIELD0_ADDR + 14 ) & BME680_GAS_RANGE_MSK );  
      gas_comp = calc_gas_resistance( gas_received, gas_range, dev);
      fprintf(STDOUT, "**********   Gas resistance %Lu Ohm \r\n\r\n", gas_comp);
#endif
      //*************************************//
      // Calculate temperature adjustment    //
      //*************************************//
      int32_t var1;
      int32_t var2;
      int32_t var3;
//      int16_t calc_temp;
      
      var1 = ((int32_t)temperature_received>>3)-((int32_t)_T1<<1);
      var2 = (var1*(int32_t)_T2)>>11;
      var3 = ((var1>>1)*(var1>>1))>>12;
      var3 = ((var3)*((int32_t)_T3<<4))>>14;
      _tfine  = (int32_t)(var2+var3);
      _Temperature = (int16_t)(((_tfine*5)+128)>>8);
      
      
#ifdef DEBUG     
      //fprintf(STDOUT, "**********   var1 = %Lu\r\n\r\n", var1);
      //fprintf(STDOUT, "**********   var2 = %Lu\r\n\r\n", var2);
      //fprintf(STDOUT, "**********   var3 = %Lu\r\n\r\n", var3);
      //fprintf(STDOUT, "**********   t_fine = %Lu\r\n\r\n", _tfine);
      fprintf(STDOUT, "*   Temperature %.2f Celsius \r\n\r\n", (float)_Temperature / 100);
#endif

      //Verify temperature calculation matches BME library calculation
  
      temperature_comp = calc_temperature(temperature_received, dev);
      temperature = (float)temperature_comp / 100.0;
      
      /*fprintf(STDOUT, "**********   BME Library Temperature %.2f Celsius \r\n\r\n", (float)temperature_comp / 100);*/
       
      //*******************************//
      // Now compute the pressure      //
      //*******************************//
      int32_t var11;
      int32_t var22;
      int32_t var33;
      int32_t var4;
      int32_t _Pressure;

////////////////////////////////////////////////////////////////////////////
//    Equation 1 : var11  = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
////////////////////////////////////////////////////////////////////////////
      var11  = (((int32_t)_tfine) >> 1) - 64000;
      //fprintf(STDOUT, "**********   Equation 1 var1 = %Lu\r\n\r\n", var11);

////////////////////////////////////////////////////////////////////////////
//    Equation 2 :  var22  = ((((var11 >> 2)*(var11 >> 2)) >> 11)*(int32_t)_P6) >> 2;
////////////////////////////////////////////////////////////////////////////

      var22 = ((var11 >> 2));
      if (var11 < 0)
      {
         var22 = var22 + 0xc0000000;
      }      
      //fprintf(STDOUT, "**********   Equation 2 var2 = %Lu\r\n\r\n", var22);
      var22 = var22 * var22;
      
      //fprintf(STDOUT, "**********   Equation 2 var2 = %Lu\r\n\r\n", var22);      
      
      if (var22 < 0)
      {
         var22 = var22 >> 11;
         var22 = var22 + 0xffe00000;
      }
      else
      {
      var22 = var22 >> 11;
      }
      //fprintf(STDOUT, "**********   Equation 2 var2 = %Lu\r\n\r\n", var22);
      
      var22 = var22*(int32_t)_P6;
      //fprintf(STDOUT, "**********   Equation 2 var2 = %Lu\r\n\r\n", var22);
      
      var22 = var22 >> 2;
      //fprintf(STDOUT, "**********   Equation 2 final output = %Lu\r\n\r\n", var22);
      
/////////////////////////////////////////////////////////////////////////////
//    Equation 3 : var22  = var22 + ((var11 * (int32_t)_P5) << 1);
/////////////////////////////////////////////////////////////////////////////
      
      var22  = var22 + ((var11 * (int32_t)_P5) << 1);
      //fprintf(STDOUT, "**********   Equation 3 final output = %Lu\r\n\r\n", var22);

/////////////////////////////////////////////////////////////////////////////
//    Equation 4 : var22  = (var22 >> 2) + ((int32_t)_P4 << 16);
/////////////////////////////////////////////////////////////////////////////
      if (var22 < 0)
      {
         var22  = (var22 >> 2);
         var22 = var22 + 0xc0000000;
      }
      
      else
      {
         var22  = (var22 >> 2);
      }
      
      var22 = var22 + ((int32_t)_P4 << 16);
      //fprintf(STDOUT, "**********   Equation 4 final output = %Lu\r\n\r\n", var22);
      
/////////////////////////////////////////////////////////////////////////////
//    Equation 5 : var11  = (((((var11>>2)*(var11>>2))>>13)*((int32_t)dev->calib.par_p3<<5))>>3)+(((int32_t)dev->calib.par_p2*var11)>>1)
/////////////////////////////////////////////////////////////////////////////
      int32_t var1_2 = var11 >> 2;
      
      if (var11 < 0)
      {
         var1_2 = var1_2 + 0xc0000000;
      }
      
      int32_t var1_13 = ((var1_2)*(var1_2)) >> 13;
      //fprintf(STDOUT, "**********   Equation 5 var1_13 = %Lu\r\n\r\n", var1_13);
      
      var1_13 = (var1_13)*((int32_t)_P3 << 5);
      //fprintf(STDOUT, "**********   Equation 5 var1_13 = %Lu\r\n\r\n", var1_13);
      
      if (var1_13 < 0)
      {
         var1_13 = var1_13 >> 3;
         //fprintf(STDOUT, "**********   Equation 5 var1_13 = %Lu\r\n\r\n", var1_13);
         var1_13 = var1_13 + 0xE0000000;
      }
      
      else
      {
         var1_13 = var1_13 >> 3;
         //fprintf(STDOUT, "**********   Equation 5 var1_13 = %Lu\r\n\r\n", var1_13);
      }
      
      int32_t var1_1 = ((int32_t)_P2 * var11);
      
      if (var1_1 < 0)
      {
         var1_1 = var1_1 >> 1;
         var1_1 = var1_1 + 0x80000000;
      }
      
      else
      {
         var1_1 = var1_1 >> 1;
      }
      
      //fprintf(STDOUT, "**********   Equation 5 var1_1 = %Lu\r\n\r\n", var1_1);
      var11 = var1_13 + var1_1;
      //fprintf(STDOUT, "**********   Equation 5 final output = %Lu\r\n\r\n", var11);
      
      
/////////////////////////////////////////////////////////////////////////////////////////
//    Equation 6 : var11 = var11 >> 18;
/////////////////////////////////////////////////////////////////////////////////////////
      if (var11 < 0)
      {
         var11  = var11 >> 18;
         var11 = var11 + 0xffffc000;
      }
      
      else
      {
         var11  = var11 >> 18;
      }
      //fprintf(STDOUT, "**********   Equation 6 final output = %Lu\r\n\r\n", var11);
      
/////////////////////////////////////////////////////////////////////////////////////////
//    Equation 7 : var11  = ((32768 + var11) * (int32_t)_P1) >> 15;
/////////////////////////////////////////////////////////////////////////////////////////
      var11  = ((32768 + var11) * (int32_t)_P1) >> 15;
      //fprintf(STDOUT, "**********   Equation 7 final output = %Lu\r\n\r\n", var11);

/////////////////////////////////////////////////////////////////////////////////////////
//    Equation 8 : _Pressure = 1048576 - (uint32_t)pressure_received;
/////////////////////////////////////////////////////////////////////////////////////////
      _Pressure = 1048576 - pressure_received;
      //_Pressure = 1048576 - _pressure_received;
      //fprintf(STDOUT, "**********   Equation 8  final output = %Lu\r\n\r\n", _Pressure);
      
/////////////////////////////////////////////////////////////////////////////////////////
//     Equation 9 : _Pressure = (int32_t)((_Pressure - (var22 >> 12)) * ((uint32_t)3125));
/////////////////////////////////////////////////////////////////////////////////////////
      _Pressure = (int32_t)((_Pressure - (var22 >> 12)) * ((uint32_t)3125));
      //fprintf(STDOUT, "**********   Equation 9 final output = %Lu\r\n\r\n", _Pressure);

/////////////////////////////////////////////////////////////////////////////////////////
//    Equation 10 : Check pressure maximum condition
/////////////////////////////////////////////////////////////////////////////////////////
      if (_Pressure < 0)
      {
         uint32_t pressure = _Pressure;
         uint32_t _var11 = var11;
         _Pressure = ((pressure/_var11) << 1);
         //fprintf(STDOUT, "**********   Equation 10 Pressure = %Lu\r\n\r\n", _Pressure);
      }
     
      else
      {
         //fprintf(STDOUT, "**********  Equation 10 va11 = %Lu\r\n\r\n", (uint32_t)var11);
         _Pressure = ((_Pressure << 1));
         //fprintf(STDOUT, "**********   Equation 10 Pressure = %Lu\r\n\r\n", _Pressure);
      }
      
      uint32_t pressure = _Pressure;
      uint32_t _var11 = var11;
      _Pressure = pressure/_var11;
      
      //fprintf(STDOUT, "**********   Equation 10 final output = %Lu\r\n\r\n", _Pressure);
      //_Pressure = 3593256250/35863;
      /*fprintf(STDOUT, "**********   Look Here Pressure = %Lu\r\n\r\n", _Pressure);*/
       
//////////////////////////////////////////////////////////////////////////////////////////
//     Equation 11 : var11 = ((int32_t)_P9*(int32_t)(((_Pressure>>3)*(_Pressure>>3))>>13))>>12;
//////////////////////////////////////////////////////////////////////////////////////////
      var11 = (_Pressure>>3)*(_Pressure>>3);
      
      //fprintf(STDOUT, "**********   Equation 11 var11 = %Lu\r\n\r\n", var11);

      if (var11 < 0)
      {
         var11 = (int32_t)( var11 >> 13);
         var11 = var11 + 0xfff8000;
      }
      
      else
      {
         var11 = (int32_t)( var11 >> 13);
      }
      
      //fprintf(STDOUT, "**********   Equation 11 var11 = %Lu\r\n\r\n", var11);
      
      var11 = (int32_t)_P9*(int32_t)var11;
      //fprintf(STDOUT, "**********   Equation 11 var11 = %Lu\r\n\r\n", var11);
       
      if (var11 < 0)
      {
         var11 = var11 >> 12;
         var11 = var11 + 0xFFF00000;
      }
      
      else
      {
         var11 = var11 >> 12;
      }
      
      //fprintf(STDOUT, "**********   Equation 11 final output = %Lu\r\n\r\n", var11);
       
/////////////////////////////////////////////////////////////////////////////////////////
//    Equation 12 : var22 = ((int32_t)(_Pressure >> 2) * (int32_t)dev->calib.par_p8) >> 13;
/////////////////////////////////////////////////////////////////////////////////////////
       
      var22 = ((int32_t)(_Pressure >> 2) *(int32_t)_P8);
      
      //fprintf(STDOUT, "**********   Equation 12 var22 = %Lu\r\n\r\n", var22);
      if (var22 < 0)
      {
         var22 = var22 >> 13;
         var22 = var22 + 0xfff80000;
      }
      
      else
      {
         var22 = var22 >> 13;
      }
      
      //fprintf(STDOUT, "**********   Equation 12 final output = %Lu\r\n\r\n", var22);
       
//////////////////////////////////////////////////////////////////////////////////////////
//     Equation 13 : var33 = ((int32_t)(_Pressure >> 8) * (int32_t)(_Pressure >> 8) *   (int32_t)(_Pressure >> 8) * (int32_t)dev->calib.par_p10) >> 17;
//////////////////////////////////////////////////////////////////////////////////////////

      var33 = ((int32_t)(_Pressure >> 8)*(int32_t)(_Pressure >> 8)*(int32_t)(_Pressure >> 8)*(int32_t)_P10);
      
      //fprintf(STDOUT, "**********   Equation 13 var33 = %Lu\r\n\r\n", var33);
      if (var33 < 0)
      {
         var33 = (var33 >> 17);
         var33 = var33 + 0xffff8000;
         //fprintf(STDOUT, "**********   Equation 13 var33 = %Lu\r\n\r\n", var33);
      }
      
      else
      {
         var33 = (var33 >> 17);
         //fprintf(STDOUT, "**********   Equation 13 final output = %Lu\r\n\r\n", var33);
      }
////////////////////////////////////////////////////////////////////////////////////////////
//     Equation 14 : _Pressure = (int32_t)(_Pressure)+((var11+var22+var33+((int32_t)dev->calib.par_p7<<7))>>4);
////////////////////////////////////////////////////////////////////////////////////////////
      var4 = (var11+var22+var33+((int32_t)_P7<<7));
      
      //fprintf(STDOUT, "**********   Equation 14 var4 = %Lu\r\n\r\n", var4);
      if (var4 < 0)
      {
         var4 = var4 >> 4;
         var4 = var4 + 0xf0000000;
      }
      
      else
      {
         var4 = var4 >> 4;
      }
      
      //fprintf(STDOUT, "**********   Equation 14 var4 = %Lu\r\n\r\n", var4);
      _Pressure = (int32_t)(_Pressure)+ var4;
      //fprintf(STDOUT, "\r\n\r\n*   Final Pressure  before %Lu Pascal \r\n", _Pressure);
       
/////////////////////////////////////////////////////////////////////////////////////////////
//     Update variable counts
/////////////////////////////////////////////////////////////////////////////////////////////
      count +=1;
      temperature_received = pressure_received = humidity_received = gas_received = 0;
      var11 = var22 = var33 = var1 = var2 = var3 = var4 =0;     
      
      //fprintf(STDOUT, "*   Final Pressure after %Lu Pascal \r\n", _Pressure);
      //uint32_t Pressure = calc_pressure(pressure_received,dev);
      //fprintf(STDOUT, "*   Pressure  %Lu Pascal \r\n", _Pressure);
      //fprintf(STDOUT, "**********   Library Pressure  %Lu Pascal \r\n", _Pressure);
      
      pressure1 = _Pressure;
      
      //fprintf(STDOUT, "Current Pressure %lu", pressure1 );
      if(startCount)
      {
         startCount--;
         startPressure = pressure1;
      }      
      
#ifdef OLD_FLOW_TALBE  
      deltaP = abs((int32_t)pressure1-(int32_t)startPressure);
#else
      if( pressure1 > startPressure )
      {
         deltaP = (int32_t)pressure1-(int32_t)startPressure;
      }
      
      else
      {
         deltaP = 0;
      } 
#endif      
   
      if(timeStamp1<prevTimeStamp1) deltaT = 65535-prevTimeStamp1+timeStamp1;
   
      else                          deltaT = timeStamp1-prevTimeStamp1;


      switch (breathTestState)
      {   
         case BREATH_TEST_NOT_RUN:
           
            if( deltaP >= dPressureThresheldStart )//&& abs((int32_t)deltaP - (int32_t)prevdeltaP)>6)//Start or continue volume accumulation
            {
               deltaVol_1 = GetNewVolume(dPressureThresheldStart/2, deltaT); //
               AccumulatedVolume1_ml = deltaVol_1;
               accumulatedTimeMs = deltaT;
               lowPressureLastCount = 0;
               breathTestState = BREATH_TEST_STARTED;
               timerFlag = ON;

               //Test stared turn of notification light
               output_low(ILLUMINATION_588);
               inVolumeAccumulation = 1;//no longer needed
               
#ifdef DEBUG
               fprintf(STDOUT, "\r\n\r\n");
#endif               
            }
      
            else
            {
               startPressure = pressure1;  
               timer_current = get_ticks();
               timer_diff = timer_current - timer_previous;
               
               
               if( timer_diff > 4500 )
               {
                  illumination_brightness( 30 );
                  illumination_use(ILLUMINATION_588);
               }
               
               if( timer_diff > FIVE_SECONDS )
               {
                  output_low(ILLUMINATION_588);
                  timerCtr += timer_diff;
                  timer_previous = timer_current;
                  fprintf(STDOUT, "Unit is ready for testing, please start breathing into the mouth piece\r\n\r\n");
               }

               if( timerCtr > FOUR_MINUTES )
               {
                  breathTestState = BREATH_TEST_ENDED;
                  bleNotifyErrorSet(TIME_OUT);
                  delay_ms( ONE_MINUTE );
                  testResult.status = TIME_OUT;
                  illumination_all_off();    
                  timer_previous = 0;
               }
            }      
            
            break;
   
         case BREATH_TEST_STARTED:
           
//            fprintf(STDOUT, "Breath volume setting %lu\r\n", breathVolume);   
            output_low(ILLUMINATION_588);
            deltaVol_1 = GetNewVolume(deltaP, deltaT);
            tmpVol = AccumulatedVolume1_ml += deltaVol_1;
            accumulatedTimeMs += deltaT;

            tmpVol = breathVolume + 10 - tmpVol;
            float tmpVol1 = tmpVol / breathVolume; 
            tmpVol1 *= 100;              

            //illumination_brightness( tmpVol1 );
            //illumination_use(ILLUMINATION_588);
            
            //Reach the test mid-point - update the BLE data struct
            if( ( tmpVol1 > 48 ) && ( tmpVol1 < 52 ) )      
            {
               //update the pressure 
               testStat.pressure = deltaP;
               
               //update the temperature
               testStat.temperature = f_PICtoIEEE(temperature);
               
               //update the humidity
               //Note - humidity not define yet...
               
            }

            if( (AccumulatedVolume1_ml > ( breathVolume - 15 ))  && earlyWarningFlag )      
            {
               //fprintf(STDOUT, "\r\n\r\n Early warning Accumulated Volume %.2f \r\n\r\n", AccumulatedVolume1_ml);
               pwm_on(BUZZER);
               tone(360);
               earlyWarningFlag = OFF;
            }

            if( AccumulatedVolume1_ml >= breathVolume )   
            {
               testStat.volume = (uint16_t)AccumulatedVolume1_ml;
               breathTestState = BREATH_TEST_COMPLETED;
               bleNotifyErrorSet(NO_ERROR);
               status = PASS;
            }
            
            if( prevdeltaP == deltaP || prevdeltaP + 1 == deltaP || prevdeltaP - 1 == deltaP )
            {
               pressureChangeNotDetected++;
//               fprintf(STDOUT, "\r\nChange in deltaP not detected**** %d \r\n\r\n", pressureChangeNotDetected);
            }
               
            else
            {
               pressureChangeNotDetected = 0;
//               fprintf(STDOUT, "\r\nPressure change detected \r\n\r\n");
            }

//            if( ( deltaP < dPressureThresheldEnd ) ||  ( abs((int32_t)deltaP - (int32_t)prevdeltaP)<4 ) )
//            if( deltaP < dPressureThresheldEnd )

            if( deltaP < dPressureThresheldEnd || pressureChangeNotDetected >= 2 )
            {  
               lowPressureLastCount++;
               
               if(lowPressureLastCount > 2)
               {
                  output_low(ILLUMINATION_588);
                  startCount = 5;
                  deltaP = pressure1 = _Pressure = pressure_received = 0;
                  AccumulatedVolume1_ml = 0.0; //AccumulatedVolume2_ml = 0.0;
                  accumulatedTimeMs = 0;
                  numberOfAttempts++;
                  inVolumeAccumulation = 0;
                  lowPressureLastCount = 0;
                  earlyWarningFlag = ON;
                  pwm_off(BUZZER);
                  breathTestState = BREATH_TEST_NOT_RUN;
                  bleNotifyErrorSet(BAD_BLOW_PRESSURE);
                  status = LOW_PRESSURE_BREATH;
                  
#ifdef DEBUG
                  fprintf(STDOUT, "\r\n\r\n Volume reset to 0, P1= %lu, lowPressureLastCount= %lu,  deltaP= %lu/%lu \r\n\r\n", pressure1, lowPressureLastCount, prevdeltaP, deltaP);
#endif        
             
               }
               
               pressureChangeNotDetected = 0;
            }

            break;

         case BREATH_TEST_EXITING:
            breathTestState = BREATH_TEST_COMPLETED;
            testStat.numOfBlowAttempts = numberOfAttempts;
            
            break;
   
         case BREATH_TEST_COMPLETED:
            fprintf(STDOUT, "\r\n\r\n");
            pwm_off(BUZZER);
            AccumulatedVolume1_ml = 0.0; //AccumulatedVolume2_ml = 0.0;
            deltaVol_1 = 0.0;
            accumulatedTimeMs = 0;
            inVolumeAccumulation = 0;
            breathTestState = BREATH_TEST_ENDED;
            break;  
   
         default:
            status = BME680_SENSOR_FAIL; 
            fprintf(STDOUT, " ******* BME state machine failed *******"); 
      }

      prevTimeStamp1 = timeStamp1; 

#ifdef DEBUG
      float BreathTimePeriod = (float)accumulatedTimeMs/(1000.0);
      fprintf(STDOUT, "%lu / %05lu ms: Humidity %0.2f%%, Temp %lu / %.2f, pressure_received %lu pressure_comp %lu\r\n",  
      count, timeTickMs, humi, temperature_received, temperature, pressure_received, pressure_comp);
      fprintf(STDOUT, "%lu / %05lu ms: Temp %08LX / %.2f, pressure_received %08LX pressure_comp %lu\r\n",  
      count, timeTickMs, temperature_received, temperature, pressure_received, pressure_comp);
      temperature_received = pressure_received = humidity_received = gas_received = 0;      
#endif

      

      if( breathTestState == BREATH_TEST_ENDED )
         break;
      
      else
      {
         if( AccumulatedVolume1_ml > 0 )
         {
            fprintf(STDOUT, "%lu / %05lu ms: state %d, %lu; Temp %.2f; CurrentPressure: %lu; BasePressure: %lu; deltaP: %lu/%lu; flowLPM: %.2f; deltaVol: %.2f; AccumulatedVolume: %.2f ml\r\n",  
            count, timeStamp1, breathTestState, lowPressureLastCount, temperature, pressure1, startPressure, prevdeltaP, deltaP, flowLPM, deltaVol_1, AccumulatedVolume1_ml );
         }
      }
      
      prevdeltaP = deltaP;      
   }
   
   illumination_all_off();
   return status;
}

///////////////////////////////////////////////////////////////////////////////
int8_t SetBME_Oversampling_and_00IIR(struct bme680_dev *dev, uint8 osrs_h, uint8 osrs_t, uint8 osrs_p, uint8 filter)
{
   uint8    control_reg;
   uint8    tmp, tmp1;
   uint8    temp_mem_page;
   uint8    tmp_osrs_h, tmp_osrs_t, tmp_osrs_p;
   
   tmp_osrs_h = osrs_h;
   tmp_osrs_t = osrs_t;   
   tmp_osrs_p = osrs_p;
   
   tmp_osrs_t <<= 5;                  
   tmp_osrs_p <<= 2;                  
   
   //Get mem page
   temp_mem_page = Read_Byte(BME680_SPI_STATUS);
   
   if( temp_mem_page != BME680_MEM_PAGE1 )
      Write_Byte( BME680_SPI_STATUS, BME680_MEM_PAGE1 );   //Set mem page to 1
    
   //Check the mem page
   temp_mem_page = Read_Byte(BME680_SPI_STATUS);
   temp_mem_page &= 0x10;
   
   if( temp_mem_page != BME680_MEM_PAGE1 )
   {
#ifdef DEBUG   
      fprintf(STDOUT, " Set mem page failed at Oversampling \r\n");
#endif      
      return BME680_SENSOR_FAIL;
   }
   
   //Update the gas sensor structure
   dev->mem_page = temp_mem_page;

   //Set oversampling for temperature and pressure value
   control_reg = tmp_osrs_t | tmp_osrs_p;

#ifdef DEBUG
   fprintf(STDOUT, "**********control_reg = %02X\r\n", control_reg);
   fprintf(STDOUT, "*********    SPI   MODE   SELECT  *******\r\n");

   /* Set Oversampling */
   //Read values of oversampling register before writing
   Read_Byte(BME680_SPI_CTRL);
   Read_Byte(BME680_SPI_CTRL_HUM);
#endif
  
   //Set oversampling values for temperature, pressure, and humidity
   Write_Byte( BME680_SPI_CTRL_HUM, tmp_osrs_h );  
   Write_Byte( BME680_SPI_CTRL, control_reg );
   
   //Read back and verify 
   tmp1 = Read_Byte(BME680_SPI_CTRL_HUM);
   tmp = Read_Byte(BME680_SPI_CTRL);
   
   tmp1 &= 0x7;     //Mask the 5 MSb 
   tmp &= 0xFC;     //Mask the 2 LSb
   
   if( tmp1 != osrs_h || tmp != control_reg )
   {
#ifdef DEBUG   
      fprintf(STDOUT, "*** Failed oversampling setting ***\r\n");
#endif   
      return BME680_SENSOR_FAIL;
   }
   
   //Update the gas sensor structure
   dev->tph_sett.os_hum = osrs_h;
   dev->tph_sett.os_temp = osrs_t;
   dev->tph_sett.os_pres = osrs_p;
   
   //Read values of oversampling register after writing
   Read_Byte(BME680_SPI_CTRL);
   Read_Byte(BME680_SPI_CTRL_HUM);

   /* Set IIR Filter */
   filter <<= 2;
   
   //Set filter values for temperature
   Write_Byte( BME680_SPI_CONFIG, filter );  

   //Read back and verify
   tmp = Read_Byte(BME680_SPI_CONFIG);
   if( tmp != filter )
   {
#ifdef DEBUG   
      fprintf(STDOUT, "*** Failed IIR setting ***\r\n");
#endif   
      return BME680_SENSOR_FAIL;
   }
   
   //Update the gas sensor structure
   dev->tph_sett.filter = filter;
   
   return BME680_OK;
}

///////////////////////////////////////////////////////////////////////////////
int8_t checkBMEPowerUp(struct bme680_dev *dev)
{
   int8_t rslt = BME680_OK;
   uint8  chipIDAddr;
      
   if( dev->intf  == BME680_I2C_INTF) 
      chipIDAddr = BME680_I2CCHIP_ID_ADDR;

   else      
      chipIDAddr = BME680_SPI_CHIP_ID_ADDR;
      
   /* Check for null pointer in the device structure*/
   rslt = null_ptr_check(dev);
   
   if (rslt != BME680_OK) 
      return NULL_POINTER_DETECTED;
   
   /* Soft reset to restore it to default values*/
   rslt = bme680_soft_reset(dev);
      
   if (rslt != BME680_OK)
   {
      fprintf(STDOUT, "Failed BME_SOFT_RESET \r\n");
      return BME680_SENSOR_FAIL;
   }

   dev->chip_id = Read_Byte(chipIDAddr);  
   if (dev->chip_id != BME680_CHIP_ID) 
      return BME680_SENSOR_FAIL;

#ifdef DEBUG
   fprintf(STDOUT, "BME680_CHIP_ID = %02X\r\n", dev->chip_id);
#endif   
   
   /* Get the Calibration data */
   rslt = get_calib_data(dev);
   
   if (rslt != BME680_OK)
   {
      fprintf(STDOUT, "BME Calib Init failed \r\n");
      rslt = BME680_E_DEV_NOT_FOUND;
   }

#ifdef DEBUG   
    rslt = bme680_get_regs(TMP_HUM_ADDR, &temp, 1, dev);
    fprintf(STDOUT, "hum_msb default value = %02X\r\n", temp);
    
    rslt = bme680_get_regs(TMP_MSB_ADDR, &temp, 1, dev);
    fprintf(STDOUT, "tmp_msb default value = %02X\r\n", temp);
    
    rslt = bme680_get_regs(PRESS_MSB_ADDR, &temp, 1, dev);
    fprintf(STDOUT, "press_msb default value = %02X\r\n", temp);
#endif

   return rslt;
}

///////////////////////////////////////////////////////////////////////////////
/*!
 * @brief This API performs the soft reset of the sensor.
 */
int8_t bme680_soft_reset(struct bme680_dev *dev)
{
   int8_t   rslt = BME680_OK;
   uint8    temp_mem_page;
   uint8_t  reg_addr;
   
   /* 0xb6 is the soft reset command */
   uint8_t soft_rst_cmd = BME680_SOFT_RESET_CMD;
   
   //Set mem page only apply to SPI mode
   if( dev->intf == BME680_I2C_INTF )
      reg_addr = BME680_I2C_SOFT_RESET_ADDR;
   
   else
   {
      reg_addr = BME680_SPI_SOFT_RESET_ADDR;
      rslt = set_mem_page( reg_addr, dev );
   }
        
   /* Reset the device */
   if (rslt != BME680_OK) 
      return BME680_SENSOR_FAIL;
   
   Write_Byte(reg_addr, soft_rst_cmd);

   /* Wait for 20ms */
   delay_ms(20);
   temp_mem_page = Read_Byte(BME680_SPI_STATUS);  
   temp_mem_page &= 0x10;
       
   //Update the gas sensor structure
   dev->mem_page = temp_mem_page;

   return rslt;
}

///////////////////////////////////////////////////////////////////////////////
/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev)
{
   int8_t rslt;

   /* Check for null pointer in the device structure*/
   rslt = null_ptr_check(dev);
   if (rslt == BME680_OK) {
      if (dev->intf == BME680_SPI_INTF) {
         /* Set the memory page */
         rslt = set_mem_page(reg_addr, dev);
         if (rslt == BME680_OK)
            reg_addr = reg_addr | BME680_SPI_RD_MSK;
      }
      
      //rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);
      rslt = Read(dev->dev_id, reg_addr, reg_data, len);
      if ( rslt != BME680_OK)
         return BME680_E_COM_FAIL;
   }

   return rslt;
}

/*****************************************************************************/
/*!
 * @brief This internal API is used to calculate the temperature value.
 */
int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
   int32_t var1;
   int32_t var2;
   int32_t var3;
   int16_t calc_temp;
 
   var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->calib.par_t1 << 1);
   var2 = (var1 * (int32_t) dev->calib.par_t2) >> 11;
   var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
   var3 = ((var3) * ((int32_t) dev->calib.par_t3 << 4)) >> 14;
   dev->calib.t_fine = (int32_t) (var2 + var3);
   calc_temp = (int16_t) (((dev->calib.t_fine * 5) + 128) >> 8);
 
   return calc_temp;
}
/*****************************************************************************/
/*!
 * @brief This internal API is used to read the calibrated data from the sensor.
 */
int8_t get_calib_data(struct bme680_dev *dev)
{
   uint32_t    _T1,_T2,_T3;
   uint32_t    _P1,_P2,_P4,_P8,_P9,_P5,_P10;
   int32_t     _P3,_P6,_P7;       
   
    int8_t  rslt = BME680_OK;
//    int8_t  temp8;
//    int16   temp16;
//    uint8   u_temp8;
//    uint16  u_temp16;
    uint8   temp_mem_page;
    uint8_t temp_var = 0;     /* Temporary variable */
    
   if( dev->intf != BME680_I2C_INTF )       
   {
      //Check for mem page 0
      temp_mem_page = Read_Byte(BME680_SPI_STATUS);  
      temp_mem_page >>= 4;
      temp_mem_page &= 0x1;

      if( temp_mem_page != BME680_MEM_PAGE0 )
      {
         //Set mem page to 0
         Write_Byte( BME680_SPI_STATUS, BME680_MEM_PAGE0 );  
         
         //Check the mem page
         temp_mem_page = Read_Byte(BME680_SPI_STATUS);      
   
         if( temp_mem_page == BME680_MEM_PAGE0 )
            dev->mem_page = BME680_MEM_PAGE0;
      }
   }

   if (rslt == BME680_OK)
   {
      /**************************************/
      /* Set up Calib Temperature data t1-t3*/
      /**************************************/
      _T1 = ( Read_Byte(0xEA) << 8 );
      _T1 |= Read_Byte(0xE9);
      dev->calib.par_t1 = _T1;

      _T2 = ( Read_Byte(0x8B) << 8 );
      _T2 |= Read_Byte(0x8A);
      dev->calib.par_t2 = _T2;
      
      _T3 = Read_Byte(0x8C);
      dev->calib.par_t3 = _T3;
   
      /************************************/
      /* Set up Calib Pressure data p1-p10*/
      /************************************/
      
      _P1 = ( Read_Byte(0x8F) << 8 );
      _P1 |= Read_Byte(0x8E);
      //_P1 = 0xFFFF0000 | _P1;
      dev->calib.par_p1 = _P1;

      _P2 = ( Read_Byte(0x91) << 8 );
      _P2 |= Read_Byte(0x90);
      _P2 = 0xFFFF0000 | _P2;
      dev->calib.par_p2 = _P2;
   
      _P3 = Read_Byte(0x92);
      dev->calib.par_p3 = _P3;

      _P4 = ( Read_Byte(0x95) << 8 );
      _P4 |= Read_Byte(0x94);
      //_P4 = 0xFFFF0000 | _P4;
      dev->calib.par_p4 = _P4;

      _P5 = ( Read_Byte(0x97) << 8 );
      _P5 |=  Read_Byte(0x96);
      _P5 = 0xFFFF0000 | _P5;
      dev->calib.par_p5 = _P5;
  
      _P6 = Read_Byte(0x99);
      dev->calib.par_p6 = _P6;
  
      _P7 = Read_Byte(0x98);
      dev->calib.par_p7 = _P7;

      _P8 = ( Read_Byte(0x9D) << 8 );
      _P8 |= Read_Byte(0x9C);
      _P8 = 0xFFFF0000 | _P8;
      dev->calib.par_p8 = _P8;

      _P9 = ( Read_Byte(0x9F) << 8 );
      _P9 |= Read_Byte(0x9E);
      _P9 = 0xFFFF0000 | _P9;
      dev->calib.par_p9 = _P9;

      _P10 = Read_Byte(0xA0);
      _P10 = 0xFFFF0000 | _P10;
      dev->calib.par_p10 = _P10;

#ifdef DEBUG
   fprintf(STDOUT, "**********   _T1 = %Lu\r\n\r\n", _T1);
   fprintf(STDOUT, "**********   _T2 = %Lu\r\n\r\n", _T2);
   fprintf(STDOUT, "**********   _T3 = %Lu\r\n\r\n", _T3);
   fprintf(STDOUT, "**********   _P1 = %Lu\r\n\r\n", _P1);
   fprintf(STDOUT, "**********   _P2 = %Lu\r\n\r\n", _P2);
   fprintf(STDOUT, "**********   _P3 = %Lu\r\n\r\n", _P3);
   fprintf(STDOUT, "**********   _P4 = %Lu\r\n\r\n", _P4);
   fprintf(STDOUT, "**********   _P5 = %Lu\r\n\r\n", _P5);
   fprintf(STDOUT, "**********   _P6 = %Lu\r\n\r\n", _P6);
   fprintf(STDOUT, "**********   _P7 = %Lu\r\n\r\n", _P7);
   fprintf(STDOUT, "**********   _P8 = %Lu\r\n\r\n", _P8);
   fprintf(STDOUT, "**********   _P9 = %Lu\r\n\r\n", _P9);
   fprintf(STDOUT, "**********   _P10 = %Lu\r\n\r\n", _P10);
#endif
       
         /* Other coefficients */
      if (rslt == BME680_OK) 
      {
         rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1, dev);
         dev->calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
   
         if (rslt == BME680_OK)
         {
            rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1, dev);
            dev->calib.res_heat_val = (int8_t) temp_var;
         
            if (rslt == BME680_OK)
               rslt = bme680_get_regs(BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1, dev);
         }
      }
        
      dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;

   }
 
   return rslt;
}

/*****************************************************************************/
/*!
 * @brief This internal API is used to set the memory page based on register address.
 */
int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev)
{
   int8_t rslt = BME680_OK;
   uint8_t mem_page, tmp;
    
   if ( reg_addr == 0x60 || reg_addr == 0x50 || reg_addr == 0xE0 || reg_addr == 0xD0 )
         mem_page = BME680_MEM_PAGE0;
   
   else
      mem_page = BME680_MEM_PAGE1;

#ifdef DEBUG    
   fprintf(STDOUT, "Current Mem_page %02X\r\n", dev->mem_page);
   fprintf(STDOUT, "Set mem_page to %02X\r\n", mem_page);
#endif

   //update the mem_page setting
   Write_Byte(BME680_MEM_PAGE_ADDR, mem_page);
   delay_ms(100);
        
   //Verify
   tmp = get_mem_page(dev);
       
   if ( tmp != mem_page)
      rslt = BME680_E_COM_FAIL;
         
   else
      dev->mem_page = mem_page;
 
    return rslt;
}
 
/*****************************************************************************/ 
/*!
 * @brief This internal API is used to get the memory page based on register address.
 */
int8_t get_mem_page(struct bme680_dev *dev)
{
   int8_t rslt;
    
   rslt = Read_Byte(BME680_MEM_PAGE_ADDR);
   
   if ( rslt != 0 || rslt != 0x10 )
      return BME680_E_COM_FAIL;
        
   else
   {
      dev->mem_page = rslt;
      return BME680_OK;
   }
}

/*****************************************************************************/
/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
int8_t null_ptr_check(const struct bme680_dev *dev)
{
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL)) 
    {
#ifdef DEBUG   
      fprintf(STDOUT, "Null pointer detected in null_ptr_check\r\n");
#endif
      return NULL_POINTER_DETECTED;                    
    }
 
    return BME680_OK;
}

/*****************************************************************************/
void Write_Byte(uint8_t reg_addr, uint8_t data)
{
#ifdef DEBUG   
   fprintf(STDOUT, "Write_Byte 0x%X to reg 0x%X\r\n", data, reg_addr);
#endif

#ifdef BME_SPI_MODE
   uint8_t bit0 = 1;
   uint8_t regAddr = reg_addr&0x7F; //Bit7 = 0 for write
   

   output_low(PIN_E2);
   delay_ms(1); 
   //send reg address
   for(int i=0; i<8;i++)
   {
      output_low(PIN_C3);//clk low
      delay_ms(1); 
      if(regAddr&(bit0<<(7-i)))
         output_high(PIN_C4);
      else                      
         output_low(PIN_C4);
      delay_ms(1); 
      output_high(PIN_C3);//clk hi
      delay_ms(1); 
   }
   delay_ms(2); 
   //send data byte 
   for(int i=0; i<8;i++)
   {
      output_low(PIN_C3);//clk low
      delay_ms(1); 
      if(data&(bit0<<(7-i)))
         output_high(PIN_C4);
      else                      
         output_low(PIN_C4);
      delay_ms(1); 
      output_high(PIN_C3);//clk hi
      delay_ms(1); 
   }   
   
   output_high(PIN_E2);
   delay_ms(1); 

#else // use I2C
   output_low(PIN_D4);
//   output_high(PIN_D4);
   i2c_start();
   i2c_write(BME680_I2C_Write_addr);
   i2c_write(reg_addr);
   i2c_write(data);
   i2c_stop();
#endif
}

uint8_t Read_Byte(uint8_t reg_addr)
{
   uint8_t dataByte = 0;

#ifdef BME_SPI_MODE
   uint8_t bit0 = 1;
   uint8_t regAddr = reg_addr|0x80; //Bit7 = 1 for read

   output_low(PIN_E2);
   delay_ms(1); 
   //send reg address
   for(int i=0; i<8;i++)
   {
      output_low(PIN_C3);//clk low
      if(regAddr&(bit0<<(7-i)))
         output_high(PIN_C4);
      else                      
         output_low(PIN_C4);
      output_high(PIN_C3);//clk hi
   }
   delay_ms(4); 
   //get data byte 
   for(int i=0; i<8;i++)
   {
      output_low(PIN_C3);//clk low
      if( input(PIN_D4) )
         dataByte |= (bit0<<(7-i));
      output_high(PIN_C3);//clk hi
   }   
   
   output_high(PIN_E2);
   delay_ms(1); 
   
#else // use I2C
   output_low(PIN_D4);
//   output_high(PIN_D4);
   i2c_start();
   i2c_write(BME680_I2C_Write_addr);
   i2c_write(reg_addr);
   i2c_start();
   i2c_write(BME680_I2C_Read_addr);
   dataByte = i2c_read(0);
   i2c_stop();  
#endif

#ifdef DEBUG    
   fprintf(STDOUT, "Read_Ryte 0x%X from reg 0x%X\r\n", dataByte, reg_addr);
#endif

   return dataByte;
}

/*****************************************************************************/
/*****************************************************************************/
int8_t Write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{  
   for(int i=0; i<len; i++)
   {
      Write_Byte(reg_addr+i,*(data+i));
   }
   
   return BME680_OK;
}

int8_t Read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
   uint8_t     command;
   int8_t      status = BME680_OK;
   uint8_t     tmp_data;
   
   if( !len )
      return BME680_E_INVALID_LENGTH;
      
   command = reg_addr;
#ifdef DEBUG   
   fprintf(STDOUT, "read command number 0x%LX\r\n", command);
#endif
   while( len-- )
   {
      //Assumtion - Only one byte read at this time for BME.  CCS compiler limitation of using spi_xfer.
      tmp_data = Read_Byte(command);
      *data++ = tmp_data;
#ifdef DEBUG      
      fprintf(STDOUT, "Contents of read command 0x%X\r\n", tmp_data);
#endif      
   }  
   return status;
}


///////////////////////////////////////////////////////////////////////////////

#ifdef BME_CODE

/*!
 *@brief This API is the entry point.
 *It reads the chip-id and calibration data from the sensor.
 */
int8_t bme680_init(struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t temp;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    
    if (rslt == BME680_OK) 
    {
        /* Soft reset to restore it to default values*/
        rslt = bme680_soft_reset(dev);
        if (rslt != BME680_OK)
           fprintf(STDOUT, "Failed BME_SOFT_RESET \r\n");
        
        if (rslt == BME680_OK)
        {
            rslt = bme680_get_regs(BME680_SPI_CHIP_ID_ADDR, &temp, 1, dev);
            if (rslt == BME680_OK) 
            {
               fprintf(STDOUT, "\r\n\r\n");
               
               if (temp == BME680_CHIP_ID)
               {
                  dev->chip_id = temp;
                  fprintf(STDOUT, "BME680_CHIP_ID = %02X\r\n", dev->chip_id);
                  
                  /* Get the Calibration data */
                  rslt = get_calib_data(dev);
               }
                
               else 
               {
                  fprintf(STDOUT, "BME680_CHIP_ID = %02X/%02X\r\n", dev->chip_id, BME680_CHIP_ID);
                  rslt = BME680_E_DEV_NOT_FOUND;
               }
            }
        }
    } 
    
    else
       fprintf(STDOUT, "      ERROR in    bme680_init = %u\r\n", rslt);

    return rslt;
}
 
/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev)
{
   int8_t rslt;
   /* Length of the temporary buffer is 2*(length of register)*/
   uint8_t tmp_buff[BME680_TMP_BUFFER_LENGTH] = { 0 };
   uint16_t index;

   /* Check for null pointer in the device structure*/
   rslt = null_ptr_check(dev);
   if (rslt == BME680_OK) {
      if ((len > 0) && (len < BME680_TMP_BUFFER_LENGTH / 2)) {
         /* Interleave the 2 arrays */
         for (index = 0; index < len; index++) {
            if (dev->intf == BME680_SPI_INTF) {
               /* Set the memory page */
               rslt = set_mem_page(reg_addr[index], dev);
               tmp_buff[(2 * index)] = reg_addr[index] & BME680_SPI_WR_MSK;
            } else {
               tmp_buff[(2 * index)] = reg_addr[index];
            }
            tmp_buff[(2 * index) + 1] = reg_data[index];
         }
         /* Write the interleaved array */
         if (rslt == BME680_OK)
         {
            //dev->com_rslt = dev->write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);
            dev->com_rslt = Write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);
            if (dev->com_rslt != 0)
               rslt = BME680_E_COM_FAIL;
         }
      } else {
         rslt = BME680_E_INVALID_LENGTH;
      }
   }

   return rslt;
}
 

 
/*!
 * @brief This API is used to set the oversampling, filter and T,P,H, gas selection
 * settings in the sensor.
 */
int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr;
    uint8_t data = 0;
    uint8_t count = 0;
    uint8_t reg_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    uint8_t intended_power_mode = dev->power_mode; /* Save intended power mode */
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        if (desired_settings & BME680_GAS_MEAS_SEL)
            rslt = set_gas_config(dev);
 
        dev->power_mode = BME680_SLEEP_MODE;
        if (rslt == BME680_OK)
            rslt = bme680_set_sensor_mode(dev);
 
        /* Selecting the filter */
        if (desired_settings & BME680_FILTER_SEL) {
            rslt = boundary_check(&dev->tph_sett.filter, BME680_FILTER_SIZE_0, BME680_FILTER_SIZE_127, dev);
            reg_addr = BME680_CONF_ODR_FILT_ADDR;
 
            if (rslt == BME680_OK)
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
 
            if (desired_settings & BME680_FILTER_SEL)
                data = BME680_SET_BITS(data, BME680_FILTER, dev->tph_sett.filter);
 
            reg_array[count] = reg_addr; /* Append configuration */
            data_array[count] = data;
            count++;
        }
 
        /* Selecting heater control for the sensor */
        if (desired_settings & BME680_HCNTRL_SEL) {
            rslt = boundary_check(&dev->gas_sett.heatr_ctrl, BME680_ENABLE_HEATER,
                                  BME680_DISABLE_HEATER, dev);
            reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
 
            if (rslt == BME680_OK)
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
            data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev->gas_sett.heatr_ctrl);
 
            reg_array[count] = reg_addr; /* Append configuration */
            data_array[count] = data;
            count++;
        }
 
        /* Selecting heater T,P oversampling for the sensor */
        if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
            rslt = boundary_check(&dev->tph_sett.os_temp, BME680_OS_NONE, BME680_OS_16X, dev);
            reg_addr = BME680_CONF_T_P_MODE_ADDR;
 
            if (rslt == BME680_OK)
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
 
            if (desired_settings & BME680_OST_SEL)
                data = BME680_SET_BITS(data, BME680_OST, dev->tph_sett.os_temp);
 
            if (desired_settings & BME680_OSP_SEL)
                data = BME680_SET_BITS(data, BME680_OSP, dev->tph_sett.os_pres);
 
            reg_array[count] = reg_addr;
            data_array[count] = data;
            count++;
        }
 
        /* Selecting humidity oversampling for the sensor */
        if (desired_settings & BME680_OSH_SEL) {
            rslt = boundary_check(&dev->tph_sett.os_hum, BME680_OS_NONE, BME680_OS_16X, dev);
            reg_addr = BME680_CONF_OS_H_ADDR;
 
            if (rslt == BME680_OK)
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
            data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev->tph_sett.os_hum);
 
            reg_array[count] = reg_addr; /* Append configuration */
            data_array[count] = data;
            count++;
        }
 
        /* Selecting the runGas and NB conversion settings for the sensor */
        if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
            rslt = boundary_check(&dev->gas_sett.run_gas, BME680_RUN_GAS_DISABLE,
                                  BME680_RUN_GAS_ENABLE, dev);
            if (rslt == BME680_OK) {
                /* Validate boundary conditions */
                rslt = boundary_check(&dev->gas_sett.nb_conv, BME680_NBCONV_MIN,
                                      BME680_NBCONV_MAX, dev);
            }
 
            reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;
 
            if (rslt == BME680_OK)
                rslt = bme680_get_regs(reg_addr, &data, 1, dev);
 
            if (desired_settings & BME680_RUN_GAS_SEL)
                data = BME680_SET_BITS(data, BME680_RUN_GAS, dev->gas_sett.run_gas);
 
            if (desired_settings & BME680_NBCONV_SEL)
                data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev->gas_sett.nb_conv);
 
            reg_array[count] = reg_addr; /* Append configuration */
            data_array[count] = data;
            count++;
        }
 
        if (rslt == BME680_OK)
            rslt = bme680_set_regs(reg_array, data_array, count, dev);
 
        /* Restore previous intended power mode */
        dev->power_mode = intended_power_mode;
    }
 
    return rslt;
}
 
/*!
 * @brief This API is used to get the oversampling, filter and T,P,H, gas selection
 * settings in the sensor.
 */
int8_t bme680_get_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
    int8_t rslt;
    /* starting address of the register array for burst read*/
    uint8_t reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
    uint8_t data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        rslt = bme680_get_regs(reg_addr, data_array, BME680_REG_BUFFER_LENGTH, dev);
 
        if (rslt == BME680_OK) {
            if (desired_settings & BME680_GAS_MEAS_SEL)
                rslt = get_gas_config(dev);
 
            /* get the T,P,H ,Filter,ODR settings here */
            if (desired_settings & BME680_FILTER_SEL)
                dev->tph_sett.filter = BME680_GET_BITS(data_array[BME680_REG_FILTER_INDEX],
                                                       BME680_FILTER);
 
            if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
                dev->tph_sett.os_temp = BME680_GET_BITS(data_array[BME680_REG_TEMP_INDEX], BME680_OST);
                dev->tph_sett.os_pres = BME680_GET_BITS(data_array[BME680_REG_PRES_INDEX], BME680_OSP);
            }
 
            if (desired_settings & BME680_OSH_SEL)
                dev->tph_sett.os_hum = BME680_GET_BITS_POS_0(data_array[BME680_REG_HUM_INDEX],
                                       BME680_OSH);
 
            /* get the gas related settings */
            if (desired_settings & BME680_HCNTRL_SEL)
                dev->gas_sett.heatr_ctrl = BME680_GET_BITS_POS_0(data_array[BME680_REG_HCTRL_INDEX],
                                           BME680_HCTRL);
 
            if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
                dev->gas_sett.nb_conv = BME680_GET_BITS_POS_0(data_array[BME680_REG_NBCONV_INDEX],
                                        BME680_NBCONV);
                dev->gas_sett.run_gas = BME680_GET_BITS(data_array[BME680_REG_RUN_GAS_INDEX],
                                                        BME680_RUN_GAS);
            }
        }
    } else {
        rslt = BME680_E_NULL_PTR;
    }
 
    return rslt;
}
 
/*!
 * @brief This API is used to set the power mode of the sensor.
 */
int8_t bme680_set_sensor_mode(struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;
    uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        /* Call repeatedly until in sleep */
        do {
            rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1, dev);
            if (rslt == BME680_OK) {
                /* Put to sleep before changing mode */
                pow_mode = (tmp_pow_mode & BME680_MODE_MSK);
 
                if (pow_mode != BME680_SLEEP_MODE) {
                    tmp_pow_mode = tmp_pow_mode & (~BME680_MODE_MSK); /* Set to sleep */
                    rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
                    delay_ms(BME680_POLL_PERIOD_MS);
                }
            }
        } while (pow_mode != BME680_SLEEP_MODE);
 
        /* Already in sleep */
        if (dev->power_mode != BME680_SLEEP_MODE) {
            tmp_pow_mode = (tmp_pow_mode & ~BME680_MODE_MSK) | (dev->power_mode & BME680_MODE_MSK);
            if (rslt == BME680_OK)
                rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
        }
    }
 
    return rslt;
}
 
/*!
 * @brief This API is used to get the power mode of the sensor.
 */
int8_t bme680_get_sensor_mode(struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t mode;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        rslt = bme680_get_regs(BME680_CONF_T_P_MODE_ADDR, &mode, 1, dev);
        /* Masking the other register bit info*/
        dev->power_mode = mode & BME680_MODE_MSK;
    }
 
    return rslt;
}
 
/*!
 * @brief This API is used to set the profile duration of the sensor.
 */
void bme680_set_profile_dur(uint16_t duration, struct bme680_dev *dev)
{
    uint32_t tph_dur; /* Calculate in us */
    uint32_t meas_cycles;
    uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};
 
    meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];
 
    /* TPH measurement duration */
   tph_dur = meas_cycles * (uint32_t)(1963);
   tph_dur += (uint32_t)(477 * 4); /* TPH switching duration */
   tph_dur += (uint32_t)(477 * 5); /* Gas measurement duration */
   tph_dur += (uint32_t)(500); /* Get it to the closest whole number.*/
   tph_dur /= (uint32_t)(1000); /* Convert to ms */
 
   tph_dur += (uint32_t)(1); /* Wake up duration of 1ms */
    /* The remaining time should be used for heating */
    dev->gas_sett.heatr_dur = duration - (uint16_t) tph_dur;
}
 
/*!
 * @brief This API is used to get the profile duration of the sensor.
 */
void bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev)
{
    uint32_t tph_dur; /* Calculate in us */
    uint32_t meas_cycles;
    uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};
 
    meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];
 
    /* TPH measurement duration */
   tph_dur = meas_cycles * (uint32_t)(1963);
   tph_dur += (uint32_t)(477 * 4); /* TPH switching duration */
   tph_dur += (uint32_t)(477 * 5); /* Gas measurement duration */
   tph_dur += (uint32_t)(500); /* Get it to the closest whole number.*/
   tph_dur /= (uint32_t)(1000); /* Convert to ms */
 
   tph_dur += (uint32_t)(1); /* Wake up duration of 1ms */
 
    *duration = (uint16_t) tph_dur;
 
    /* Get the gas duration only when the run gas is enabled */
    if (dev->gas_sett.run_gas) {
        /* The remaining time should be used for heating */
        *duration += dev->gas_sett.heatr_dur;
    }
}
 
/*!
 * @brief This API reads the pressure, temperature and humidity and gas data
 * from the sensor, compensates the data and store it in the bme680_data
 * structure instance passed by the user.
 */
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
    int8_t rslt = BME680_OK;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) 
    {
        /* Reading the sensor data in forced mode only */
        rslt = read_field_data(data, dev);
        if (rslt == BME680_OK) {
            if (data->status & BME680_NEW_DATA_MSK)
                dev->new_fields = 1;
            else
                dev->new_fields = 0;
        } 
    }
 
    return rslt;
}
 
/*!
 * @brief This internal API is used to read the calibrated data from the sensor.
 */
int8_t get_calib_data(struct bme680_dev *dev)
{
   uint32_t    _T1,_T2,_T3;
   uint32_t    _P1,_P2,_P4,_P8,_P9,_P5,_P10;
   int32_t     _P3,_P6,_P7;       
   
    int8_t  rslt = BME680_OK;
    int8_t  temp8;
    int16   temp16;
    uint8   u_temp8;
    uint16  u_temp16;
    uint8   temp_mem_page;
    uint8_t temp_var = 0;     /* Temporary variable */
    
   if( dev->intf != BME680_I2C_INTF )       
   {
      //Check for mem page 0
      temp_mem_page = Read_Byte(BME680_SPI_STATUS);  
      temp_mem_page >>= 4;
      temp_mem_page &= 0x1;

      if( temp_mem_page != BME680_MEM_PAGE0 )
      {
         //Set mem page to 0
         Write_Byte( BME680_SPI_STATUS, BME680_MEM_PAGE0 );  
         
         //Check the mem page
         temp_mem_page = Read_Byte(BME680_SPI_STATUS);      
   
         if( temp_mem_page == BME680_MEM_PAGE0 )
            dev->mem_page = BME680_MEM_PAGE0;
      }
   }

   if (rslt == BME680_OK)
   {
    
#ifndef ADUINO_CODE
   /**************************************/
   /* Set up Calib Temperature data t1-t3*/
   /**************************************/
   _T1 = ( Read_Byte(0xEA) << 8 );
   _T1 |= Read_Byte(0xE9);
   dev->calib.par_t1 = _T1;

   _T2 = ( Read_Byte(0x8B) << 8 );
   _T2 |= Read_Byte(0x8A);
   dev->calib.par_t2 = _T2;
      
   _T3 = Read_Byte(0x8C);
   dev->calib.par_t3 = _T3;
   
   /************************************/
   /* Set up Calib Pressure data p1-p10*/
   /************************************/
      
   _P1 = ( Read_Byte(0x8F) << 8 );
   _P1 |= Read_Byte(0x8E);
   //_P1 = 0xFFFF0000 | _P1;
   dev->calib.par_p1 = _P1;

   _P2 = ( Read_Byte(0x91) << 8 );
   _P2 |= Read_Byte(0x90);
   _P2 = 0xFFFF0000 | _P2;
   dev->calib.par_p2 = _P2;
   
   _P3 = Read_Byte(0x92);
   dev->calib.par_p3 = _P3;

   _P4 = ( Read_Byte(0x95) << 8 );
   _P4 |= Read_Byte(0x94);
   //_P4 = 0xFFFF0000 | _P4;
   dev->calib.par_p4 = _P4;

   _P5 = ( Read_Byte(0x97) << 8 );
   _P5 |=  Read_Byte(0x96);
   _P5 = 0xFFFF0000 | _P5;
   dev->calib.par_p5 = _P5;

   
   _P6 = Read_Byte(0x99);
   dev->calib.par_p6 = _P6;

   
   _P7 = Read_Byte(0x98);
   dev->calib.par_p7 = _P7;

   _P8 = ( Read_Byte(0x9D) << 8 );
   _P8 |= Read_Byte(0x9C);
   _P8 = 0xFFFF0000 | _P8;
   dev->calib.par_p8 = _P8;

   _P9 = ( Read_Byte(0x9F) << 8 );
   _P9 |= Read_Byte(0x9E);
   _P9 = 0xFFFF0000 | _P9;
   dev->calib.par_p9 = _P9;

   _P10 = Read_Byte(0xA0);
   _P10 = 0xFFFF0000 | _P10;
   dev->calib.par_p10 = _P10;

#ifdef DEBUG
   fprintf(STDOUT, "**********   _T1 = %Lu\r\n\r\n", _T1);
   fprintf(STDOUT, "**********   _T2 = %Lu\r\n\r\n", _T2);
   fprintf(STDOUT, "**********   _T3 = %Lu\r\n\r\n", _T3);
   fprintf(STDOUT, "**********   _P1 = %Lu\r\n\r\n", _P1);
   fprintf(STDOUT, "**********   _P2 = %Lu\r\n\r\n", _P2);
   fprintf(STDOUT, "**********   _P3 = %Lu\r\n\r\n", _P3);
   fprintf(STDOUT, "**********   _P4 = %Lu\r\n\r\n", _P4);
   fprintf(STDOUT, "**********   _P5 = %Lu\r\n\r\n", _P5);
   fprintf(STDOUT, "**********   _P6 = %Lu\r\n\r\n", _P6);
   fprintf(STDOUT, "**********   _P7 = %Lu\r\n\r\n", _P7);
   fprintf(STDOUT, "**********   _P8 = %Lu\r\n\r\n", _P8);
   fprintf(STDOUT, "**********   _P9 = %Lu\r\n\r\n", _P9);
   fprintf(STDOUT, "**********   _P10 = %Lu\r\n\r\n", _P10);
#endif


#else   

    uint8_t coeff_array[BME680_COEFF_SIZE] = { 0 };
    uint8_t temp_var = 0; /* Temporary variable */
 
    /* Check for null pointer in the device structure*/
    //Null pointer should be check before getting to this point
    //    rslt = null_ptr_check(dev);

    rslt = bme680_get_regs(BME680_COEFF_ADDR1, coeff_array, BME680_COEFF_ADDR1_LEN, dev);
        /* Append the second half in the same array */
        if (rslt == BME680_OK)
            rslt = bme680_get_regs(BME680_COEFF_ADDR2, &coeff_array[BME680_COEFF_ADDR1_LEN]
                                   , BME680_COEFF_ADDR2_LEN, dev);
 
        /* Temperature related coefficients */
        dev->calib.par_t1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],
                                        coeff_array[BME680_T1_LSB_REG]));
        dev->calib.par_t2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG],
                                       coeff_array[BME680_T2_LSB_REG]));
        dev->calib.par_t3 = (int8_t) (coeff_array[BME680_T3_REG]);
 
        /* Pressure related coefficients */
        dev->calib.par_p1 = (uint16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG],
                                        coeff_array[BME680_P1_LSB_REG]));
        dev->calib.par_p2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],
                                       coeff_array[BME680_P2_LSB_REG]));
        dev->calib.par_p3 = (int8_t) coeff_array[BME680_P3_REG];
        dev->calib.par_p4 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG],
                                       coeff_array[BME680_P4_LSB_REG]));
        dev->calib.par_p5 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],
                                       coeff_array[BME680_P5_LSB_REG]));
        dev->calib.par_p6 = (int8_t) (coeff_array[BME680_P6_REG]);
        dev->calib.par_p7 = (int8_t) (coeff_array[BME680_P7_REG]);
        dev->calib.par_p8 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],
                                       coeff_array[BME680_P8_LSB_REG]));
        dev->calib.par_p9 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],
                                       coeff_array[BME680_P9_LSB_REG]));
        dev->calib.par_p10 = (uint8_t) (coeff_array[BME680_P10_REG]);
 
        /* Humidity related coefficients */
        dev->calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
                                        | (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
        dev->calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL)
                                        | ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
        dev->calib.par_h3 = (int8_t) coeff_array[BME680_H3_REG];
        dev->calib.par_h4 = (int8_t) coeff_array[BME680_H4_REG];
        dev->calib.par_h5 = (int8_t) coeff_array[BME680_H5_REG];
        dev->calib.par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
        dev->calib.par_h7 = (int8_t) coeff_array[BME680_H7_REG];
 
        /* Gas heater related coefficients */
        dev->calib.par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
        dev->calib.par_gh2 = (int16_t) (BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG],
                                        coeff_array[BME680_GH2_LSB_REG]));
        dev->calib.par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];

#endif        
         /* Other coefficients */
      if (rslt == BME680_OK) 
      {
         rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1, dev);
         dev->calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
   
         if (rslt == BME680_OK)
         {
            rslt = bme680_get_regs(BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1, dev);
            dev->calib.res_heat_val = (int8_t) temp_var;
         
            if (rslt == BME680_OK)
               rslt = bme680_get_regs(BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1, dev);
         }
      }
        
      dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;

   }
 
   return rslt;
}
 
/*!
 * @brief This internal API is used to set the gas configuration of the sensor.
 */
int8_t set_gas_config(struct bme680_dev *dev)
{
    int8_t rslt;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
 
        uint8_t reg_addr[2] = {0};
        uint8_t reg_data[2] = {0};
 
        if (dev->power_mode == BME680_FORCED_MODE) {
            reg_addr[0] = BME680_RES_HEAT0_ADDR;
            reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);
            reg_addr[1] = BME680_GAS_WAIT0_ADDR;
            reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);
            dev->gas_sett.nb_conv = 0;
        } else {
            rslt = BME680_W_DEFINE_PWR_MODE;
        }
        if (rslt == BME680_OK)
            rslt = bme680_set_regs(reg_addr, reg_data, 2, dev);
    }
 
    return rslt;
}
 
/*!
 * @brief This internal API is used to get the gas configuration of the sensor.
 * @note heatr_temp and heatr_dur values are currently register data
 * and not the actual values set
 */
int8_t get_gas_config(struct bme680_dev *dev)
{
    int8_t rslt;
    /* starting address of the register array for burst read*/
    uint8_t reg_addr1 = BME680_ADDR_SENS_CONF_START;
    uint8_t reg_addr2 = BME680_ADDR_GAS_CONF_START;
    uint8_t reg_data = 0;
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    if (rslt == BME680_OK) {
        if (BME680_SPI_INTF == dev->intf) {
            /* Memory page switch the SPI address*/
            rslt = set_mem_page(reg_addr1, dev);
        }
 
        if (rslt == BME680_OK) {
            rslt = bme680_get_regs(reg_addr1, &reg_data, 1, dev);
            if (rslt == BME680_OK) {
                dev->gas_sett.heatr_temp = reg_data;
                rslt = bme680_get_regs(reg_addr2, &reg_data, 1, dev);
                if (rslt == BME680_OK) {
                    /* Heating duration register value */
                    dev->gas_sett.heatr_dur = reg_data;
                }
            }
        }
    }
 
    return rslt;
}
 
#ifndef BME680_FLOAT_POINT_COMPENSATION
 

 
/*!
 * @brief This internal API is used to calculate the pressure value.
 */
uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t pressure_comp;
 
    var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)dev->calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)dev->calib.par_p3 << 5)) >> 3) + (((int32_t)dev->calib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
    pressure_comp = 1048576 - pres_adc;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    
    if (pressure_comp >= BME680_MAX_OVERFLOW_VAL)
        pressure_comp = ((pressure_comp / var1) << 1);
    
    else
        pressure_comp = ((pressure_comp << 1) / var1);
    
    var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) * (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) * (int32_t)dev->calib.par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) * (int32_t)dev->calib.par_p10) >> 17;
 
    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 + ((int32_t)dev->calib.par_p7 << 7)) >> 4);
 
    return (uint32_t)pressure_comp;
 
}
 
/*!
 * @brief This internal API is used to calculate the humidity value.
 */
uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t calc_hum;
 
    temp_scaled = (((int32_t) dev->calib.t_fine * 5) + 128) >> 8;
    var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev->calib.par_h1 * 16)))
           - (((temp_scaled * (int32_t) dev->calib.par_h3) / ((int32_t) 100)) >> 1);
    var2 = ((int32_t) dev->calib.par_h2
            * (((temp_scaled * (int32_t) dev->calib.par_h4) / ((int32_t) 100))
               + (((temp_scaled * ((temp_scaled * (int32_t) dev->calib.par_h5) / ((int32_t) 100))) >> 6)
                  / ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = (int32_t) dev->calib.par_h6 << 7;
    var4 = ((var4) + ((temp_scaled * (int32_t) dev->calib.par_h7) / ((int32_t) 100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;
 
    if (calc_hum > 100000) /* Cap at 100%rH */
        calc_hum = 100000;
    else if (calc_hum < 0)
        calc_hum = 0;
 
    return (uint32_t) calc_hum;
}
 
/*!
 * @brief This internal API is used to calculate the Gas Resistance value.
 */
uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
    int32_t var1;
    uint32_t var2;
    int32_t var3;
    uint32_t calc_gas_res;
    /**Look up table 1 for the possible gas range values */
   uint32_t lookupTable1[16] = { (uint32_t)(2147483647), (uint32_t)(2147483647), (uint32_t)(2147483647), (uint32_t)(2147483647),
      (uint32_t)(2147483647), (uint32_t)(2126008810), (uint32_t)(2147483647), (uint32_t)(2130303777),
      (uint32_t)(2147483647), (uint32_t)(2147483647), (uint32_t)(2143188679), (uint32_t)(2136746228),
      (uint32_t)(2147483647), (uint32_t)(2126008810), (uint32_t)(2147483647), (uint32_t)(2147483647) };
    /**Look up table 2 for the possible gas range values */
   uint32_t lookupTable2[16] = { (uint32_t)(4096000000), (uint32_t)(2048000000), (uint32_t)(1024000000), (uint32_t)(512000000),
      (uint32_t)(255744255), (uint32_t)(127110228), (uint32_t)(64000000), (uint32_t)(32258064), (uint32_t)(16016016),
      (uint32_t)(8000000), (uint32_t)(4000000), (uint32_t)(2000000), (uint32_t)(1000000), (uint32_t)(500000),
      (uint32_t)(250000), (uint32_t)(125000) };
 
    var1 = (int32_t) ((1340 + (5 * (int32_t) dev->calib.range_sw_err)) *
                      ((int32_t) lookupTable1[gas_range])) >> 16;
    var2 = (((int32_t) ((int32_t) gas_res_adc << 15) - (int32_t) (16777216)) + var1);
    var3 = (((int32_t) lookupTable2[gas_range] * (int32_t) var1) >> 9);
    calc_gas_res = (uint32_t) ((var3 + ((int32_t) var2 >> 1)) / (int32_t) var2);
 
    return calc_gas_res;
}
 
/*!
 * @brief This internal API is used to calculate the Heat Resistance value.
 */
uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
    uint8_t heatr_res;
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;
 
    if (temp > 400) /* Cap temperature */
        temp = 400;
 
    var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
    var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (dev->calib.res_heat_range + 4));
    var5 = (131 * dev->calib.res_heat_val) + 65536;
    heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);
 
    return heatr_res;
}
 
#else
 
 
/*!
 * @brief This internal API is used to calculate the
 * temperature value in float format
 */
float calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
    float var1 = 0;
    float var2 = 0;
    float calc_temp = 0;
 
    /* calculate var1 data */
    var1  = ((((float)temp_adc / 16384.0f) - ((float)dev->calib.par_t1 / 1024.0f))
             * ((float)dev->calib.par_t2));
 
    /* calculate var2 data */
    var2  = (((((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f)) *
              (((float)temp_adc / 131072.0f) - ((float)dev->calib.par_t1 / 8192.0f))) *
             ((float)dev->calib.par_t3 * 16.0f));
 
    /* t_fine value*/
    dev->calib.t_fine = (var1 + var2);
 
    /* compensated temperature data*/
    calc_temp  = ((dev->calib.t_fine) / 5120.0f);
 
    return calc_temp;
}
 
/*!
 * @brief This internal API is used to calculate the
 * pressure value in float format
 */
float calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
    float calc_pres = 0;
 
    var1 = (((float)dev->calib.t_fine / 2.0f) - 64000.0f);
    var2 = var1 * var1 * (((float)dev->calib.par_p6) / (131072.0f));
    var2 = var2 + (var1 * ((float)dev->calib.par_p5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)dev->calib.par_p4) * 65536.0f);
    var1 = (((((float)dev->calib.par_p3 * var1 * var1) / 16384.0f)
             + ((float)dev->calib.par_p2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)dev->calib.par_p1));
    calc_pres = (1048576.0f - ((float)pres_adc));
 
    /* Avoid exception caused by division by zero */
    if ((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)dev->calib.par_p9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)dev->calib.par_p8) / 32768.0f);
        var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f)
                * (dev->calib.par_p10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)dev->calib.par_p7 * 128.0f)) / 16.0f);
    } else {
        calc_pres = 0;
    }
 
    return calc_pres;
}
 
/*!
 * @brief This internal API is used to calculate the
 * humidity value in float format
 */
float calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
    float calc_hum = 0;
    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
    float var4 = 0;
    float temp_comp;
 
    /* compensated temperature data*/
    temp_comp  = ((dev->calib.t_fine) / 5120.0f);
 
    var1 = (float)((float)hum_adc) - (((float)dev->calib.par_h1 * 16.0f) + (((float)dev->calib.par_h3 / 2.0f)
                                      * temp_comp));
 
    var2 = var1 * ((float)(((float) dev->calib.par_h2 / 262144.0f) * (1.0f + (((float)dev->calib.par_h4 / 16384.0f)
                           * temp_comp) + (((float)dev->calib.par_h5 / 1048576.0f) * temp_comp * temp_comp))));
 
    var3 = (float) dev->calib.par_h6 / 16384.0f;
 
    var4 = (float) dev->calib.par_h7 / 2097152.0f;
 
    calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
 
    if (calc_hum > 100.0f)
        calc_hum = 100.0f;
    else if (calc_hum < 0.0f)
        calc_hum = 0.0f;
 
    return calc_hum;
}
 
/*!
 * @brief This internal API is used to calculate the
 * gas resistance value in float format
 */
static float calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
    float calc_gas_res;
    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
 
    const float lookup_k1_range[16] = {
        0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8,
   0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
    const float lookup_k2_range[16] = {
        0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8,
   -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
 
    var1 = (1340.0f + (5.0f * dev->calib.range_sw_err));
    var2 = (var1) * (1.0f + lookup_k1_range[gas_range]/100.0f);
    var3 = 1.0f + (lookup_k2_range[gas_range]/100.0f);
 
    calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * (float)(1 << gas_range) * (((((float)gas_res_adc)
                                  - 512.0f)/var2) + 1.0f));
 
    return calc_gas_res;
}
 
/*!
 * @brief This internal API is used to calculate the
 * heater resistance value in float format
 */
float calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
    float var1 = 0;
    float var2 = 0;
    float var3 = 0;
    float var4 = 0;
    float var5 = 0;
    float res_heat = 0;
 
    if (temp > 400) /* Cap temperature */
        temp = 400;
 
    var1 = (((float)dev->calib.par_gh1 / (16.0f)) + 49.0f);
    var2 = ((((float)dev->calib.par_gh2 / (32768.0f)) * (0.0005f)) + 0.00235f);
    var3 = ((float)dev->calib.par_gh3 / (1024.0f));
    var4 = (var1 * (1.0f + (var2 * (float)temp)));
    var5 = (var4 + (var3 * (float)dev->amb_temp));
    res_heat = (uint8_t)(3.4f * ((var5 * (4 / (4 + (float)dev->calib.res_heat_range)) *
                                  (1/(1 + ((float) dev->calib.res_heat_val * 0.002f)))) - 25));
 
    return res_heat;
}
 
#endif
 
/*!
 * @brief This internal API is used to calculate the Heat duration value.
 */
uint8_t calc_heater_dur(uint16_t dur)
{
    uint8_t factor = 0;
    uint8_t durval;
 
    if (dur >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dur > 0x3F) {
            dur = dur / 4;
            factor += 1;
        }
        durval = (uint8_t) (dur + (factor * 64));
    }
 
    return durval;
}
 
/*****************************************************************************/
/*!
 * @brief This internal API is used to calculate the field data of sensor.
 */
/*****************************************************************************/ 
int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
    int8_t rslt;
    uint8_t buff[BME680_FIELD_LENGTH] = { 0 };
    uint8_t gas_range;
    uint32_t adc_temp;
    uint32_t adc_pres;
    uint16_t adc_hum;
    uint16_t adc_gas_res;
    uint8_t tries = 10;
    uint32_t tmp;
    
    
 
    /* Check for null pointer in the device structure*/
    rslt = null_ptr_check(dev);
    do {
        if (rslt == BME680_OK) {
            rslt = bme680_get_regs(((uint8_t) (BME680_FIELD0_ADDR)), buff, (uint16_t) BME680_FIELD_LENGTH,
                                   dev);
 
            data->status = buff[0] & BME680_NEW_DATA_MSK;
            data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
            data->meas_index = buff[1];
 
            /* read the raw data from the sensor */
            adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16)
                                   | ((uint32_t) buff[4] / 16));
            fprintf(STDOUT, "ADC pressure - %Lu \r\n", adc_pres);
 
            adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16)
                                   | ((uint32_t) buff[7] / 16));
            fprintf(STDOUT, "ADC temperature - %Lu \r\n", adc_temp);                                   
                                   
            adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
            fprintf(STDOUT, "ADC humidity - %Lu \r\n", adc_hum);
            
            adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
            fprintf(STDOUT, "ADC gas resolution  - %Lu \r\n", adc_gas_res);
            
            gas_range = buff[14] & BME680_GAS_RANGE_MSK;
 
            data->status |= buff[14] & BME680_GASM_VALID_MSK;
            data->status |= buff[14] & BME680_HEAT_STAB_MSK;
            if (data->status & BME680_NEW_DATA_MSK) 
            {
                data->temperature = calc_temperature(adc_temp, dev);
                tmp = data->temperature;
                fprintf(STDOUT, "Temperature: %Lu \r\n", tmp);
                
                data->pressure = calc_pressure(adc_pres, dev);
                tmp = data->pressure;
                fprintf(STDOUT, "Pressure: %Lu  \r\n", tmp);
                
                data->humidity = calc_humidity(adc_hum, dev);
                tmp = data->humidity;
                fprintf(STDOUT, "Humidity: %Lu \r\n", tmp);
                
                data->gas_resistance = calc_gas_resistance(adc_gas_res, gas_range, dev);
                tmp = data->gas_resistance;
                fprintf(STDOUT, "Gas resistance : %Lu \r\n", tmp);

                //break;
            }
            
            /* Delay to poll the data */
            delay_ms(BME680_POLL_PERIOD_MS);
            
       }
        tries--;
    } while (tries);

     
    if (!tries)
        rslt = BME680_W_NO_NEW_DATA;
 
    return rslt;
}
 

 
/*!
 * @brief This internal API is used to validate the boundary
 * conditions.
 */
int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev)
{
    int8_t rslt = BME680_OK;
 
    if (value != NULL) {
        /* Check if value is below minimum value */
        if (*value < min) {
            /* Auto correct the invalid value to minimum value */
            *value = min;
            dev->info_msg |= BME680_I_MIN_CORRECTION;
        }
        /* Check if value is above maximum value */
        if (*value > max) {
            /* Auto correct the invalid value to maximum value */
            *value = max;
            dev->info_msg |= BME680_I_MAX_CORRECTION;
        }
    } else {
        rslt = BME680_E_NULL_PTR;
    }
 
    return rslt;
}
 
#endif   // BME_CODE

///////////////////////////////////////////////////////////////////////////////
