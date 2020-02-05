
////////////////////////////////////////

#include <main.h>

////////////////////////////////////////


#include <.\Drivers\stdlib.h>
#include <.\Drivers\math.h>
#include <.\Drivers\stdio.h>
#include <.\Drivers\string.h>
#include <.\Drivers\ieeefloat.c>

////////////////////////////////////////

#define STREAM_SERIAL_INPUT STDOUT
#include <.\Drivers\input.c>

////////////////////////////////////////

typedef unsigned int8 uint8;
typedef unsigned int16 uint16;
typedef unsigned int32 uint32;

////////////////////////////////////////

#define TURNOFF_AUTO_POWERDOWN
//#define SOFTWARE_DEBUG
#define BYPASS_BREATH_COLLECTION
//#define SKIP_WETCARTRIDGE_CHECK      
//#define HIGH_SCORE_TEST
//#define SHORT_TIME
//#define TURN_OFF_BRIGHTNESS_CHECK  


#define EEPROM_SDA PIN_C4
#define EEPROM_SCL PIN_C3

extern uint8 iosApp;
int8 TestMode = 0;

//#include <rtc.h>

////////////////////////////////////////
void ShowPrompt() {
   fprintf(STDOUT, "[I50 V%lu.%lu.%lu",
     FIRMWARE_VERSION_MAJOR,
     FIRMWARE_VERSION_MINOR,
     FIRMWARE_VERSION_REVISION);
//   PrintRtcTime();
   fprintf(STDOUT, "] ");
}

#include <24256-custom.c>
#include <DS3231.c>
#include <config.c>
#include <ble.c>
#include <reading.c>
#include <peripherals.c>
#include <util.c>
#include <bme680.c>
#include <battery.c>
#include "datatype.h"
#include "bme680_defs.h"
#include "ErrorCodes.h"
#include "Events.c"
#include "reading.h"

////////////////////////////////////////

static uint16 myStatus;
//static struct BreathTest myTestResult;

static struct bme680_dev gas_sensor;


static struct commandPara     cmdReceived;
static struct testResultPara  testResult;
static struct statusPara      testTime;
static struct deviceInfoPara  deviceStat;
static struct testInfoPara    testStat;

uint8   cartridgeDetected;
uint8   cartridgeState;
uint8   brightnessChecked;
uint8   calibrationStatus;
uint8   ledBrightnessCalibrated;

uint8   bluetoothState;
uint16  breathVolume;
uint16  powerDownTimeCtr;


//uint32  timestamp = 0X0;
//char    string_resp[50];

//uint8   stepSequences[MAXIMUM_NUMBER_OF_RECORDS];
//uint8   errorCode[MAXIMUM_NUMBER_OF_RECORDS];




////////////////////////////////////////

struct LightProperties {
   uint8 LevelBrightness730;
   uint8 LevelBrightness588;
   uint8 LevelBrightness475;
   
   uint16 DarkReading730;
   uint16 DarkReading588;
   uint16 DarkReading475;
};

struct LightMeasurements {
   float32 Light730;
   float32 Light588;
   float32 Light475;
};

struct LedReadings {
   uint16 Raw730;
   uint16 Raw588;
   uint16 Raw475;
};

////////////////////////////////////////
void initBME ( void )
{
#ifdef BME_SPI_MODE
   uint8    *addrPtr = NULL;
   uint8    tmpValue = 0x0;
   gas_sensor.intf = BME680_SPI_INTF;
   
#else 
   gas_sensor.intf = BME680_I2C_INTF;
   
#endif

   //Init gas sensor structure
   gas_sensor.write = Write;
   gas_sensor.read = Read;
   gas_sensor.delay_ms = NULL;

#ifdef USE_BME_SPI_MODE
   //Clear Port C
   addrPtr = PORTC;
   *addrPtr = 0x0;
   
   //Clear data latches
   addrPtr = LATC;
   *addrPtr = 0x0;
   
   //Init PORTC - C3 & C4 Direction (TRISC)
   tmpValue = get_tris_c();
   
   //Set C3 to output and C4 to input 
   //tmpValue &= 0xF7;       //b1111 0111
   tmpValue &= 0xE7;         //b1110 0111
   set_tris_c(tmpValue);
   
   //Set C3 and C4 to digital I/O
   //C3 and C4 located on bit 4 and 5 respectively.  0 - digital, 1 - analog
   addrPtr = ANSELC;
   tmpValue = *addrPtr;
   tmpValue &= 0xE4;       //b1110 0100
   *addrPtr = tmpValue;
   
   addrPtr = NULL;
#endif

}
////////////////////////////////////////////////////////////////////////////////
void shutDown ()
{
   DisableBleModule();
   
   fprintf(STDOUT, "\r\nShutdown external devices...\r\n");
   output_low(PIN_C1);
   output_low(PIN_C2);
   
   fprintf(STDOUT, "\r\nShutdown processor...\r\n");
   sleep(SLEEP_FULL);
}

////////////////////////////////////////////////////////////////////////////////
void hardwareFailure()
{
   uint32   timer_current = 0;
   uint8_t  tmpCtr = 0;
   
   while( myStatus )
   {
      illumination_brightness( 90 );
      illumination_use(ILLUMINATION_730);
      ble_wait(100);
      output_low(ILLUMINATION_730);
      ble_wait(50);
           
      play_error();
      delay_ms(100);

      timer_current = get_ticks();
      if( timer_current >= TEN_SECONDS )
      {
         tmpCtr++;
         set_ticks(0x0);
         fprintf(STDOUT, "Hardware failure code %lu\r\n", myStatus);
      }
      
      if( tmpCtr >= 3 )
      {
         fprintf(STDOUT, "Hardware failure, unit is shutting down\r\n");
         shutDown();
      }
   }
}

////////////////////////////////////////
struct LightProperties LedProperties;
struct LedReadings CartridgeRawDataSet1[9];
struct LedReadings CartridgeRawDataSet2[7];
struct LightMeasurements whitebalances;

////////////////////////////////////////
struct LedReadings MeasureLEDs() {
   struct LedReadings x;

   illumination_brightness(LedProperties.LevelBrightness730);
   illumination_use(ILLUMINATION_730);
   ble_wait(45);
   x.Raw730 = photodiode();
   //illumination_all_off();
   ble_wait(5);
   
   illumination_brightness(LedProperties.LevelBrightness588);
   illumination_use(ILLUMINATION_588);
   ble_wait(45);
   x.Raw588 = photodiode();
   //illumination_all_off();
   ble_wait(5);
   
   illumination_brightness(LedProperties.LevelBrightness475);
   illumination_use(ILLUMINATION_475);
   ble_wait(45);
   x.Raw475 = photodiode();
   ble_wait(5);  

   illumination_all_off();
   return x;
}

////////////////////////////////////////
#ifdef BASE_40
uint8 check_LEDs_below(uint16 value) 
{
   struct LedReadings x;
   
   x = MeasureLEDs();
   
   fprintf(STDOUT, "LEDs Check (<%lu):\r\n730nm, 588nm, 475nm\r%lu, %lu, %lu\r\n", value, x.Raw730, x.Raw588, x.Raw475);
   if (TestMode == 0) 
   {
      if (x.Raw730 < value || x.Raw588 < value || x.Raw475 < value) 
         return 1;
      
      else 
         return 0;
   }
   
   else 
      return 0; // TEST MODE
}
#endif
////////////////////////////////////////
uint8 check_wet_cartridge( ) 
{
   uint16 Raw588;
   
   illumination_all_off();
   ble_wait(5);

#ifdef DEBUG
   fprintf(STDOUT, cartridge_wetting_check_resp, new_line_resp);
#endif

   illumination_brightness(LedProperties.LevelBrightness588);
   illumination_use(ILLUMINATION_588);
   ble_wait(45);
   Raw588 = photodiode();
   ble_wait(5); 
   output_low(ILLUMINATION_588);

#ifdef DEBUG
   fprintf(STDOUT, bright588_resp);
   fprintf(STDOUT, colon_resp);
   fprintf(STDOUT, "%lu", Raw588);
   fprintf(STDOUT, new_line_resp);
#endif

   if (TestMode == 0)
   {
      //if ( Raw588 < JUST_WET_CARTRIDGE_MAX_588 && Raw588 > JUST_WET_CARTRIDGE_MIN_588 ) 
      if ( Raw588 < JUST_WET_CARTRIDGE_MAX_588 && Raw588 > 75 ) 
      {
         fprintf(STDOUT, cartridge_wetting_check_resp);
         fprintf(STDOUT, dash_resp);
         fprintf(STDOUT, wet_resp);
         fprintf(STDOUT, two_new_lines_resp);
         return YES;
      }
      
      else 
      {
         fprintf(STDOUT, cartridge_wetting_check_resp);
         fprintf(STDOUT, dash_resp);
         fprintf(STDOUT, not_resp);
         fprintf(STDOUT, wet_resp);
         fprintf(STDOUT, two_new_lines_resp);
         return NO;
      }
   }
   
   else 
   {
//      fprintf(STDOUT, "Cartridge Wetting Check: skipped test mode");
      return 0; // TEST MODE overwriting
   }
   
   illumination_all_off();

}

////////////////////////////////////////
void get_LED_Brightness(void )
{
   ledBrightnessCalibrated = ConfigReadByte(CONFIG_LED_BRIGHTNESS_CALIBRATED);
   
   if( ledBrightnessCalibrated != UNIT_CALIBRATED )
      fprintf(STDOUT, "Brightness not calibrated - 0x%x\r\n", ledBrightnessCalibrated);
   
   fprintf(STDOUT, bright730_resp);
   fprintf(STDOUT, colon_resp);
   fprintf(STDOUT, "%u", ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS));
   fprintf(STDOUT, new_line_resp);
   

   fprintf(STDOUT, bright588_resp);
   fprintf(STDOUT, colon_resp);
   fprintf(STDOUT, "%u", ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS));
   fprintf(STDOUT, new_line_resp);


   fprintf(STDOUT, bright475_resp);
   fprintf(STDOUT, colon_resp);
   fprintf(STDOUT, "%u", ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS));
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
int autoSetLEDBrightness(int16 bt730, int16 bt588, int16 bt475)
{
   uint8 status = FAIL;
   uint8 ctr = 0;
   int16 Led730, Led588, Led475;
   int16 tmpLed730, tmpLed588, tmpLed475;                 
   
   Led730 = Led588 = Led475 = 0;
   status = FAIL;
   
   ledBrightnessCalibrated = ConfigReadByte(CONFIG_LED_BRIGHTNESS_CALIBRATED);
//   fprintf(STDOUT, "Brightness calibration stat = %x\r\n", ledBrightnessCalibrated);
   
   if( ledBrightnessCalibrated == UNIT_CALIBRATED )
   {
      tmpLed730 = ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS);
      tmpLed588 = ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS);
      tmpLed475 = ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS);
      
      if( tmpLed730 < LED_BRIGHTNESS_MINIMUM_SETTING )
         tmpLed730 = bt730;
         
      if( tmpLed588 < LED_BRIGHTNESS_MINIMUM_SETTING )
         tmpLed588 = bt588;         
         
      if( tmpLed475 < LED_BRIGHTNESS_MINIMUM_SETTING )
         tmpLed475 = bt475;
   }
   
   else     //First time setting LED brightness
   {
      tmpLed730 = bt730;                 
      tmpLed588 = bt588;              
      tmpLed475 = bt475;
   }
                
   //Get the LED brightness readinng
   illumination_brightness(tmpLed730);
   illumination_use(ILLUMINATION_730);
   ble_wait(45);
   Led730 = photodiode();
   ble_wait(5);
   output_low(ILLUMINATION_730);
   
   illumination_brightness(tmpLed588);
   illumination_use(ILLUMINATION_588);
   ble_wait(45);
   Led588 = photodiode();
   ble_wait(5);
   output_low(ILLUMINATION_588);
   
   illumination_brightness(tmpLed475);
   illumination_use(ILLUMINATION_475);
   ble_wait(45);
   Led475 = photodiode();
   ble_wait(5);
   output_low(ILLUMINATION_475);

   delay_ms(2850);
                  
//   if( Led588 < NEW_CARTRIDGE_MIN && tmpLed588 > 75 )
   if( Led730 < NEW_CARTRIDGE_MIN_730 && tmpLed730 > NO_CARTRIDGE )
   { 
      cartridgeState = CARTRIDGE_STATE_USED;
      bleNotifyErrorSet(BAD_CATRIDGE);
      fprintf(STDOUT, bad_resp);
      fprintf(STDOUT, cartridge_resp);
      fprintf(STDOUT, detected_resp);
      fprintf(STDOUT, insert_new_cartridge_resp);
      fprintf(STDOUT, two_new_lines_resp);

      return status;
   }
                  
   while( status != PASS )
   {
      fprintf(STDOUT, bright730_resp);
      fprintf(STDOUT, dash_resp);   
      fprintf(STDOUT, "%ld @ %ld", Led730, tmpLed730);
      fprintf(STDOUT, new_line_resp);   
   
      fprintf(STDOUT, bright588_resp);
      fprintf(STDOUT, dash_resp);   
      fprintf(STDOUT, "%ld @ %ld", Led588, tmpLed588);                     
      fprintf(STDOUT, new_line_resp);   
      
      fprintf(STDOUT, bright475_resp);
      fprintf(STDOUT, dash_resp);   
      fprintf(STDOUT, "%ld @ %ld", Led475, tmpLed475);
      fprintf(STDOUT, new_line_resp);   
      fprintf(STDOUT, new_line_resp);   

      //Check for cartridge
      if( Led730 < NO_CARTRIDGE || Led588 < NO_CARTRIDGE || Led475 < NO_CARTRIDGE )
      {
         bleNotifyErrorSet(BAD_CATRIDGE);
         fprintf(STDOUT, "%lu, %lu, %lu", Led730, Led588, Led475);
         fprintf(STDOUT, two_new_lines_resp);
         fprintf(STDOUT, insert_new_cartridge_resp);
         fprintf(STDOUT, new_line_resp);
         return FAIL;                        
      }

      if( tmpLed730 >= LED_BRIGHTNESS_MAXIMUM_SETTING || tmpLed588 >= LED_BRIGHTNESS_MAXIMUM_SETTING || tmpLed475 >= LED_BRIGHTNESS_MAXIMUM_SETTING )                    
      {
         fprintf(STDOUT, new_line_resp);
         fprintf(STDOUT, max_power_setting_exceeded_resp);
         fprintf(STDOUT, new_line_resp);
         
         bleNotifyErrorSet(HARDWARE_FAIL);
         return FAIL;                        
      }
                    
      //Adjust the LED brightness
      if( Led730 <= 590 || Led730 >= 610 )
      {
         int16 tmp_led730 = 600 - Led730;
                      
         if( tmp_led730 < 20  && tmp_led730 > 5 )
            tmp_led730 = 1;
                        
         else if ( tmp_led730 > -20 && tmp_led730 < -5 )
            tmp_led730 = -1;
                           
         else  
            tmp_led730 /= 20;
                           
         tmpLed730 += tmp_led730;
      }
                     
      if( Led588 <= 610 || Led588 >= 630 )
      {
         int16 tmp_led588 = 620 - Led588;
                        
         if( tmp_led588 < 20  && tmp_led588 > 5 )
            tmp_led588 = 1;
                       
         else if ( tmp_led588 > -20 && tmp_led588 < -5 )
            tmp_led588 = -1;
                           
         else  
            tmp_led588 /= 20;
                        
         tmpLed588 += tmp_led588;
      }
                  
      if( Led475 <= 690 || Led475 >= 710 )
      {
         int16 tmp_led475 = 700 - Led475;
                     
         if( tmp_led475 < 20  && tmp_led475 > 5 )
            tmp_led475 = 1;
                        
         else if ( tmp_led475 > -20 && tmp_led475 < -5 )
            tmp_led475 = -1;
                           
         else  
            tmp_led475 /= 20;
                           
         tmpLed475 += tmp_led475;
      }
                      
      if( Led730 < 620 && Led730 >= 590 
          && Led588 < 640 && Led588 >= 610
          && Led475 < 720 && Led475 >= 690 )
      {
         ConfigWriteByte(CONFIG_LED475_BRIGHTNESS_ADDRESS, tmpLed475);
         LedProperties.LevelBrightness475 = ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS);
                      
         ConfigWriteByte(CONFIG_LED588_BRIGHTNESS_ADDRESS, tmpLed588);
         LedProperties.LevelBrightness588 = ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS);
                        
         ConfigWriteByte(CONFIG_LED730_BRIGHTNESS_ADDRESS, tmpLed730);
         LedProperties.LevelBrightness730 = ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS);
         
         status = PASS;
         
         ledBrightnessCalibrated = UNIT_CALIBRATED;
         ConfigWriteByte(CONFIG_LED_BRIGHTNESS_CALIBRATED, ledBrightnessCalibrated);

         if( ledBrightnessCalibrated == UNIT_CALIBRATED )
         {
            fprintf(STDOUT, brightness_resp);   
            fprintf(STDOUT, calibrated_resp);         
            fprintf(STDOUT, new_line_resp);         
         }
 
         else
         {
            fprintf(STDOUT, brightness_resp);         
            fprintf(STDOUT, not_resp);         
            fprintf(STDOUT, calibrated_resp);         
            fprintf(STDOUT, new_line_resp);         
         }
         
         get_LED_Brightness();
         fprintf(STDOUT, new_line_resp);
      }
                     
      else
      {
         //Get the LED brightness readinng
         illumination_brightness(tmpLed730);
         illumination_use(ILLUMINATION_730);
         ble_wait(45);
         Led730 = photodiode();
         ble_wait(5);
         output_low(ILLUMINATION_730);
   
         illumination_brightness(tmpLed588);
         illumination_use(ILLUMINATION_588);
         ble_wait(45);
         Led588 = photodiode();
         ble_wait(5);
         output_low(ILLUMINATION_588);
   
         illumination_brightness(tmpLed475);
         illumination_use(ILLUMINATION_475);
         ble_wait(45);
         Led475 = photodiode();
         ble_wait(5);
         output_low(ILLUMINATION_475);
         delay_ms(2850);
      }
      
      ctr++;
      if( ctr > 15 )       //Try 15 times to set 
      {
         bleNotifyErrorSet(BAD_CATRIDGE);
         return FAIL;
      }
   }
   
//   fprintf(STDOUT, "Brightness calibration stat = %x\r\n", ledBrightnessCalibrated);
   return status;
}

////////////////////////////////////////
void check_bad_or_used_cartridge( ) 
{
#ifdef 588
   uint32   Raw588[3], tmpRaw588;
#else
   uint32   Raw730[3], tmpRaw730;
   uint32   Raw475[3], tmpRaw475;
#endif

   uint8    ctr = 0;
   
   cartridgeDetected = YES;
                 
   if( !( ConfigReadByte (CONFIG_LED_BRIGHTNESS_CALIBRATED) ) )
   {
      fprintf(STDOUT, brightness_resp);
      fprintf(STDOUT, not_resp);
      fprintf(STDOUT, set_resp);
      cartridgeState = CARTRIDGE_STATE_UNKNOWN;
      return;
   }

   fprintf(STDOUT, cartridge_wetting_check_resp);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, please_wait_resp);
   fprintf(STDOUT, new_line_resp);

   while ( ctr++ < 3 )
   {
#ifdef 588   
      illumination_brightness(LedProperties.LevelBrightness588);
      illumination_use(ILLUMINATION_588);
      ble_wait(45);
      Raw588[ctr] = photodiode();
      output_low(ILLUMINATION_588);
      delay_ms( ONE_SECOND );
      
      fprintf(STDOUT, new_line_resp);
      fprintf(STDOUT, bright588_resp);
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, "%lu", Raw588[ctr]);
      fprintf(STDOUT, new_line_resp);
#else      

      illumination_brightness(LedProperties.LevelBrightness730);
      illumination_use(ILLUMINATION_730);
      ble_wait(45);
      Raw730[ctr] = photodiode();
      output_low(ILLUMINATION_730);
            
      illumination_brightness(LedProperties.LevelBrightness475);
      illumination_use(ILLUMINATION_475);
      ble_wait(45);
      Raw475[ctr] = photodiode();
      output_low(ILLUMINATION_475);
      
      delay_ms( 500 );
      
#ifdef DEBUG      
      fprintf(STDOUT, new_line_resp);
      fprintf(STDOUT, bright730_resp);
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, "%lu", Raw730[ctr]);
      fprintf(STDOUT, new_line_resp);

      fprintf(STDOUT, bright475_resp);
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, "%lu", Raw475[ctr]);
      fprintf(STDOUT, new_line_resp);
#endif

#endif
   }

#ifdef 588  
   if( Raw588[2] < NO_CARTRIDGE )
#else   
   if( Raw730[2] < NO_CARTRIDGE || Raw475[2] < NO_CARTRIDGE )
#endif   
   {
      cartridgeState = CARTRIDGE_STATE_UNKNOWN;
      //bleNotifyErrorSet(BAD_CATRIDGE);
      
      fprintf(STDOUT, new_line_resp);
      fprintf(STDOUT, cartridge_resp);
      fprintf(STDOUT, not_resp);
      fprintf(STDOUT, detected_resp);
      fprintf(STDOUT, new_line_resp);
      
      cartridgeDetected = NO;
      return;
   }
   
#ifdef DEBUG
   fprintf(STDOUT, "temp 730 %lu", tmpRaw730);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "temp 475 %lu", tmpRaw475);
   fprintf(STDOUT, new_line_resp);
#endif


#ifdef 588   
   if( Raw588[2] > NEW_CARTRIDGE_MIN && Raw588[2] < NEW_CARTRIDGE_MAX )
#else

   if( Raw730[2] > NEW_CARTRIDGE_MIN_730  && 
       Raw475[2] > NEW_CARTRIDGE_MIN_475  && 
       Raw730[2] < Raw475[2] )
       
#endif   
   { 
      cartridgeState = CARTRIDGE_STATE_NEW;
      bleNotifyErrorSet(NO_ERROR);
      
      fprintf(STDOUT, new_line_resp);
      fprintf(STDOUT, new_resp);
      fprintf(STDOUT, cartridge_resp);
      fprintf(STDOUT, detected_resp);
      fprintf(STDOUT, two_new_lines_resp);
   }

   //Determined if it is used or just wet
   else
   {
      //Wait for 3 seconds to see if the catridge color changes
      delay_ms(THREE_SECONDS);
#ifdef 588      
      illumination_brightness(LedProperties.LevelBrightness588);
      illumination_use(ILLUMINATION_588);
      ble_wait(45);
      tmpRaw588 = photodiode();
      fprintf(STDOUT, "temp 588 %lu", tmpRaw588);
      output_low(ILLUMINATION_588);

      if( tmpRaw588 != Raw588[2] )
#else      
      illumination_brightness(LedProperties.LevelBrightness730);
      illumination_use(ILLUMINATION_730);
      ble_wait(45);
      tmpRaw730 = photodiode();
      output_low(ILLUMINATION_730);
      
      fprintf(STDOUT, new_line_resp);
      illumination_brightness(LedProperties.LevelBrightness475);
      illumination_use(ILLUMINATION_475);
      ble_wait(45);
      tmpRaw475 = photodiode();
      output_low(ILLUMINATION_475);


      if( ( tmpRaw730 <= Raw730[2] + 1 && tmpRaw730 >= Raw730[2] - 1 ) || 
          ( tmpRaw475 <= Raw475[2] + 1 && tmpRaw475 >= Raw475[2] - 1 ) )
#endif      
      {
         cartridgeState = CARTRIDGE_STATE_USED;
         fprintf(STDOUT, used_resp);
         fprintf(STDOUT, cartridge_resp);
         fprintf(STDOUT, detected_resp);
         fprintf(STDOUT, new_line_resp);
      }
                     
      else
      {
         cartridgeState = CARTRIDGE_STATE_JUST_WET;
         fprintf(STDOUT, wet_resp);
         fprintf(STDOUT, cartridge_resp);
         fprintf(STDOUT, detected_resp);
         fprintf(STDOUT, two_new_lines_resp);
      }
      
      bleNotifyErrorSet(BAD_CATRIDGE);
      fprintf(STDOUT, new_line_resp);
   }
}

////////////////////////////////////////
void MeasureDarkness() 
{
   struct LedReadings x;
 
   illumination_all_off();
   ble_wait(45);
   x.Raw730 = photodiode();
   ble_wait(5);
   x.Raw588 = photodiode();
   ble_wait(5);
   x.Raw475 = photodiode();
   ble_wait(5);
   
   LedProperties.DarkReading730 = x.Raw730;
   LedProperties.DarkReading588 = x.Raw588;
   LedProperties.DarkReading475 = x.Raw475;
   
   fprintf(STDOUT, darkness_resp);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright730_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", x.Raw730);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright588_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", x.Raw588);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright475_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", x.Raw475);
   
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
struct LightMeasurements Whitebalances() 
{
   struct LightMeasurements x;
   struct LedReadings y;
  
   y = MeasureLEDs();
   fprintf(STDOUT, "Whitebalance: ");
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright730_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", y.Raw730);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright588_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", y.Raw588);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright475_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%lu", y.Raw475);
   fprintf(STDOUT, new_line_resp);
   
   x.Light730 = 62.0729508196721 / (float32)(y.Raw730 - LedProperties.DarkReading730);
   x.Light588 = 62.413275862069 / (float32)(y.Raw588 - LedProperties.DarkReading588);
   x.Light475 = 60.92125 / (float32)(y.Raw475 - LedProperties.DarkReading475);
   
   fprintf(STDOUT, "WB Coeffficients: ");
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright730_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%.4f", x.Light730);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright588_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%.4f", x.Light588);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, bright475_resp);
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, "%.4f", x.Light475);
   fprintf(STDOUT, new_line_resp);
   
   return x;
}

////////////////////////////////////////
uint16 AnalyzeBreathSample(float32* AD2, float32* AD_Final)
{   
   uint16 status = READING_ERROR_NONE;
   int i1=0;
   int i2=0;
   struct LedReadings RawReadings;
   struct LightMeasurements x;
   float32 r475o588;
   float32 r588o730;
   float32 Diff1 = 0.0;
   float32 Diff2 = 0.0;
   float32 Diff3 = 0.0;
      
   fprintf(STDOUT, "Measuring breath sample...\r\nTime(s), Red 730, Orange 588, Blue 475\r\n");
   testResult.stepID = ANALYZE_BREATH_SAMPLE;
   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
//   SetFullTestStepID(ANALYZE_BREATH_SAMPLE);

   for( int16 i = 2; i <= 40; i++ )
   {
      illumination_brightness(LedProperties.LevelBrightness730);
      illumination_use(ILLUMINATION_730);

      delay_ms(45);
      RawReadings.Raw730 = photodiode();
      
      delay_ms(5);
      illumination_brightness(LedProperties.LevelBrightness588);
      illumination_use(ILLUMINATION_588);
      
      delay_ms(45);
      RawReadings.Raw588 = photodiode();

      delay_ms(5);
      illumination_brightness(LedProperties.LevelBrightness475);
      illumination_use(ILLUMINATION_475);

      delay_ms(45);
      RawReadings.Raw475 = photodiode();
      illumination_all_off();

      if( RawReadings.Raw730 < NO_CARTRIDGE ||
          RawReadings.Raw588 < NO_CARTRIDGE ||
          RawReadings.Raw475 < NO_CARTRIDGE )
      {
         fprintf(STDOUT, "\r\n\r\n*****Cartridge removed during reading*****\r\n\r\n");
         for( i = 0; i < 5; i++ )
         {
            play_error();
            delay_ms( ONE_SECOND );
         }
         
         bleNotifyErrorSet(CARTRIDGE_REMOVED_DURING_ANALYSIS);
         return CARTRIDGE_REMOVED_DURING_ANALYSIS;
      }

      fprintf(STDOUT, "%lu, %lu, %lu, %lu\r\n", i*3, RawReadings.Raw730, RawReadings.Raw588, RawReadings.Raw475);
      
      if (i < 11) 
         CartridgeRawDataSet1[ i1++ ] = RawReadings;
      
      else if (i > 33 && i <= 40) 
         CartridgeRawDataSet2[ i2++ ] = RawReadings;

      delay_ms(5);
      if (i == 40)
         break;

      delay_ms(2850);
   }
  
   illumination_all_off();
   
   if(!status) 
   {
      fprintf(STDOUT, "\r\nAnalyzing wet cartridge...\r\nTime(s), Red 730, Orange 588, Blue 475\r\n");
      r475o588 = 0.0;
      r588o730 = 0.0;
      for( i1=0, i2=2; i1<4; i1++, i2++ ) 
      {
         RawReadings = CartridgeRawDataSet1[i1];
         
         x.Light730 = (float32)RawReadings.Raw730 * whitebalances.Light730;
         x.Light588 = (float32)RawReadings.Raw588 * whitebalances.Light588;
         x.Light475 = (float32)RawReadings.Raw475 * whitebalances.Light475;
         
         fprintf(STDOUT, "%u, %.4f, %.4f, %.4f\r\n", i2*3, x.Light730, x.Light588, x.Light475);
         r475o588 += (x.Light475 / x.Light588);
         r588o730 += (x.Light588 / x.Light730);
      }
      
      Diff1 = (r475o588 - r588o730) / 4.0;
      fprintf(STDOUT, "Diff1= %.4f\r\n", Diff1);
      
      fprintf(STDOUT, "\r\nAnalyzing initial cartridge...\r\nTime(s), Red 730, Orange 588, Blue 475\r\n");
      r475o588 = 0.0;
      r588o730 = 0.0;
      for(i1=1, i2=3; i1<9; i1++, i2++) 
      {
         RawReadings = CartridgeRawDataSet1[i1];
         
         x.Light730 = (float32)RawReadings.Raw730 * whitebalances.Light730;
         x.Light588 = (float32)RawReadings.Raw588 * whitebalances.Light588;
         x.Light475 = (float32)RawReadings.Raw475 * whitebalances.Light475;
         
         fprintf(STDOUT, "%u, %.4f, %.4f, %.4f\r\n", i2*3, x.Light730, x.Light588, x.Light475);
         r475o588 += (x.Light475 / x.Light588);
         r588o730 += (x.Light588 / x.Light730);
      }
      
      Diff2 = (r475o588 - r588o730) / 8.0;
      fprintf(STDOUT, "Diff2= %.4f\r\n", Diff2);
      
      fprintf(STDOUT, "\r\nAnalyzing final cartridge...\r\nTime(s), Red 730, Orange 588, Blue 475\r\n");
      r475o588 = 0.0;
      r588o730 = 0.0;
      for(i1=0, i2=34; i1<7; i1++, i2++) 
      {
         RawReadings = CartridgeRawDataSet2[i1];
         
         x.Light730 = (float32)RawReadings.Raw730 * whitebalances.Light730;
         x.Light588 = (float32)RawReadings.Raw588 * whitebalances.Light588;
         x.Light475 = (float32)RawReadings.Raw475 * whitebalances.Light475;
         
         fprintf(STDOUT, "%u, %.4f, %.4f, %.4f\r\n", i2*3, x.Light730, x.Light588, x.Light475);
         r475o588 += (x.Light475 / x.Light588);
         r588o730 += (x.Light588 / x.Light730);
      }
      
      Diff3 = (r475o588 - r588o730) / 7.0;
      fprintf(STDOUT, "Diff3= %.4f\r\n", Diff3);
      
      float32 AD1 = Diff3 - Diff2;
      *AD2 = Diff3 - Diff1;
      *AD_Final = *AD2 - AD1;
      fprintf(STDOUT, "\rAD1=%.4f, AD2=%.4f, AD_Final=%.4f\r\n", AD1, *AD2, *AD_Final);
     
   } 
   
   else 
      play_error();
   
   return status;
}
////////////////////////////////////////
float32 EvaluateConcentration( float32 x, float32 AD_Final) 
{
   if ((x < 1.5 && AD_Final < 0.0) || x < 0.0) 
      return 0.0;
   return x;
}

////////////////////////////////////////
uint16 Calibrate() 
{
   float32 AD2;
   float32 AD_Final;
   uint16 status = AnalyzeBreathSample(&AD2, &AD_Final);
   
   if ( status == CARTRIDGE_REMOVED_DURING_ANALYSIS )
      return status;
      
   if (!status) 
   {
      float32 Conc1 = (AD2-0.0300)/0.0200;
      float32 Conc2 = (AD2-0.0500)/0.0200;
      float32 Conc3 = (AD2-0.0900)/0.0200;
      float32 Conc;
      
      Conc = EvaluateConcentration(Conc1, AD_Final);
      fprintf(STDOUT, "Conc1 = (AD2 - 0.03) / 0.02 = %.4f => Concentration= %.4f\r\n", Conc1, Conc);
      
      Conc = EvaluateConcentration(Conc2, AD_Final);
      fprintf(STDOUT, "Conc2 = (AD2 - 0.05) / 0.02 = %.4f => Concentration= %.4f\r\n", Conc2, Conc);
      
      Conc = EvaluateConcentration(Conc3, AD_Final);
      fprintf(STDOUT, "Conc3 = (AD2 - 0.09) / 0.02 = %.4f => Concentration= %.4f\r\n", Conc3, Conc);
   }
   
   return status;
}

////////////////////////////////////////
uint16 readings_csv(float32* concentration)
{   
   float32 AD2;
   float32 AD_Final;
   uint16 status = AnalyzeBreathSample(&AD2, &AD_Final);
   
   if ( status == CARTRIDGE_REMOVED_DURING_ANALYSIS )
      return status;
      
   if ( !status ) 
   {
      float32 slope = ConfigReadFloat(CONFIG_SLOPE);
      float32 intercept = ConfigReadFloat(CONFIG_INTERCEPT);
      float32 x = (AD2-intercept)/slope;     
      float32 conc = EvaluateConcentration(x, AD_Final);
      
      fprintf(STDOUT, "Conc = (AD2 - %.4f) / %.4f = %.4f => Concentration= %f\r\n", intercept, slope, x, conc);
      *concentration = conc;
   }
   
   else 
      *concentration = 0.0;
   
   return status;
}

////////////////////////////////////////
uint8_t checkForCartridge()
{
   //check for cartridge
   fprintf(STDOUT, "\r\n**** Check for cartridge ****\r\n");

   check_bad_or_used_cartridge( );
   
   if( !cartridgeDetected )
      return FAIL;
   
   else
   {
      delay_ms( ONE_SECOND );
      return PASS;
   }
}
////////////////////////////////////////
void full_test( int mode )
{
   int                status;
   float32            concentration;
   
   if( mode == KETONE_TEST ) 
   {
      status = readings_csv( &concentration );
      
      if ( status == CARTRIDGE_REMOVED_DURING_ANALYSIS )
      {
         bleNotifyErrorSet(CARTRIDGE_REMOVED_DURING_ANALYSIS);
         testResult.status = CARTRIDGE_REMOVED_DURING_ANALYSIS;
         goto FullTest_End;
      }
      
      else
      {


#ifdef HIGH_SCORE_TEST
         concentration = 55.55;
         testResult.score = f_PICtoIEEE(concentration);
#else
         testResult.score = f_PICtoIEEE(concentration);
#endif
     
         if( concentration > 20 )
         {
            fprintf(STDOUT, "Ketone reading error, replace catridge and retest.\r\n");
            bleNotifyErrorSet(BAD_SCORE);
            testResult.status = BAD_SCORE;
            testResult.totalTestTime = get_ticks();
         }
      
         if( status == READING_ERROR_NONE )
         {
            fprintf(STDOUT, "FullTest finished successfully.\r\n");
            testResult.totalTestTime = get_ticks();
            testResult.status = NO_ERROR;
         }
      }
   }
   
   else 
   {
      status = Calibrate();

      if( status == READING_ERROR_NONE )
         fprintf(STDOUT, "Calibration finished successfully.\r\n");
   }

FullTest_End:  

#ifdef SOFTWARE_DEBUG
   fprintf(STDOUT, "Test Mode %ld.\r\n", testResult.testMode);
   fprintf(STDOUT, "Test Score %ld.\r\n", testResult.score);
   fprintf(STDOUT, "Test Time %ld.\r\n", testResult.totalTestTime);
   fprintf(STDOUT, "Test Status %d.\r\n", testResult.status);
   fprintf(STDOUT, "Test ID %d.\r\n", testResult.stepID);   
#endif

   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
}

////////////////////////////////////////////////////////////////////////////////
int8 performBreathTest( int testMode )
{
   struct   timePara   *time;
   int8     ctr, rslt = FAIL;
   int8     errorTerminated = NO;
//   uint8    tmpStepID = HARDWARE_CHECK;
   uint32   saveTimer, setTimer, currentTime, previousTime;
   uint32   timeCounter = 0;
   uint32   Raw730;
   
   time = &cmdReceived.time;
   testResult.testMode = testMode;   
   brightnessChecked = NO;
   
   fprintf(STDOUT, new_line_resp);
   ShowPrompt();
   fprintf(STDOUT, "Breath test started\r\n");
   fprintf(STDOUT, new_line_resp);
   
   //Save the current timer count
   saveTimer = get_ticks();
   setTimer = 0x0;
   previousTime = 0x0;
   set_ticks( setTimer );
   
   
   //Set the busy flag, no interruption from BLE from this point on
   FullTestRunning = 1;
   
   //Update the breathVolume
   breathVolume = ConfigReadFloat(CONFIG_BREATH_VOLUME);
   
   // Reset previously set error bits.
   bleNotifyErrorResetErrorBits();
   
   fprintf(STDOUT, "Initial Light-Conditions:");
   fprintf(STDOUT, new_line_resp);
   
   LedProperties.LevelBrightness730 = ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS);
   LedProperties.LevelBrightness588 = ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS);
   LedProperties.LevelBrightness475 = ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS);
   
   fprintf(STDOUT, "Brightness levels:\r\n730nm, 588nm, 475nm\r%u, %u, %u\r\n", LedProperties.LevelBrightness730, LedProperties.LevelBrightness588, LedProperties.LevelBrightness475);
   
   testResult.stepID = INSERT_CARTRIDGE;
   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
   
   //Check and verify cartridge condition prior to test
   while( TRUE )
   {
      rslt = checkForCartridge();   

      if ( cartridgeState != CARTRIDGE_STATE_NEW )
      {
         fprintf(STDOUT, new_line_resp);
         fprintf(STDOUT, insert_new_cartridge_resp);
         fprintf(STDOUT, new_line_resp);
         
         delay_ms(FIVE_SECONDS);
         timeCounter += FIVE_SECONDS;
      }

#ifdef TURN_OFF_BRIGHTNESS_CHECK  

      if( cartridgeState == CARTRIDGE_STATE_NEW )
      {
         cartridgeDetected = YES;
         brightnessChecked = YES;
         break;
      }

#else

      if( (cartridgeState == CARTRIDGE_STATE_NEW) && !brightnessChecked )
      {
         rslt = autoSetLEDBrightness( 65, 65, 65 );
           
         if( rslt != PASS )
         {
            fprintf(STDOUT, "***** LED brightness setting failed *****\r\n");
            fprintf(STDOUT, two_new_lines_resp);
            fprintf(STDOUT, insert_new_cartridge_resp);
            fprintf(STDOUT, new_line_resp);
         }
         
         else 
         {
            brightnessChecked = YES;
            cartridgeDetected = YES;
            break;
         }
      }
      
#endif

      if( timeCounter >= FOUR_MINUTES )
      {
         testResult.totalTestTime = get_ticks();
         testResult.status = TIME_OUT;
         
         bleNotifyErrorSet( TIME_OUT );
         delay_ms(ONE_MINUTE);
         
         fprintf(STDOUT, new_line_resp);
         fprintf(STDOUT, time_limit_exceeded);
         fprintf(STDOUT, new_line_resp);
         rslt = FAIL;
         errorTerminated = YES;
         goto FullTestEnd;
      }
   }   
   
   currentTime = get_ticks();
   testTime.cartridgeInsertionTime = currentTime - previousTime;
   previousTime = currentTime;
   
   MeasureDarkness();
   whitebalances = Whitebalances();           
   
   // Reset previously set error bits.
   bleNotifyErrorResetErrorBits();
   
   switch( testMode )
   {
      case KETONE_TEST:
         fprintf(STDOUT, "Ketone ");
         break;
         
      case AMMONIA_TEST:
         fprintf(STDOUT, "Ammonia ");
         break;
         
      default:
         fprintf(STDOUT, "Calibration ");

   }

   fprintf(STDOUT, test_started_resp);
   fprintf(STDOUT, new_line_resp);
   
   
//************* Need to set the stepcode and errorcode here **************

   if( testMode != CALIBRATION_TEST )
   {
      illumination_brightness( 30 );
      illumination_use(ILLUMINATION_588);
      //output_high(ILLUMINATION_588);

      testResult.stepID = BLOW;
      ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
      
      fprintf(STDOUT, "....Start breathing into the mouth piece...\r\n");


#ifdef BYPASS_BREATH_COLLECTION
   
      testStat.pressure = 110000;
      testStat.temperature = f_PICtoIEEE(89.5);
      testStat.volume = 600;
      testStat.numOfBlowAttempts = 3;
      bleNotifyErrorSet(NO_ERROR);
      rslt = PASS;
      fprintf(STDOUT, "Breath volume settings %lu\r\n", breathVolume);   
#else

      //Get breath sample
      rslt = Compute_Breath_Volume(&gas_sensor);
      
      if( testResult.status == TIME_OUT )
      {
         errorTerminated = YES;
         testResult.totalTestTime = get_ticks(); //update the Test Report BLE
         goto FullTestEnd;
      }

#endif

      if( rslt != PASS )
      {
         fprintf(STDOUT, "***** Insufficient breath volume *****\r\n");
         bleNotifyErrorSet(BAD_BLOW_VOLUME);
         testResult.totalTestTime = get_ticks(); //update the Test Report BLE
         goto FullTestEnd;
      }
   }      
    
   else
      fprintf(STDOUT, "Insert catridge with known gas samples for calibration into unit when done\r\n");

   currentTime = get_ticks();
   testTime.blowTime = currentTime - previousTime;
   previousTime = currentTime;
   
   //reset flag to fail
   rslt = FAIL;
   timeCounter = 0;
      
#ifndef SKIP_WETCARTRIDGE_CHECK      
   //Check for wet cartridge once every 5 seconds for the next 5 minutes
   while( !rslt && timeCounter <= 48 )  //5 minutes time limit to wet the cartridge
   {
      delay_ms( FIVE_SECONDS );
      testResult.stepID = WET;
      ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
      
      rslt = check_wet_cartridge();
         
      if( rslt != YES )
      {
         illumination_brightness(ONE_HUNDRED);
         illumination_use(ILLUMINATION_475);
         
         if( timeCounter == 6 )              //Wait for 30 seconds before notify BLE
         {
            bleNotifyErrorSet(NO_WETTING);
            ble_wait(75);
         }
      }
         
      timeCounter++;
   }
  
   if( rslt != PASS )
   {
      bleNotifyErrorSet(TIME_OUT);
      testResult.totalTestTime = get_ticks();
      testResult.status = TIME_OUT;
      errorTerminated = YES;

#ifdef DEBUG
      fprintf(STDOUT, "\r\n\r\n***** Cartridge not wet *****\r\n");
#endif

      delay_ms( ONE_MINUTE );
      goto FullTestEnd;
   }
               
   else     
#endif   
   {
      for( ctr = 7; ctr > 0; ctr-- )
      {
         fprintf(STDOUT, "%d seconds to ketone analysis\r\n", ctr);
         delay_ms( ONE_SECOND );
      }
      
      currentTime = get_ticks();
      testTime.wettingTime = currentTime - previousTime;
      previousTime = currentTime;
      
      bleNotifyErrorSet(NO_ERROR);
      testResult.stepID = ANALYZE_BREATH_SAMPLE;      
      ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
      full_test( testMode );
      
      if( testResult.status == CARTRIDGE_REMOVED_DURING_ANALYSIS )
      {
         cartridgeDetected = NO;
         rslt = FAIL;
         errorTerminated = YES;
         goto FullTestEnd;
      }
   }
   
   timeCounter = 0;
   
FullTestEnd:
   
   fprintf(STDOUT, "Update the BLE info\r\n");
   
   testResult.stepID = SEND_RESULTS;
   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
   
   ble_wait( 150 );
   
   //update the Test result to BLE
   ble_cmd_attributes_write(BLE_HANDLE_TEST_REPORT, 0, sizeof(testResult), (uint8*)&testResult);
   ble_cmd_attributes_write(BLE_HANDLE_TEST_REPORT, sizeof(testResult), sizeof(struct timePara), (uint8*)time);
   ble_wait( 150 );
   
   ble_cmd_attributes_write(BLE_HANDLE_TEST_STAT, 0, sizeof(testStat), (uint8*)&testStat);
   ble_wait( 150 );
   
   ble_cmd_attributes_write(BLE_HANDLE_TEST_STAGE, 0, sizeof(testTime), (uint8*)&testTime);
   ble_wait( 150 );

   fprintf(STDOUT, "BLE info updated\r\n");

   //calculated time test finished
   testResult.totalTestTime = get_ticks();
   
   if( testResult.status != TIME_OUT )
   {
      testResult.stepID = REMOVE_CARTRIDGE;
      ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
  
//      fprintf(STDOUT, "cartridge present - %d\r\n", cartridgeDetected);

      while( cartridgeDetected )
      {

#ifdef DEBUG
         //check for cartridge
         fprintf(STDOUT, "\r\n**** Check for cartridge ****\r\n");
#endif            
         illumination_brightness(LedProperties.LevelBrightness730);
         illumination_use(ILLUMINATION_730);
         ble_wait(45);
         Raw730 = photodiode();
         output_low(ILLUMINATION_730);
         ble_wait(50);

#ifdef DEBUG
         fprintf(STDOUT, "\r\nRaw 730 reading - %lu \r\n", Raw730);
#endif  

//         if( Raw730 < 550 && Raw730 > 75 )
         if( Raw730 > 75 )
         {
            cartridgeDetected = YES;
            
            if( timeCounter == THIRTY_SECONDS )
               bleNotifyErrorSet(CARTRIDGE_NOT_REMOVED);
            
            fprintf(STDOUT, "****Please removes cartridge****\r\n");
            illumination_brightness( ONE_HUNDRED );
            illumination_use(ILLUMINATION_475);
            ble_wait(ONE_HUNDRED);
        
            delay_ms( FIVE_SECONDS );
            timeCounter += FIVE_SECONDS;
            play_error();
            illumination_all_off();
            ShowPrompt();
         }
         
         else
         {
            currentTime = get_ticks();
            testTime.removeCartridgeTime = currentTime - previousTime;
            previousTime = currentTime;
            bleNotifyErrorSet(NO_ERROR);
            rslt = PASS;
            cartridgeDetected = NO;
            brightnessChecked = NO;
         }
      
         if( timeCounter > FOUR_MINUTES )            //5 minutes is up
         {  
            bleNotifyErrorSet(TIME_OUT);
            errorTerminated = YES;
            delay_ms( ONE_MINUTE );
            break;
         }
      }
   }
   
   currentTime = get_ticks();
   if( errorTerminated == YES || testResult.status == TIME_OUT )
      testResult.stepID = ERROR_TERMINATION;
      
   else
   {
      testResult.stepID = TEST_COMPLETED;
      bleNotifyErrorResetErrorBits();
      ble_wait( 150 );
   
#ifdef CONNIE_STUDY
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Number of blow attempt - %ld", testStat.numOfBlowAttempts);

   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Cartridge insertion time - %ld", testTime.cartridgeInsertionTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Breath exhale time - %ld", testTime.blowTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Wetting time - %ld", testTime.wettingTime);
   fprintf(STDOUT, new_line_resp);
#endif
      
   }
   
//   fprintf(STDOUT, "StepID - %d", testResult.stepID);
   
   FullTestRunning = 0;
   reading_save( &testResult, time );   
   
//   ble_cmd_attributes_write(BLE_HANDLE_TEST_STAGE, 0, sizeof(testTime), (uint8*)&testTime);
//   ble_wait( 150 );
   
   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
   ble_wait(ONE_SECOND);
   fprintf(STDOUT, new_line_resp);
   
   if( testResult.status == TIME_OUT )
      shutDown ();
      
   else
   {
      struct reading r;
      
      for( ctr = 0; ctr < 10; ctr++ )
      {
         r = reading_fetch( ctr );
            
         ble_cmd_attributes_write( BLE_HANDLE_READING, 0, sizeof(struct reading), (uint8 *)&r );  
         ble_wait( 150 );
      }

      //put the timer back the way it was
      set_ticks ( saveTimer );
      powerDownTimeCtr = INACTIVITY_TIME_LIMIT;
   }
      
   return rslt;
}

////////////////////////////////////////////////////////////////////////////////
uint8 setBreathVolume( uint16 volume )
{
   uint8_t  rslt = FAIL;
   
   if( volume < MININUM_BREATH_VOLUME || volume > MAXINUM_BREATH_VOLUME )
   {
      fprintf(STDOUT, "\r\nBreath volume must be between 300-1000 ml\r\n");
      return rslt;
   }

   ConfigWriteFloat(CONFIG_BREATH_VOLUME, volume);
   
   //update breath volume setting
   breathVolume = ConfigReadFloat(CONFIG_BREATH_VOLUME);
//   fprintf(STDOUT, "**Breath Volume Setting : %lu ml \r\n\r\n", breathVolume);
}

////////////////////////////////////////////////////////////////////////////////

#ifdef SOFTWARE_DEBUG
void testStage()
{
   testTime.blowTime = 8200;
   testTime.cartridgeInsertionTime = 12000;
   testTime.removeCartridgeTime = 24150;
   testTime.wettingTime = 18550; 
   
   //update the Teststage BLE
   ble_cmd_attributes_write(BLE_HANDLE_TEST_STAGE, 0, sizeof(testTime), (uint8*)&testTime);
   while(!ble_log_process());
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Cartridge insertion Time - %lx", testTime.cartridgeInsertionTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Blow Time - %lx", testTime.blowTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Wetting Time - %lx", testTime.wettingTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Removed Cartridge Time - %lx", testTime.removeCartridgeTime);
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////////////////////////////////////////////
void testStats()
{

   testStat.temperature = f_PICtoIEEE(90.25);
   testStat.pressure = 115000;
   testStat.volume = 612;
   testStat.humidity = 80;
   testStat.numOfBlowAttempts = 12;
   
   
   //update the Teststat BLE
   ble_cmd_attributes_write(BLE_HANDLE_TEST_STAT, 0, sizeof(testStat), (uint8*)&testStat);
   while(!ble_log_process());
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Temperature - %lx", testStat.temperature);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Pressure - %lx", testStat.pressure);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Volume - %lx", testStat.volume);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Humidity - %lx", testStat.humidity);

   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Number of attempts - %lx", testStat.numOfBlowAttempts);
   
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////////////////////////////////////////////
void testReport()
{
   uint8   ctr, *tmpPtr;
#define TEST_VECTOR   
#ifdef TEST_VECTOR
   
   testResult.testMode = 0XFF;
   testResult.totalTestTime = 123456;
   testResult.score = f_PICtoIEEE(11);
   testResult.status = NO_ERROR;
   testResult.stepID = REMOVE_CARTRIDGE;
   
   cmdReceived.time.hour = 0XAA;
   cmdReceived.time.minute = 0XBB;
   cmdReceived.time.second = 0XCC;
   cmdReceived.time.day = 0XDD;
   cmdReceived.time.month = 0XEE;
   cmdReceived.time.year = 19;
   
#else

   tmpPtr = (uint8* )&testResult;
   for( ctr = 0; ctr < sizeof(testResult); ctr++ )
      *tmpPtr++ = 0;
      
   tmpPtr = (uint8* )&cmdReceived.time;
   for( ctr = 0; ctr < sizeof(struct timePara); ctr++ )
      *tmpPtr++ = 0;

#endif
   
   //update the Teststat BLE
   ble_cmd_attributes_write(BLE_HANDLE_TEST_REPORT, 0, sizeof(testResult), (uint8*)&testResult);
   ble_cmd_attributes_write(BLE_HANDLE_TEST_REPORT, sizeof(testResult), sizeof(struct timePara), (uint8*)&cmdReceived.time);
   while(!ble_log_process());

   ble_cmd_attributes_write(BLE_HANDLE_READING, 0, sizeof(testResult), (uint8*)&testResult);
   ble_cmd_attributes_write(BLE_HANDLE_READING, sizeof(testResult), sizeof(struct timePara), (uint8*)&cmdReceived.time);
   while(!ble_log_process());
  
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "testMode - %x", testResult.testMode);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "totalTestTime - %lu", testResult.totalTestTime);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "totalTestTime - 0x%lx", testResult.totalTestTime);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Score - 0x%lx", testResult.score);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Status - %x", testResult.status);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "stepID - %x", testResult.stepID);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Hour - %x", cmdReceived.time.hour);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Minute - %x", cmdReceived.time.minute);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Second - %x", cmdReceived.time.second);
   
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Date - %x", cmdReceived.time.day);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Month - %x", cmdReceived.time.month);
   fprintf(STDOUT, new_line_resp);
   fprintf(STDOUT, "Year - %x", cmdReceived.time.year);
   
   fprintf(STDOUT, new_line_resp);
}

#endif

////////////////////////////////////////////////////////////////////////////////
void selfTest()
{
   uint8_t  tmpCtr, rslt = FAIL;
   uint16   tmpBreathVolume;
   uint32   tmpTime, previousTimer, tmp = 0;
   float    temp_float, temp_float_1;   
   
   int opCode = HARDWARE_CHECK;
   myStatus = MYSTATUS_OK;
   
   ConstructBatteryModule();
   
   deviceStat.fwMajor = FIRMWARE_VERSION_MAJOR;
   deviceStat.fwMinor = FIRMWARE_VERSION_MINOR;
   deviceStat.fwRevision = FIRMWARE_VERSION_REVISION;
   deviceStat.useCount = read_ext_eeprom(CONFIG_MOUTHPIECE_USES);


   fprintf(STDOUT, "\r\nBLE ID : 0x");
   
   //Updtate the BLE ID
   for( tmpCtr = 0; tmpCtr < BLE_MODULE_ID_SIZE; tmpCtr++ )
   {
      deviceStat.bleModuleId[tmpCtr] = BleModuleId[tmpCtr];
      fprintf(STDOUT, "%2X", deviceStat.bleModuleId[tmpCtr]);      
   }

   
   fprintf(STDOUT, new_line_resp);
   ble_configure();

   //We are here, bluetooth works
   fprintf(STDOUT, "BLE ");
   fprintf(STDOUT, dash_resp);
   fprintf(STDOUT, pass_resp);
   fprintf(STDOUT, new_line_resp);
  
  
   //update bluetooth step code info
   ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &opCode);

#ifdef LIS3DH_PRESENT     
   if (!LIS3DH_Present())
   {
      fprintf(STDOUT, "LIS3DH ");
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, fail_resp);
      fprintf(STDOUT, new_line_resp);
      
      bleNotifyErrorSet(HARDWARE_FAIL);   
      myStatus |= LIS3DH_MALFUNCTION;
   }
   
   else 
   {
      LIS3DH_Configure();
      fprintf(STDOUT, "LIS3DH ");
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, pass_resp);
      fprintf(STDOUT, new_line_resp);
   }
#endif

   //Init Temp, Pressure, Humidity sensor
   initBME();      
   rslt = checkBMEPowerUp(&gas_sensor);

   if( rslt != BME680_OK )
   {
      myStatus |= BME680_MALFUNCTION;
      fprintf(STDOUT, "BME");
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, fail_resp);
      fprintf(STDOUT, new_line_resp);
      bleNotifyErrorSet(HARDWARE_FAIL);
   }
   
   else
   {
      fprintf(STDOUT, "BME");
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, pass_resp);
      fprintf(STDOUT, new_line_resp);
   }

   //Set breath volume
   tmpBreathVolume = ConfigReadFloat(CONFIG_BREATH_VOLUME);
   if( tmpBreathVolume < MININUM_BREATH_VOLUME || tmpBreathVolume > MAXINUM_BREATH_VOLUME )
      setBreathVolume( STANDARD_BREATH_VOLUME );
      
   else 
      breathVolume = tmpBreathVolume;
   
   BatteryManagement();
   deviceStat.batteryLevel = BatteryCapacity();
   fprintf(STDOUT, "Battery Level = %lu%%", deviceStat.batteryLevel);
   fprintf(STDOUT, new_line_resp);
   
   //Set slope to .020
   temp_float = SLOPE;
   ConfigWriteFloat(CONFIG_SLOPE, temp_float);
   
   //Set intercept to .090
   temp_float = INTERCEPT;
   ConfigWriteFloat(CONFIG_INTERCEPT, temp_float);
   
   //Read back and verify EEPROM
   temp_float   = ConfigReadFloat(CONFIG_SLOPE);
   temp_float_1 = ConfigReadFloat(CONFIG_INTERCEPT);

   if( temp_float != SLOPE || temp_float_1 != INTERCEPT )
   {
      fprintf(STDOUT, "EEPROM ");
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, fail_resp);
      fprintf(STDOUT, new_line_resp);
      myStatus |= EEPROM_MALFUNCTON;
   }

   else
   {
      fprintf(STDOUT, "Slope: %.3f\r\n", temp_float);
      fprintf(STDOUT, "Intercept: %.3f\r\n", temp_float_1);
      fprintf(STDOUT, "Volume Settings : %lu ml \r\n\r\n", breathVolume);
   }
   
   ledBrightnessCalibrated = ConfigReadByte(CONFIG_LED_BRIGHTNESS_CALIBRATED);
   fprintf(STDOUT, led_resp);
   
   if( ledBrightnessCalibrated != UNIT_CALIBRATED )
   {
      fprintf(STDOUT, brightness_resp);
      fprintf(STDOUT, not_resp);
      fprintf(STDOUT, calibrated_resp);
      fprintf(STDOUT, new_line_resp);
   }
   
   else
   {
      fprintf(STDOUT, brightness_resp);
      fprintf(STDOUT, calibrated_resp);
      fprintf(STDOUT, new_line_resp);
   }
   
   if( myStatus )
   {
      //update bluetooth error code info
      bleNotifyErrorSet(HARDWARE_FAIL);
      hardwareFailure();
   }
   
   else
   {
      fprintf(STDOUT, new_line_resp);
      
      fprintf(STDOUT, hardware_check_resp);
      fprintf(STDOUT, dash_resp);
      fprintf(STDOUT, pass_resp);
      fprintf(STDOUT, two_new_lines_resp);
      
      previousTimer = get_ticks();
      set_ticks(tmp);

      //Unit pass hardware test
      while( TRUE )
      {
         illumination_brightness( 90 );
         illumination_use(ILLUMINATION_730);
         ble_wait(ONE_HUNDRED);
         output_low(ILLUMINATION_730);
         ble_wait(50);

         illumination_use(ILLUMINATION_588);
         ble_wait(ONE_HUNDRED);
         output_low(ILLUMINATION_588);
         ble_wait(50);

         illumination_use(ILLUMINATION_475);
         ble_wait(ONE_HUNDRED);
         output_low(ILLUMINATION_475);
         ble_wait(500);
         
         bleNotifyErrorResetErrorBits();
         tmpTime = get_ticks();
         //fprintf(STDOUT, "\r\n %lu \r\n", tmpTime);
         if( tmpTime > FIVE_SECONDS )
            break;
      }
   }
   
   ble_cmd_attributes_write(BLE_HANDLE_DEVICE_STAT, 0, sizeof(deviceStat), (uint8 *)&deviceStat);
   set_ticks(previousTimer);
}

////////////////////////////////////////////////////////////////////////////////
void main()
{
   uint32   tmpTimer, timer_current = 0;
   char     status = PASS;
   uint8_t  *tmpPtr, tmpCtr = 0, rslt = 0;
   uint16   tmpValue = 0;
   
   output_high(PIN_C1);
   output_high(PIN_C2);
   
   setup_oscillator(OSC_8MHZ|OSC_INTRC,OSC_STATE_STABLE);
   setup_adc_ports(sAN0|sAN1);
   setup_adc(ADC_CLOCK_INTERNAL|ADC_TAD_MUL_0);

   init_ext_eeprom();

   enable_interrupts(GLOBAL);
   enable_interrupts(INT_RDA);

   fprintf(STDOUT, "\r\n");
   fprintf(STDOUT, "Invoy 5.0 Version Major %d. Minor %d. Revision %d\r\n\r\n",
     FIRMWARE_VERSION_MAJOR,
     FIRMWARE_VERSION_MINOR,
     FIRMWARE_VERSION_REVISION);
  
   selfTest();

   TestMode = 0;
   char buffer[256];
   uint8 buffer_index = 0;
   
   LedProperties.LevelBrightness730 = ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS);
   LedProperties.LevelBrightness588 = ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS);
   LedProperties.LevelBrightness475 = ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS);
   
   //Initializing 
   bluetoothState = NO;
   cartridgeDetected = NO;
   brightnessChecked = NO;
   calibrationStatus = NO;
   cartridgeState = CARTRIDGE_STATE_UNKNOWN;
   powerDownTimeCtr = INACTIVITY_TIME_LIMIT;  
   
   tmpPtr = &cmdReceived;

   //clear the cmd structure
   for( tmpCtr = 0; tmpCtr < sizeof( cmdReceived ); tmpCtr++ )
      *tmpPtr[tmpCtr] = 0x0;

   tmpPtr = NULL;   
   tmpCtr = 0;
   tmpTimer = 0;
   ShowPrompt();
   
  
   for(;;)
   {
      timer_current = get_ticks();
      if( timer_current >= TEN_SECONDS )
      {
         tmpTimer++;
         tmpCtr++;
         set_ticks(tmpValue);
      }

      if( tmpCtr >= 3 )
      { 
         powerDownTimeCtr-= tmpCtr;
         fprintf(STDOUT, "\r\n****Device inactivity detected*****\r\n");
         fprintf(STDOUT, "****%lu second(s) to powerdown****\r\n\r\n", powerDownTimeCtr * 10);
         ShowPrompt();
         tmpCtr = 0;

#ifdef TURNOFF_AUTO_POWERDOWN        
//***********************Turn on for testing purposes only, turn off when testing is done************************************
         powerDownTimeCtr = INACTIVITY_TIME_LIMIT;         
//***********************Turn on for testing purposes only, turn off when testing is done************************************
#endif

      }
      
      if( !powerDownTimeCtr )          // 5 minutes time limit of inactivity allow
      {
         fprintf(STDOUT, "\r\n****Exceed inactivity time limit, entering sleep mode****\r\n\r\n");
         illumination_all_off();
         bleNotifyErrorSet(TIME_OUT);
         ble_wait(TEN_SECONDS);
         shutDown();
      }

      if(transmit_reading_at_index != -1)
      {
         struct reading r;
         r = reading_fetch(transmit_reading_at_index);
         uint8 * pointer = &r;

         ble_cmd_attributes_write(BLE_HANDLE_READING, 0, sizeof(struct reading), pointer);       
         transmit_reading_at_index = -1;
         while(!ble_log_process());
      }
      
      switch( iosApp )
      {
         case IOS_REQUEST_START_KETONE_TEST:
            
            iosApp = 0;
         
            //Update bluetooth info
            testResult.stepID = TEST_STARTED;            
            ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *)&testResult.stepID);
            bleNotifyErrorSet(NO_ERROR);                           
            fprintf(STDOUT, "Ketone test started\r\n");        
            performBreathTest( KETONE_TEST );

            break;
         
         case IOS_REQUEST_START_AMMONIA_TEST:
            iosApp = 0;
            //Update bluetooth info
            testResult.stepID = TEST_STARTED;            
            ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *)&testResult.stepID);
            bleNotifyErrorSet(NO_ERROR);                           
            
            performBreathTest( AMMONIA_TEST );
            fprintf(STDOUT, "Ammonia test started\r\n");        

            break;
         
         case IOS_REQUEST_RESET_USE_COUNTER:           
            iosApp = 0;
            //Zero the use counter
            write_ext_eeprom(CONFIG_MOUTHPIECE_USES, 0x0);
            //Update the useCount and the BLE handle
            deviceStat.useCount = 0;
            ble_cmd_attributes_write(BLE_HANDLE_DEVICE_STAT, 0, sizeof(struct deviceInfoPara), (uint8 *)&deviceStat);       
            break;  
            
         case IOS_REQUEST_RESET_READINGS:
            iosApp = 0;
            fprintf(STDOUT, "Reseting readings\r\n");
            readings_reset();
            
            break;

         case IOS_REQUEST_RESET_STATE_MACHINE:           
            int8  ctr;
            uint8_t  *tmpTestResultPtr, *tmpTestStatPtr, *tmpTestTimePtr;
            
            iosApp = 0;
            tmpTestResultPtr = (uint8 *)&testResult;
            tmpTestStatPtr = (uint8 *)&testStat;
            tmpTestTimePtr = (uint8 *)&testTime;
            
            //Update the battery condition and update the device status
            BatteryManagement();
            deviceStat.batteryLevel = BatteryCapacity();
            ble_cmd_attributes_write(BLE_HANDLE_DEVICE_STAT, 0, sizeof(deviceStat), (uint8 *)&deviceStat);
            ble_wait( 150 );
   
            //Clear and update the BLE handle
            for( ctr = 0; ctr < 18; ctr++ )
               tmpTestResultPtr[ctr] = 0;
               
            ble_cmd_attributes_write(BLE_HANDLE_TEST_REPORT, 0, 18, (uint8*)&testResult);
            ble_wait( 150 );
            
            for( ctr = 0; ctr < sizeof(testStat); ctr++ )
               tmpTestStatPtr[ctr] = 0;
               
            ble_cmd_attributes_write(BLE_HANDLE_TEST_STAT, 0, sizeof(testStat), (uint8*)&testStat);
            ble_wait( 150 );               
               
            for( ctr = 0; ctr < sizeof(testTime); ctr++ )
               tmpTestTimePtr[ctr] = 0;
  
            ble_cmd_attributes_write(BLE_HANDLE_TEST_STAGE, 0, sizeof(testTime), (uint8*)&testTime);
            ble_wait( 150 );
            
            ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &testResult.stepID);
            bleNotifyErrorResetErrorBits();
            ble_wait( 150 );

            break;
         
         case IOS_REQUEST_START_FIRMWARE_UPDATE:
            iosApp = 0;
            fprintf(STDOUT, "Firmware update started\r\n");
            //Firmware update code here
            //******************************
            //******************************
            //******************************
            break;
            
         default:
         
      }

      if(kbhit(STDOUT)) 
      {
         uint8 c = fgetc(STDOUT);

         if(c != 13) 
         {
            fputc(c, STDOUT);
            buffer[buffer_index] = c;
            buffer_index++;
         } 
       
         else
         {
            // Enter Key Hit
            fputc('\r', STDOUT);
            fputc('\n', STDOUT);
            buffer[buffer_index] = 0;
            buffer_index = 0;

            for(;;) 
            {
            

#ifdef STATE_TEST            
               if(strcmp(buffer, (unsigned char *)"tt") == 0) 
               {
                  uint8    stateID;
/*
                  stateID = HARDWARE_CHECK;
                  
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Code Hardware Check 0x%x \r\n", stateID);

                  bleNotifyErrorSet(HARDWARE_FAIL);
                  fprintf(STDOUT, "Error Code - Hardware Fail 0x%x \r\n", HARDWARE_FAIL);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
*/
                  stateID = TEST_STARTED;
                  
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Code Start Test 0x%x \r\n", stateID);
                  
                  bleNotifyErrorSet(HARDWARE_FAIL);
                  fprintf(STDOUT, "Error Code - Hardware Fail 0x%x \r\n", HARDWARE_FAIL);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);

                  stateID = INSERT_CARTRIDGE;
                  
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Code Insert Cartridge 0x%x \r\n", stateID);
                  
                  bleNotifyErrorSet(BAD_CATRIDGE);
                  fprintf(STDOUT, "Error Code - Bad Catridge 0x%x \r\n", BAD_CATRIDGE);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
                  
                  stateID = BLOW;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Code Blow 0x%x \r\n", stateID);
                  
                  bleNotifyErrorSet(BAD_BLOW_VOLUME);
                  fprintf(STDOUT, "Error Code - Bad Blow Volume 0x%x \r\n", BAD_BLOW_VOLUME);
                  
                  bleNotifyErrorSet(BAD_BLOW_PRESSURE);
                  fprintf(STDOUT, "Error Code - Bad Blow Pressure 0x%x \r\n", BAD_BLOW_PRESSURE);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
                  
                  stateID = WET;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Code Wet 0x%x \r\n", stateID);
                  
                  bleNotifyErrorSet(NO_WETTING);
                  fprintf(STDOUT, "Error Code - No Wetting 0x%x \r\n", NO_WETTING);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
                  
                  stateID = ANALYZE_BREATH_SAMPLE;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Analyze Breath Sample 0x%x \r\n", stateID);
                  
                  bleNotifyErrorSet(BAD_SCORE_);
                  fprintf(STDOUT, "Error Code - Bad Score 0x%x \r\n", BAD_SCORE_);
                  
                  bleNotifyErrorSet(CARTRIDGE_REMOVED_DURING_ANALYSIS);
                  fprintf(STDOUT, "Error Code - Cartridge removes during analysis 0x%x \r\n", CARTRIDGE_REMOVED_DURING_ANALYSIS);
                  
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                  
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
                  
                  stateID = REMOVE_CARTRIDGE;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Remove Cartridge 0x%x \r\n", stateID);
                                    
                  bleNotifyErrorSet(CARTRIDGE_NOT_REMOVED);
                  fprintf(STDOUT, "Error Code - Cartridge not removes 0x%x \r\n", CARTRIDGE_NOT_REMOVED);
                                    
                  bleNotifyErrorSet(TIME_OUT);
                  fprintf(STDOUT, "Error Code - Timeout 0x%x \r\n", TIME_OUT);
                                    
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No Error 0x%x \r\n", NO_ERROR);
                  
                  stateID = TEST_COMPLETED;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Test Completed 0x%x \r\n", stateID);
                                    
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No error 0x%x \r\n", NO_ERROR);

                  stateID = SEND_RESULTS;
                  ble_cmd_attributes_write(BLE_HANDLE_STATE_CODE, 0, 1, (uint8 *) &stateID);
                  fprintf(STDOUT, "State Send Result 0x%x \r\n", stateID);
                                    
                  bleNotifyErrorSet(NO_ERROR);
                  fprintf(STDOUT, "Error Code - No error 0x%x \r\n", NO_ERROR);

                  break;
               }
#endif            
            
            
#ifdef SOFTWARE_DEBUG            
               if(strcmp(buffer, (unsigned char *)"testStage") == 0) 
               {
                  fprintf(STDOUT, "\r\n**Testing testState Structure**\r\n");
                  testStage();
                  break;
               }
               
               
               if(strcmp(buffer, (unsigned char *)"testStats") == 0) 
               {
                  fprintf(STDOUT, "\r\n**Testing testState Structure**\r\n");
                  testStats();
                  break;
               }
               
               if(strcmp(buffer, (unsigned char *)"testReport") == 0) 
               {
                  fprintf(STDOUT, "\r\n**Testing testReport Structure**\r\n");
                  testReport();
                  break;
               }
#endif               
               
               if(strcmp(buffer, (unsigned char *)"pd") == 0)
               {
                  fprintf(STDOUT, "\r\n****Sleep Mode****\r\n");
                  shutDown();
                  break;
               }
               
               if(strcmp(buffer, (unsigned char *)"help") == 0) 
               {
                  fprintf(STDOUT, " \r\n\r\n  Type the following command \r\n" );
                  fprintf(STDOUT, "------------------------------\r\n\r\n" );
                  fprintf(STDOUT, "  bright730=  - set the led 730 brightness \r\n");
                  fprintf(STDOUT, "  bright588=  - set the led 588 brightness \r\n");
                  fprintf(STDOUT, "  bright475=  - set the led 475 brightness \r\n");
                  fprintf(STDOUT, "  bt          - get the led brightness values \r\n");
                  fprintf(STDOUT, "  calibrate   - calibrate the unit \r\n");
                  fprintf(STDOUT, "  ct          - check the cartridge condition \r\n");
                  fprintf(STDOUT, "  ft          - perform ketone test \r\n");
                  fprintf(STDOUT, "  identify    - identify unit \r\n");
                  fprintf(STDOUT, "  ble.reset   - reset bluetooth \r\n");
                  fprintf(STDOUT, "  intercept=  - set the intercept value \r\n");
                  fprintf(STDOUT, "  intercept   - get the intercept value \r\n");
                  fprintf(STDOUT, "  play        - check the sound \r\n");
                  delay_ms(50);
                  fprintf(STDOUT, "  rp          - print all save test results \r\n");
                  fprintf(STDOUT, "  rr          - clear all save test results \r\n");
                  fprintf(STDOUT, "  rs          - auto generate 10 test results for testing purpose  \r\n");
                  fprintf(STDOUT, "  sbv         - set new breath volume \r\n");
                  fprintf(STDOUT, "  selftest    - perform a self test \r\n");
                  fprintf(STDOUT, "  serial=     - set the unit serial number \r\n");
                  fprintf(STDOUT, "  serial      - get the unit serial number \r\n");
                  fprintf(STDOUT, "  setled      - set the led brightness automatically \r\n");
                  fprintf(STDOUT, "  slope=      - set the slope value \r\n");
                  fprintf(STDOUT, "  slope       - get the slope value \r\n");
                  fprintf(STDOUT, "  time=       - set the time and date \r\n");
                  fprintf(STDOUT, "  voltest     - compute breath volume \r\n");
                  
                  fprintf(STDOUT, two_new_lines_resp);
                  fprintf(STDOUT, two_new_lines_resp);
                  break;
               }
         
               if(strcmp(buffer, (unsigned char *)"voltest") == 0) 
               {
                  rslt = Compute_Breath_Volume( &gas_sensor );
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"selftest") == 0) 
               {
                  selfTest();
                  break;
               }
               
               if(strcmp(buffer, (unsigned char *)"identify") == 0) 
               {
                  fprintf(STDOUT, "Voyager 4.0 Prototype,R%d.%d.%d,",
                     FIRMWARE_VERSION_MAJOR,
                     FIRMWARE_VERSION_MINOR,
                     FIRMWARE_VERSION_REVISION);

                  uint8 serial[6];
                  config_serial_get(serial);
   
                  for(uint8 i = 0; i < 6; i++) 
                  {
                     fprintf(STDOUT, "%u", serial[i]);
                  }

                  fprintf(STDOUT, ",%2X%2X%2X%2X%2X%2X\r\n", BleModuleId[0], BleModuleId[1], BleModuleId[2], BleModuleId[3], BleModuleId[4], BleModuleId[5]);
            
                  break;
               }
#ifdef BASE_40
               if(strcmp(buffer, (unsigned char *)"testmode") == 0) 
               {
                  if (TestMode == 0) 
                  {
                     TestMode = 1;
                     fprintf(STDOUT, "!!!! TEST MODE ACTIVATED !!!!\r\n\r\n");
                  }
            
                  else 
                     fprintf(STDOUT, "[ERROR] Already in TEST MODE!\r\n");
               
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"normalmode") == 0) 
               {
                  if (TestMode != 0) 
                  {
                     TestMode = 0;
                     fprintf(STDOUT, "Back to normal operating mode\r\n\r\n");
                  }
               
                  else 
                  {
                     fprintf(STDOUT, "[ERROR] TEST MODE was not active!\r\n");
                  }  
               
                  break;
               }
#endif               
         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"ble.reset") == 0) 
               {
                  ble_configure();
                  fprintf(STDOUT, "BLE Re-Activated\r\n");
                  break;
               }
               
               if(strcmp(buffer, (unsigned char *)"sbv") == 0) 
               {
                  tmpValue = get_Int16();
                  fprintf(STDOUT, "\r\nReset breath volume new value : %ld \r\n", tmpValue );
                  setBreathVolume( tmpValue );
                  fprintf(STDOUT, "\r\nBreath volume settings: %ld \r\n", breathVolume );
                  break;
               }

         ////////////////////////////////////////////////
#ifdef BASE_40
               if(strcmp(buffer, (unsigned char *)"temperature") == 0) 
               {
//                ShowTemperature();
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"ble.address") == 0) 
               {
                  fprintf(STDOUT, "\r\nBLE Address = %2X%2X%2X%2X%2X%2X\r\n", BleModuleId[0], BleModuleId[1], BleModuleId[2], BleModuleId[3], BleModuleId[4], BleModuleId[5]);
                  break;
               }
#endif

               if(strcmp(buffer, (unsigned char *)"serial=") == 0) 
               {
                  uint8 serial[6];

                  for(uint8 i = 0; i < 6; i++) 
                  {
                     fprintf(STDOUT, "\r\nSerial %u of 6: ", i);
                     serial[i] = get_Int8();
                  }

                  fprintf(STDOUT, "\r\n");
                  config_serial_set(serial);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"serial") == 0) 
               {
                  uint8 serial[6];
                  config_serial_get(serial);
                  fprintf(STDOUT, "Serial: ");
                  
                  for(uint8 i = 0; i < 6; i++) 
                     fprintf(STDOUT, "%u", serial[i]);

                  fprintf(STDOUT, "\r\n");
                  break;
               }

        ////////////////////////////////////////////////
#ifdef BASE_40
               if(strcmp(buffer, (unsigned char *)"mouthpiece") == 0) 
               {  
                  config_mouthpiece_get();
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"mouthpiece=") == 0) 
               {
                  fprintf(STDOUT, "Mouthpiece Uses: ");
                  uint8 value = get_Int8();
                  config_mouthpiece_set(value);
                  fprintf(STDOUT, "\r\n");
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"mouthpiece+") == 0) 
               {
                  config_mouthpiece_increment();
                  fprintf(STDOUT, "Mouthpiece Uses Incremented\r\n");
                  break;
               }
#endif        
        ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"intercept=") == 0) 
               {
                  ConfigWriteFloat(CONFIG_INTERCEPT, prompt_float());
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"intercept") == 0) 
               {
                  fprintf(STDOUT, "Intercept: %.3f\r\n", ConfigReadFloat(CONFIG_INTERCEPT));
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"slope=") == 0) 
               {
                  ConfigWriteFloat(CONFIG_SLOPE, prompt_float());
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"slope") == 0) 
               {
                  fprintf(STDOUT, "Slope: %.3f\r\n", ConfigReadFloat(CONFIG_SLOPE));
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"config.enable.dynamicwb") == 0) 
               {
                  write_ext_eeprom(CONFIG_DYNAMIC_WHITEBALANCE_ENABLED, 1);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"config.disable.dynamicwb") == 0) 
               {
                  write_ext_eeprom(CONFIG_DYNAMIC_WHITEBALANCE_ENABLED, 0);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"config.check.dynamicwb") == 0) 
               {
                  fprintf(STDOUT, "Dyanamic White Balance Enabled: %d\r\n", read_ext_eeprom(CONFIG_DYNAMIC_WHITEBALANCE_ENABLED));
                  break;
               }

         ////////////////////////////////////////////////
#ifdef BASE_40
               if(strcmp(buffer, (unsigned char *)"reading.new") == 0) 
               {
                  fprintf(STDOUT, "Reading Value: ");
                  float32 value = get_float();

                  testResult.score = f_PICtoIEEE(value);
                  testResult.stepID = FULLTEST_UNKNOWN;
                  reading_save( &testResult, &cmdReceived.time );

                  fprintf(STDOUT, "\r\n");
                  break;
               }
#endif
               if( (strcmp(buffer, (unsigned char *)"reading.simulate") == 0) || (strcmp(buffer, (unsigned char *)"rs") == 0) )
               {

                  reading_simulate();
                  fprintf(STDOUT, "\r\n");
                  break;
               }

               if( (strcmp(buffer, (unsigned char *)"readings.print") == 0) || (strcmp(buffer, (unsigned char *)"rp") == 0) )
               {
                  readings_print();
                  break;
               }  

               if( (strcmp(buffer, (unsigned char *)"readings.reset") == 0) || (strcmp(buffer, (unsigned char *)"rr") == 0) )
               {
                  fprintf(STDOUT, "Reseting readings\r\n");
                  readings_reset();
                  break;
               }

         ////////////////////////////////////////////////

               if((strcmp(buffer, (unsigned char *)"cartridge") == 0) || (strcmp(buffer, (unsigned char *)"ct") == 0)) 
               {
                  check_bad_or_used_cartridge();
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"play") == 0) 
               {
                  fprintf(STDOUT, "Jingle bells\r\n");
                  play();
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"bright730=") == 0) 
               {
                  ConfigWriteByte(CONFIG_LED730_BRIGHTNESS_ADDRESS, get_Int8());   
                  fprintf(STDOUT, "\r\n");
                  LedProperties.LevelBrightness730 = ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"bright588=") == 0) 
               {
                  ConfigWriteByte(CONFIG_LED588_BRIGHTNESS_ADDRESS, get_Int8());
                  fprintf(STDOUT, "\r\n");
                  LedProperties.LevelBrightness588 = ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"bright475=") == 0) 
               {
                  ConfigWriteByte(CONFIG_LED475_BRIGHTNESS_ADDRESS, get_Int8());
                  fprintf(STDOUT, "\r\n");
                  LedProperties.LevelBrightness475 = ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS);
                  break;
               }

               if((strcmp(buffer, (unsigned char *)"brightness") == 0) || (strcmp(buffer, (unsigned char *)"bt") == 0)) 
               {
                  get_LED_Brightness();
                  break;
               }
         
               if(strcmp(buffer, (unsigned char *)"led475") == 0) 
               {
                  illumination_brightness(ConfigReadByte(CONFIG_LED475_BRIGHTNESS_ADDRESS));
                  fprintf(STDOUT, "LED 475 on\r\n");
                  illumination_use(ILLUMINATION_475);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"led588") == 0) 
               {
                  illumination_brightness(ConfigReadByte(CONFIG_LED588_BRIGHTNESS_ADDRESS));
                  fprintf(STDOUT, "LED 588 on\r\n");
                  illumination_use(ILLUMINATION_588);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"led730") == 0) 
               {
                  illumination_brightness(ConfigReadByte(CONFIG_LED730_BRIGHTNESS_ADDRESS));
                  fprintf(STDOUT, "LED 730 on\r\n");
                  illumination_use(ILLUMINATION_730);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"ledoff") == 0) 
               {
                  fprintf(STDOUT, "LED off\r\n");
                  illumination_all_off();
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"read1") == 0) 
               {
                  fprintf(STDOUT, "Reading: %lu\r\n", photodiode());
                  break;
               }
               
               if(strcmp(buffer, (unsigned char *)"read10") == 0)
               {
                  int32 sum = 0;
                  int32 count = 10;
               
                  for(int i = 0; i < count; i++) 
                  {
                     sum += photodiode();
                     delay_ms(20);
                  }

                  fprintf(STDOUT, "Reading (10-avg): %.4f\r\n", (double)sum/(double)count);
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"readcsv") == 0) 
               {
                  float32 con;
                  readings_csv(&con);
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"calibrate") == 0) 
               {
                  fprintf(STDOUT, "\r\n");
                  ShowPrompt();
                  fprintf(STDOUT, "Executing calibration...\r\n");
                  status = performBreathTest( CALIBRATION_TEST  );
                  break;
               }

         ////////////////////////////////////////////////

               if( (strcmp(buffer, (unsigned char *)"fulltest") == 0) || (strcmp(buffer, (unsigned char *)"ft") == 0) ) 
               {
                  status = performBreathTest( KETONE_TEST );
                  
                  if( status != PASS )
                     fprintf(STDOUT, "******  Ketone Detection Test failed  ******\r\n");
               
                  break;
               }

         ////////////////////////////////////////////////

               if(strcmp(buffer, (unsigned char *)"reading") == 0) 
               {
   
                  fprintf(STDOUT, "Duration: ");
                  int16 value = get_Int16();
   
                  fprintf(STDOUT, "\r\nbright730: ");
                  int16 brightness730 = get_Int16();

                  fprintf(STDOUT, "\r\nbright588: ");
                  int16 brightness588 = get_Int16();

                  fprintf(STDOUT, "\r\nbright475: ");
                  int16 brightness475 = get_Int16();
                  fprintf(STDOUT, "\r\nindex, LED730, LED588, LED475\r\n");
   
                  for(int16 i = 0; i < value / 3; i++) 
                  {
                     illumination_brightness(brightness730);
                     illumination_use(ILLUMINATION_730);
                     ble_wait(45);
                     int16 led730 = photodiode();
                     ble_wait(5);
  
                     illumination_brightness(brightness588);
                     illumination_use(ILLUMINATION_588);
                     ble_wait(45);
                     int16 led588 = photodiode();
                     ble_wait(5);

                     illumination_brightness(brightness475);
                     illumination_use(ILLUMINATION_475);
                     ble_wait(45);
                     int16 led475 = photodiode();
                     ble_wait(5);

                     illumination_all_off();
                     delay_ms(2850);
   
                     int16 row = i * 3;
                     fprintf(STDOUT, "%lu, %lu, %lu, %lu\r\n", row, led730, led588, led475);
                  }
           
                  break;
               }

               if(strcmp(buffer, (unsigned char *)"setled") == 0)
               {
                  int16 bt730, bt588, bt475;
            
                  bt730 = bt588 = bt475 = LED_BRIGHTNESS_DEFAULT_VALUE;
                  
                  fprintf(STDOUT, "\r\nSet LED brightness index\r\n");
                  fprintf(STDOUT, "****************************\r\n");
                  status = autoSetLEDBrightness(bt730, bt588, bt475);
            
                  if( status != PASS )
                     fprintf(STDOUT, "***** Auto brightness setting for LED730, LED588, LED475 failed *****\r\n");

                  break;
               }

               if(strcmp(buffer, (unsigned char *)"caltest") == 0)
               {
                  Calibrate();
                  break;
               }
              
               break;
            }
         
            ////////////////////////////////////////////////
            powerDownTimeCtr = INACTIVITY_TIME_LIMIT;          //Reset power down counter to 5 minutes 
            tmpCtr = 0;
            ShowPrompt();
         }
      }
   }
}
