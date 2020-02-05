#ifndef __ERROR_CODES__
#define __ERROR_CODES__

#include "datatype.h"



enum ErrorBitsDefinition
{
//   LIS3DH_MALFUNCTION = 0,
   BME680_MALFUNCTION = 0,
   EEPROM_MALFUNCTON,
   LED_MALFUNCTION,
   
   ERROR_UNKNOWN = 0XFF
};

//Result Errors Code
#define MYSTATUS_OK              0x0
#define LOW_PRESSURE_BREATH      0X3

/* Device Errors Code*/
//#define LIS3DH_FAIL                       2
#define BLE_FAIL                          4
#define EEPROM_FAIL                       8
#define BME680_SENSOR_FAIL                16
#define LED_FAIL                          32
#define XXXXX_FAIL                        64
#define NULL_POINTER_DETECTED             128

extern int8 ErrorBits;
extern int8 LastErrorBits;

#endif

