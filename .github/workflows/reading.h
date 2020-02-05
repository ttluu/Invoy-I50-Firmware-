#ifndef __READING__
#define __READING__

#include "datatype.h"

#define READING_ERROR_NONE                   0
#define READING_ERROR_TRAY_OPENED            1
#define READING_ERROR_CARTRIDGE_REMOVED      2
#define READING_ERROR_BOTTLE_DOOR_OPENED     3
#define READING_ERROR_CARTRIDGE_FORGOTTEN    4
#define READING_ERROR_MODULE_NOT_STRAIGHT    5
#define READING_ERROR_BAD_TEMPERATURE        6
#define READING_ERROR_BAD_ORIENTATION        7
#define READING_ERROR_BAD_READING            8

#ifdef BASE_40
// Total Size: 12
struct reading 
{
  uint32 timestamp;

  // NOTE: WARNING. This is actually a float.
  // However the PIC processor does not use IEEE-754 floats
  // so we have to convert manually.
  int32 value;
  uint16 status;
  uint16 FullTestStepID;
};

void reading_save(uint32 timestamp, float32 value, uint16 status, uint16 fullTestStepID);
#endif

// Total Size: 18
struct reading 
{
   struct testResultPara   testResult;
   struct timePara         time;
   // NOTE: WARNING. This is actually a float.
   // However the PIC processor does not use IEEE-754 floats
   // so we have to convert manually.

};

void readings_reset();
uint16 reading_index_start();
uint16 reading_index_stop();
struct reading reading_fetch(uint16 index);
void reading_save(struct testResultPara *testResult, struct timePara *timestamp);
void reading_simulate();

uint16 SetFullTestStepID(uint16 stepID);

#endif

