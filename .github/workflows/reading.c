#include <reading.h>
//#include <rtc.h>
#include <ble.h>
//#include <Temperature.h>
//#include <Orientation.h>
#include <Peripherals.h>
#include <ErrorCodes.h>

uint16 LastFullStepID = SEND_RESULTS;

////////////////////////////////////////
void readings_reset() 
{
   //Zerorize current index
   write_ext_eeprom(CONFIG_READING_INDEX_START + 0, 0);
   write_ext_eeprom(CONFIG_READING_INDEX_START + 1, 0);

   write_ext_eeprom(CONFIG_READING_INDEX_STOP + 0, 0);
   write_ext_eeprom(CONFIG_READING_INDEX_STOP + 1, 0);

   struct reading r;
   uint8  ctr;
   uint8  *ptr = (uint8 *)&r;
  
   //Clear the r structure
   for( ctr = 0; ctr < sizeof( r ); ctr++ )
      ptr[ctr] = 0;

   uint16 a = CONFIG_READING_BASE;
  
   for(uint8 i = 0; i < CONFIG_READING_COUNT; i++) 
   {
      for(int j = 0; j < sizeof(struct reading); j++) 
         write_ext_eeprom(a++, ptr[j]);
   }

   uint8 message = ble_data_message_count;
   a = 0;
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_START, 0, 2, (uint8 *)&a);
   ble_wait_for_message(message);

   message = ble_data_message_count;
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_STOP, 0, 2, (uint8 *)&a);
   ble_wait_for_message(message);
   
   fprintf(STDOUT, "Start Index: %lu. Stop Index: 0x%lu.\r\n", reading_index_start(), reading_index_stop());
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
uint16 reading_index_start() 
{
   uint16 records_start = 0;
   uint8 * pointer = &records_start;
   pointer[0] = read_ext_eeprom(CONFIG_READING_INDEX_START + 0);
   pointer[1] = read_ext_eeprom(CONFIG_READING_INDEX_START + 1);

   return records_start;
}

////////////////////////////////////////
uint16 reading_index_stop() 
{
   uint16 records_stop = 0;
   uint8 * pointer = &records_stop;
   pointer[0] = read_ext_eeprom(CONFIG_READING_INDEX_STOP + 0);
   pointer[1] = read_ext_eeprom(CONFIG_READING_INDEX_STOP + 1);

   return records_stop;
}

////////////////////////////////////////
void reading_save(struct testResultPara *testResult, struct timePara *timestamp)
{
   struct reading r;
  
   r.time.second = timestamp->second;
   r.time.minute = timestamp->minute;
   r.time.hour = timestamp->hour;
   r.time.day = timestamp->day;
   r.time.month = timestamp->month;
   r.time.year = timestamp->year;
   
   r.testResult.testMode = testResult->testMode;
   r.testResult.totalTestTime = testResult->totalTestTime;
   r.testResult.score = testResult->score;
   
   r.testResult.status = testResult->status;
   r.testResult.stepID = testResult->stepID;
   
//   uint8 message = 0;
   uint8 * pointer;

   uint16 records_start = reading_index_start();
   uint16 records_stop = reading_index_stop();
   uint16 reading_base = CONFIG_READING_BASE + ( records_stop * sizeof(r) );

   pointer = (uint8 *) &r;
   for(uint16 i = 0; i < sizeof(struct reading); i++) 
      write_ext_eeprom(reading_base + i, pointer[i]);

   records_stop++;
   if(records_stop == CONFIG_READING_COUNT) 
      records_stop = 0;

   if(records_start >= records_stop) 
   {
      records_start++;
      
      if(records_start == CONFIG_READING_COUNT) 
         records_start = 0;
    
   }

   pointer = (uint8 *) &records_start;
   write_ext_eeprom(CONFIG_READING_INDEX_START + 0, pointer[0]);
   write_ext_eeprom(CONFIG_READING_INDEX_START + 1, pointer[1]);

   pointer = (uint8 *) &records_stop;
   write_ext_eeprom(CONFIG_READING_INDEX_STOP + 0, pointer[0]);
   write_ext_eeprom(CONFIG_READING_INDEX_STOP + 1, pointer[1]);

//   message = ble_data_message_count;
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_START, 0, 2, (uint8 *)&records_start);
//   ble_wait_for_message(message);

//   message = ble_data_message_count;
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_STOP, 0, 2, (uint8 *)&records_stop);
//   ble_wait_for_message(message);

   fprintf(STDOUT, "Start Index: %lu. Stop Index: 0x%lu.\r\n", reading_index_start(), reading_index_stop());
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
struct reading reading_fetch(uint16 index) 
{
   index = index % CONFIG_READING_COUNT;
   uint16 reading_base = CONFIG_READING_BASE + (index * sizeof(struct reading));

   struct reading r;
   uint8 * pointer = &r;

   for(uint8 i = 0; i < sizeof(struct reading); i++) 
      pointer[i] = read_ext_eeprom(reading_base + i);
  
   return r;
}

 ////////////////////////////////////////
void reading_simulate()
{
   struct testResultPara simulateTestResult;
   struct timePara simulateTimestamp;
   int8     ctr;
   
   int16    testMode = 1;
   uint32   totalTestTime = 25000;
   float    score = 13.1154;
   uint8    status = NO_ERROR;
   uint8    stepID = TEST_COMPLETED;
   
   uint8   hour = 1;
   uint8   minute = 10;
   uint8   second = 10;
   uint8   day = 12;
   uint8   month = 1;
   uint8   year = 20;
   
   simulateTestResult.testMode = testMode;
   
   simulateTestResult.totalTestTime = totalTestTime;
   simulateTestResult.status = status;
   simulateTestResult.stepID = stepID;
   
   simulateTimestamp.hour = hour;
   simulateTimestamp.minute = minute;
   simulateTimestamp.second = second;
   simulateTimestamp.day = day;
   simulateTimestamp.month = month;
   simulateTimestamp.year = year;
   
   fprintf(STDOUT, "Simulate and save 10 test results\r\n");
   
   for( ctr = 0; ctr < 10; ctr++ )
   {
      simulateTestResult.totalTestTime += 1500;
      simulateTimestamp.hour += 1;
      simulateTimestamp.minute += 3;
      simulateTimestamp.second += 4;
      simulateTimestamp.day += 1;
      simulateTimestamp.month += 1;
      score += .232;
      simulateTestResult.score = f_PICtoIEEE(score + ctr);
      reading_save( &simulateTestResult, &simulateTimestamp );
      delay_ms(50);
      fprintf(STDOUT, "%d record\r\n", ctr);
   }
    
   fprintf(STDOUT, "Start Index: %lu. Stop Index: %lu.\r\n", reading_index_start(), reading_index_stop());
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
void readings_print()
{
   struct reading r;
   float  score;
   uint8 i = 0;

   fprintf(STDOUT, "Start Index: %lu. Stop Index: %lu.\r\n", reading_index_start(), reading_index_stop());
   fprintf(STDOUT, new_line_resp);


   fprintf(STDOUT, "Record# Test Mode Test Time Test Score Test Status Test State Time Stamp");
   fprintf(STDOUT, two_new_lines_resp);
   
   for(i = 0; i < CONFIG_READING_COUNT; i++) 
   {
      r = reading_fetch(i);
      score = f_IEEEtoPIC(r.testResult.score);
      
      fprintf(STDOUT, "  %d         %ld       %lu         %.4f\t %d\t      %d", i, r.testResult.testMode, r.testResult.totalTestTime / 1000, score, r.testResult.status, r.testResult.stepID );
      fprintf(STDOUT, "       %2d:%2d:%2d, D %2d M %2d Y %2d ", r.time.hour, r.time.minute, r.time.second, r.time.day, r.time.month, r.time.year );
      fprintf(STDOUT, new_line_resp);

#ifdef TEST     
      
      uint8 c, *ptr;
      ptr = &r;
      fprintf(STDOUT, "0x");
      
      for( c = 0; c < sizeof(r); c++ )
         fprintf(STDOUT, "%x", ptr[c]);
      
      fprintf(STDOUT, new_line_resp);
#endif

   }
  
   fprintf(STDOUT, new_line_resp);
}

////////////////////////////////////////
void reading_in_progress(uint16 progress) 
{
   uint8 message = ble_data_message_count;
   ble_cmd_attributes_write(BLE_HANDLE_READING_IN_PROGRESS, 0, 2, (uint8 *)&progress);
   ble_wait_for_message(message);
}

////////////////////////////////////////
uint16 SetFullTestStepID(uint16 stepID) 
{
   LastFullStepID = stepID;
   reading_in_progress(stepID);
   return stepID;
}
