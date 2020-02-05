#include <reading.h>
#include <ble.h>
#include "datatype.h"

extern uint8   bluetoothState;
extern uint16  breathVolume;
extern struct  commandPara   cmdReceived;

extern struct testResultPara  testResult;
extern struct statusPara      testTime;
extern struct deviceInfoPara  deviceStat;
extern struct testInfoPara    testStat;



uint8 ble_log_buffer[256];
uint8 ble_log_buffer_head = 0;
uint8 ble_log_buffer_tail = 0;
uint8 BleModuleId[BLE_MODULE_ID_SIZE] = {0x01, 0x02, 0x0E, 0x0C, 0x0A, 0x0F} ;

int8 ErrorBits = 0;
int8 LastErrorBits = 0;

////////////////////////////////////////////////////////////////////////////////
void ble_log(char * message, uint8 length) 
{

#ifdef BASE_40
   //Note- Sum of this data type will never be greater than 256
   if (ble_log_buffer_tail + length > 256) 
      return;
#endif

   for(uint8 i = 0; i < length; i++)
   {
      ble_log_buffer[ble_log_buffer_tail] = message[i];
      ble_log_buffer_tail++;
   }
}

////////////////////////////////////////////////////////////////////////////////
uint8 ble_log_process() 
{
  uint8 did_work = 0;

  while(ble_log_buffer_head < ble_log_buffer_tail) 
  {
    did_work = 1;

    //fputc(ble_log_buffer[ble_log_buffer_head], STDOUT);
    ble_log_buffer_head++;
    if (ble_log_buffer_head == ble_log_buffer_tail)
    {
       ble_log_buffer_head = 0;
       ble_log_buffer_tail = 0;
    }
  }

  return did_work;
}

////////////////////////////////////////////////////////////////////////////////
uint8 ble_header_equals(struct ble_header * hdr, uint8 type_hilen, uint8 lolen, uint8 cls, uint8 command)
{
   return (hdr->type_hilen == type_hilen && hdr->cls == cls && hdr->command == command);
}

////////////////////////////////////////////////////////////////////////////////
void ble_cmd_system_hello()
{
   fputc(0x04, BLE);

   fputc(0x00, BLE);
   fputc(0x00, BLE);
   fputc(0x00, BLE);
   fputc(0x01, BLE);
}

////////////////////////////////////////////////////////////////////////////////
void ble_cmd_system_AddressGet()
{
   fputc(0x04, BLE);

   fputc(0x00, BLE);
   fputc(0x00, BLE);
   fputc(0x00, BLE);
   fputc(0x02, BLE);
}

////////////////////////////////////////////////////////////////////////////////
void ble_cmd_gap_set_mode(uint8 discover, uint8 connect)
{
   fputc(0x06, BLE);

   fputc(0x00, BLE);
   fputc(0x02, BLE);
   fputc(0x06, BLE);
   fputc(0x01, BLE);

   fputc(discover, BLE);
   fputc(connect, BLE);
}

////////////////////////////////////////////////////////////////////////////////
void ble_cmd_attributes_write(uint16 handle, uint8 offset, uint8 length, uint8 *value)
{
   fputc(0x04 + 0x4 + length, BLE);

   fputc(0x00, BLE);
   fputc(0x04 + length, BLE);
   fputc(0x02, BLE);
   fputc(0x00, BLE);

   fputc(handle & 0xff, BLE);
   handle >>= 8;
   fputc(handle & 0xff, BLE);

   fputc(offset, BLE);
   fputc(length, BLE);

   while(length--) 
   {
      fputc(*value, BLE);
      value++;
   }
}

////////////////////////////////////////////////////////////////////////////////
void ble_wait(int32 timeout)
{
   int32 timer_current = 0;
   int32 timer_previous = 0;
   int32 timer_difference = 0;

   timer_previous = get_ticks();
   
   while(TRUE) 
   {
      timer_current = get_ticks();
      timer_difference = timer_current - timer_previous;
      
      if(timer_difference > timeout)
         break;

      ble_log_process();
   }
}

////////////////////////////////////////////////////////////////////////////////
uint8 ble_data_buffer[256];
uint8 ble_data_index = 0;
uint8 ble_data_message_count = 0;
uint8 iosApp = 0;
uint8 FullTestRunning = 0;

int16 transmit_reading_at_index = -1;

////////////////////////////////////////////////////////////////////////////////

void ble_wait_for_message(uint8 start)
{
  while(start == ble_data_message_count);
}

////////////////////////////////////////////////////////////////////////////////
void ble_process() 
{
   struct   ble_header * hdr = &ble_data_buffer;
   uint8 *  data = &ble_data_buffer + 4;
   uint8 *  srcPtr, *destPtr, cmd, tmpCtr;
   uint16   tmpVolume;
   char message[256];
   uint8 length = 0;
   
   //incoming data
   //srcPtr = data;
   //Skip the header
   //srcPtr += 6;
   
   destPtr = &cmdReceived;

   if(ble_header_equals(hdr, 0x80, 0x00, 0x00, 0x00)) 
   {
      // struct ble_msg_system_boot_evt_t *evt = data;
      length = sprintf(message, "\rBLE Boot Event\r\n");
   }
  
   else if(ble_header_equals(hdr, 0x00, 0x00, 0x00, 0x01)) 
   {
      length = sprintf(message, "\rBLE Hello\r\n");
   } 
  
   else if(ble_header_equals(hdr, 0x80, 0x10, 0x03, 0x00)) 
   {
      length = sprintf(message, "\rBLE Connection\r\n");
   } 
  
   else if(ble_header_equals(hdr, 0x80, 0x03, 0x03, 0x04)) 
   {
      length = sprintf(message, "\rBLE Disconnection\r\n");
      ble_cmd_gap_set_mode(2, 2);
   } 
  
   else if(ble_header_equals(hdr, 0x00, 0x02, 0x06, 0x01)) 
   {
      uint16 *ptr = data;
      length = sprintf(message, "\rBLE Mode Changed. Error = %lu\r\n", *ptr);
   } 
  
   else if(ble_header_equals(hdr, 0x80, 0x03, 0x02, 0x02)) 
   {
      length = sprintf(message, "\rBLE Connection Status Changed: {%u, %u, %u}\r\n", data[0], data[1], data[2]);
   } 
  
   else if(ble_header_equals(hdr, 0x00, 0x02, 0x02, 0x00)) 
   {
      uint16 *ptr = data;
      length = sprintf(message, "\rBLE Write Response. Length = %u. Result = %lu\r\n", hdr->lolen, *ptr);
      ble_data_message_count++;
   } 
  
   else if(ble_header_equals(hdr, 0x80, 0x07, 0x02, 0x00)) 
   {
      length = sprintf(message, "\rBLE Value Written. Length = %u.\r\n", hdr->lolen);
//    ble_log(message, length);

      struct ble_msg_attributes_value_evt_t *evt = data;
      uint16 handle = evt->handle;
      uint16 * value_ptr = NULL;
      uint16 value = 0;

      switch(handle) 
      {
         case BLE_HANDLE_INDEX_CURRENT:
            //value_ptr = &evt->value.data[0];
            //value = *value_ptr;
            value = evt->value.data[0];
            length = sprintf(message, "\rFetching index: %lu\r\n", value);
//        ble_log(message, length);

            transmit_reading_at_index = value;
            break;
         
         case BLE_HANDLE_RX:
            value_ptr = &evt->value.data[0];
            value = *value_ptr;

            length = sprintf(message, "\rValue has changed: %lu!\r\n", value);
//        ble_log(message, length);

            value += 1;
            length = sprintf(message, "\rValue incremented. Should be: %lu!\r\n", value);
//        ble_log(message, length);
            uint8 offset = 0;
            uint8 value_length = sizeof(value);
            uint16 tx_handle = BLE_HANDLE_TX;
            fputc(0x04 + 0x4 + value_length, BLE);
          
            fputc(0x00, BLE);
            fputc(0x04 + value_length, BLE);
            fputc(0x02, BLE);
            fputc(0x00, BLE);
          
            fputc(tx_handle & 0xff, BLE);
            tx_handle >>= 8;
            fputc(tx_handle & 0xff, BLE);
          
            fputc(offset, BLE);
            fputc(value_length, BLE);
          
            while(value_length--) 
            {
               fputc(*value, BLE);
               value++;
            }

            break;
        
         case BLE_HANDLE_OPCODE:
            srcPtr = &evt->value.data[0];
            
            for( tmpCtr = 0; tmpCtr < sizeof( cmdReceived ); tmpCtr++ )
               destPtr[tmpCtr] = srcPtr[tmpCtr];
               
#ifdef DEBUG           
            fprintf(STDOUT, "Opcode Number received: %d \r\n", cmdReceived.opCode);
            fprintf(STDOUT, "Volume Settings received : %lu \r\n", cmdReceived.volume);
            fprintf(STDOUT, "Hour received   : %lu \r\n", cmdReceived.time.hour);
            fprintf(STDOUT, "Minute received   : %lu \r\n", cmdReceived.time.minute);
            fprintf(STDOUT, "Second received : %lu \r\n", cmdReceived.time.second);
            fprintf(STDOUT, "Day received   : %lu \r\n", cmdReceived.time.day);
            fprintf(STDOUT, "Month received   : %lu \r\n", cmdReceived.time.month);
            fprintf(STDOUT, "Year received : %lu \r\n", cmdReceived.time.year);
            fprintf(STDOUT, "\r\n\r\n");
 #endif           

            cmd = cmdReceived.opCode;
            tmpVolume = cmdReceived.volume;
            
            if( tmpVolume < MININUM_BREATH_VOLUME || tmpVolume > MAXINUM_BREATH_VOLUME )
               tmpVolume = 0;
            
            //check for breath volume setting
            if( tmpVolume > MININUM_BREATH_VOLUME || tmpVolume < MAXINUM_BREATH_VOLUME )
            {  //Update with new breath volume setting
               ConfigWriteFloat( CONFIG_BREATH_VOLUME, tmpVolume );
               fprintf(STDOUT, "New volume settings: %lu \r\n", tmpVolume);
            }
               
            length = sprintf(message, "\riOS App opcode%d\r\n", cmd);
//        ble_log(message, length);

            if (!FullTestRunning) 
            {
               switch(cmd) 
               {
                  case OPCODE_BLE_START_KETONE_TEST:
                     iosApp = IOS_REQUEST_START_KETONE_TEST;
                     length = sprintf(message, "\rKetone test cmd received.\r\n");

                     break;
          
                  case OPCODE_BLE_START_AMMONIA_TEST:
                     iosApp = IOS_REQUEST_START_AMMONIA_TEST;
                     length = sprintf(message, "\rAmmonia test cmd received.\r\n");
                  
                     break;
          
                  case OPCODE_RESET_USE_COUNTER:
                     iosApp = IOS_REQUEST_START_AMMONIA_TEST;
                     length = sprintf(message, "\rReset use counter cmd received\r\n");

                     break;

                  case OPCODE_RESET_READINGS:
                     iosApp = IOS_REQUEST_RESET_READINGS;
                     length = sprintf(message, "\rClear memory cmd received.\r\n");
                  
                     break;

                  case OPCODE_BLE_START_FIRMWARE_UPDATE:
                     iosApp = IOS_REQUEST_START_FIRMWARE_UPDATE;
                     length = sprintf(message, "\rFirmware update cmd detected.\r\n");
                  
                     break;
                     
                  case OPCODE_RESET_STATE_MACHINE:
                     iosApp = IOS_REQUEST_RESET_STATE_MACHINE;
                     length = sprintf(message, "\rReset State Machine cmd detected.\r\n");
                  
                     break;

                  default:
                     length = sprintf(message, "\r[NOTE] Unsupported cmd detected\r\n");
//                     ble_log(message, length);   
               }

               // Prevent the ble_log at the end of the function below.
               length = 0;
            } 
            
            break;
      }
   }
  
   else if(ble_header_equals(hdr, 0x00, 0x06, 0x00, 0x02)) 
   { // Response to ADDRESS_GET command
      uint16 *ptr = data;
      
      for(int i = 0; i < 6; i++ )
         BleModuleId[i] = ptr[i];

      length = sprintf(message, "\r\nBLE Module ID captured %2X%2X%2X%2X%2X%2X\r\n", BleModuleId[0], BleModuleId[1], BleModuleId[2], BleModuleId[3], BleModuleId[4], BleModuleId[5]);
   }
  
   else 
      length = sprintf(message, "\r[NOTE] Ignored BLE Message {%x, %x, %x, %x}\r\n", hdr->type_hilen, hdr->lolen, hdr->cls, hdr->command);

   ble_log(message, length);
}
////////////////////////////////////////////////////////////////////////////////

#INT_RDA
void ble_receive()
{
   uint8 rx = fgetc(BLE);
   ble_data_buffer[ble_data_index] = rx;
   ble_data_index++;

   if(ble_data_index == 256) 
   {
      ble_data_index = 0;
   }

   struct ble_header * hdr = &ble_data_buffer;
   uint8 length = hdr->lolen + 4;

   if(ble_data_index >= length) 
   {
      ble_data_index = 0;
      ble_process();
   }
}
////////////////////////////////////////
void ConstructBleModule()
{
   // Reset the BLE chip. Reset is active low. So hold "nReset" low.
   // Wait for the boot message
   for(uint16 i=1; ; i++) 
   {
#ifdef DEBUG
     fprintf(STDOUT, "Initialize bluetooth \r\n");
#endif

     output_low(BLE_RESET_PIN);
     delay_ms(500);
     output_high(BLE_RESET_PIN);

     delay_ms(1100);
     int j=0;
     for (; j<100; j++) 
     {
        if (ble_log_process())
           break;
     }
     
     if (j<100)
        break;
   }

   // Make sure that the "hello" ping works
   ble_cmd_system_hello();
   while(!ble_log_process());
   
   // gap_general_discoverable, gap_undirected_connectable
   ble_cmd_gap_set_mode(2, 2);
   while(!ble_log_process());
   
   uint8 buff[7];
   ble_cmd_system_AddressGet();
   while(!ble_log_process());
   
   config_serial_get( (uint8 *)&buff[0] );
   ble_cmd_attributes_write(BLE_HANDLE_SERIAL_NUMBER, 0, 7, &buff);
   while(!ble_log_process());

   ble_cmd_attributes_write(BLE_HANDLE_DEVICE_STAT, 0, sizeof(deviceStat), (uint8 *)&deviceStat);
   while(!ble_log_process());

   fprintf(STDOUT, "Bluetooth Initialized \r\n");
   bluetoothState = PASS;                          //Got this far, blu is working
//   bleNotifyState(MP_STARTUP);

}

////////////////////////////////////////
void DisableBleModule()
{
   // Reset the BLE chip. Reset is active low. So hold "nReset" low.
   // Wait for the boot message
     fprintf(STDOUT, "Shutdown bluetooth \r\n");

     output_low(BLE_RESET_PIN);
     delay_ms(300);
     output_high(BLE_RESET_PIN);
     delay_ms(300);
     output_low(BLE_RESET_PIN);
     delay_ms(300);
}
////////////////////////////////////////
void ble_configure()
{
   // Reset the BLE chip. Reset is active low. So hold "nReset" low.
   // Wait for the boot message
   for(uint16 i=1; ; i++) 
   {
     fprintf(STDOUT, "Resetting BLE %lu times...\r\n\r\n", i);
     output_low(BLE_RESET_PIN);
     delay_ms(500);

     output_high(BLE_RESET_PIN);
     delay_ms(1100);
     
     int j=0;
     for (; j<100; j++) 
     {
        if (ble_log_process())
           break;
     }
     
     if (j<100)
        break;
   }

   // Make sure that the "hello" ping works
   ble_cmd_system_hello();
   while(!ble_log_process());

   // gap_general_discoverable, gap_undirected_connectable
   ble_cmd_gap_set_mode(2, 2);
   while(!ble_log_process());  

   uint8  value = 0;
   uint8  progress = SEND_RESULTS;
   uint8  buff[6];
  
   ble_cmd_system_AddressGet();
   while(!ble_log_process());

   value = reading_index_start();
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_START, 0, sizeof(uint16), &value);
   while(!ble_log_process());

   value = reading_index_stop();
   ble_cmd_attributes_write(BLE_HANDLE_INDEX_STOP, 0, sizeof(uint16), &value);
   while(!ble_log_process());

   config_serial_get(buff);
   ble_cmd_attributes_write(BLE_HANDLE_SERIAL_NUMBER, 0, 6, &buff);
   while(!ble_log_process());
   
   ble_cmd_attributes_write(BLE_HANDLE_READING_IN_PROGRESS, 0, 2, &progress);
   while(!ble_log_process());
   
//   ble_cmd_attributes_write(BLE_HANDLE_SERIAL_NUMBER, 0, 6, &BleModuleId);
//   while(!ble_log_process());

   ble_cmd_attributes_write(BLE_HANDLE_DEVICE_STAT, 0, sizeof(deviceStat), (uint8 *)&deviceStat);
   while(!ble_log_process());
}

////////////////////////////////////////////////////////////////////////////////
void bleNotifyErrorSet(int8 errorBit)
{   
   //ErrorBits |= (1 << errorBit);
   ErrorBits = errorBit;
   if (ErrorBits != LastErrorBits) {
      fprintf(STDOUT, "Error bit %u set\r\n", errorBit);
      LastErrorBits = ErrorBits;
      ble_cmd_attributes_write(BLE_HANDLE_ERROR_CODE, 0, 2, &ErrorBits);
      while(!ble_log_process());
   }
}

////////////////////////////////////////////////////////////////////////////////
void bleNotifyErrorResetErrorBits()
{
   ErrorBits = 0;
   if (ErrorBits != LastErrorBits) {
      fprintf(STDOUT, "Error bits cleared\r\n");
      LastErrorBits = ErrorBits;
      ble_cmd_attributes_write(BLE_HANDLE_ERROR_CODE, 0, 2, &ErrorBits);
      while(!ble_log_process());
   }
}

////////////////////////////////////////////////////////////////////////////////
void bleNotifyErrorReset(int8 errorBit)
{
   ErrorBits &= ~(1 << errorBit);
   if (ErrorBits != LastErrorBits) {
      fprintf(STDOUT, "Error bit%u cleared\r\n", errorBit);
      LastErrorBits = ErrorBits;
      ble_cmd_attributes_write(BLE_HANDLE_ERROR_CODE, 0, 2, &ErrorBits);
      while(!ble_log_process());
   }
}

