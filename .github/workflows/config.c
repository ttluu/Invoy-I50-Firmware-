#include <ble.h>

#define CONFIG_LED730_BRIGHTNESS_ADDRESS 0
#define CONFIG_LED588_BRIGHTNESS_ADDRESS 1
#define CONFIG_LED475_BRIGHTNESS_ADDRESS 2

//32 bits float
#define CONFIG_SLOPE 10
#define CONFIG_INTERCEPT 14
#define CONFIG_LED_BRIGHTNESS_CALIBRATED 19     // 1 bytes

// The serial number is 6 sequential bytes. For example {1, 2, 3, 4, 5, 6} for a serial number of "123456"
#define CONFIG_SERIAL 20 

//16 bit unsigned int
#define CONFIG_BREATH_VOLUME 26
#define CONFIG_MOUTHPIECE_USES 28

// Index are 16 bit
#define CONFIG_READING_INDEX_START 30
#define CONFIG_READING_INDEX_STOP 32

// 8 Bit
#define CONFIG_DYNAMIC_WHITEBALANCE_ENABLED 34

#define CONFIG_READING_BASE   CONFIG_DYNAMIC_WHITEBALANCE_ENABLED + 6
#define CONFIG_READING_COUNT  10
#define CONFIG_READING_END    CONFIG_READING_BASE + ( sizeof(struct reading ) * CONFIG_READING_COUNT ) // Where 18 is size of reading struct


#ifdef BASE_40
#define CONFIG_READING_END 280 // 40 + (12 * 20)
#endif

//////////////////////////////////////////////////

void config_serial_set(uint8 * serial) 
{
  for(uint8 i = 0; i < 6; i++) {
    write_ext_eeprom(CONFIG_SERIAL + i, *(serial + i));
  }
}

void config_serial_get(uint8 * serial) 
{
  for(uint8 i = 0; i < 6; i++) 
  {
    serial[i] = read_ext_eeprom(CONFIG_SERIAL + i);
  }
}

//////////////////////////////////////////////////

void ConfigWriteFloat(long int address, float32 data)
{
   int *p = &data;
   for (int i=0; i<4; i++, address++)
      write_ext_eeprom(address, p[i]);
}

//////////////////////////////////////////////////
float32 ConfigReadFloat(long int address)
{
   float32 data;
   int *p = &data;

   for (int i=0; i<4; i++, address++)
      p[i] = read_ext_eeprom(address);

   return data;
}

//////////////////////////////////////////////////
int ConfigReadByte(long int address) 
{
   return read_ext_eeprom(address);
}

//////////////////////////////////////////////////
void ConfigWriteByte(long int address, int data) 
{
   write_ext_eeprom(address, data);
}

//////////////////////////////////////////////////
uint16 ConfigReadInt16(long int address) 
{
   uint16 data;
   int *p = &data;
   
   p[0] = read_ext_eeprom(address++);
   p[1] = read_ext_eeprom(address);
   return data;
}

//////////////////////////////////////////////////
void ConfigWriteIn16(long int address, uint16 data) 
{
   int *p = &data;
   write_ext_eeprom(address, p[0]);
   write_ext_eeprom(address, p[1]);
}

//////////////////////////////////////////////////
#ifdef BASE_40

#define CONFIG_SERVO_UP_TIME 280 // 2 bytes
#define CONFIG_SERVO_PARK_TIME 282 // 2 bytes
#define CONFIG_SERVO_DOWN_TIME 284 // 2 bytes

uint16 ServoUpTime;
uint16 ServoParkTime;
uint16 ServoDownTime;
uint8 g_bottle_uses = 0;
uint8 lastbottleUses = 0xFF;

uint8 config_bottle_uses_get() {
   g_bottle_uses = read_ext_eeprom(CONFIG_BOTTLE_USES);
   fprintf(STDOUT, "Tank-uses counter=%u\r\n", g_bottle_uses);
   if (lastbottleUses != g_bottle_uses) {
      lastbottleUses = g_bottle_uses;
      ble_cmd_attributes_write(BLE_HANDLE_TANK_COUNTER, 0, 1, &lastbottleUses);
      while(!ble_log_process());
   }
   return g_bottle_uses;
}

void config_bottle_uses_set(uint8 value) {
   g_bottle_uses = value;
   write_ext_eeprom(CONFIG_BOTTLE_USES, value);
   config_bottle_uses_get();
}

void config_bottle_uses_increment() {
  uint8 i = config_bottle_uses_get();

  i = i + 1;
  config_bottle_uses_set(i);
}

void config_bottle_uses_decrement() {
  uint8 i = config_bottle_uses_get();

  i = i - 1;
  if (i>127)
    i = 0;
    
  config_bottle_uses_set(i);
}

//////////////////////////////////////////////////

uint8 g_mouthpiece_uses = 0;
uint8 lastMouthpieceUses = 0xFE;

uint8 config_mouthpiece_get()
{
  g_mouthpiece_uses = read_ext_eeprom(CONFIG_MOUTHPIECE_USES);
  fprintf(STDOUT, "Mouthpiece-uses counter=%u\r\n", g_mouthpiece_uses);
   if (lastMouthpieceUses != g_mouthpiece_uses) 
   {
      lastMouthpieceUses = g_mouthpiece_uses;
      ble_cmd_attributes_write(BLE_HANDLE_MOUTHPIECE_COUNTER, 0, 1, &lastMouthpieceUses);
     while(!ble_log_process());
   }
  return g_mouthpiece_uses;
}

void config_mouthpiece_set(uint8 value) {
   g_mouthpiece_uses = value;
   write_ext_eeprom(CONFIG_MOUTHPIECE_USES, value);
   config_mouthpiece_get();
}

void config_mouthpiece_increment() {
  uint8 i = config_mouthpiece_get();

  i = i + 1;
  config_mouthpiece_set(i);
}

//////////////////////////////////////////////////
void InitializeServo() {
   ServoUpTime = EepromReadInt16(CONFIG_SERVO_UP_TIME);
   fprintf(STDOUT, "ServoUpTime= %lu us\r\n", ServoUpTime);
   if (ServoUpTime == 0) {
      ServoUpTime = 1200;
      EepromWriteInt16(CONFIG_SERVO_UP_TIME, ServoUpTime);
      fprintf(STDOUT, "ServoUpTime initialized to %lu us\r\n", ServoUpTime);
   }
   
   ServoParkTime = EepromReadInt16(CONFIG_SERVO_PARK_TIME);
   fprintf(STDOUT, "ServoParkTime= %lu us\r", ServoParkTime);
   if (ServoParkTime < 100) {
      ServoParkTime = 900;
      EepromWriteInt16(CONFIG_SERVO_PARK_TIME, ServoParkTime);
      fprintf(STDOUT, "ServoParkTime initialized to %lu us\r\n", ServoParkTime);
   }
   
   ServoDownTime = EepromReadInt16(CONFIG_SERVO_DOWN_TIME);
   fprintf(STDOUT, "ServoDownTime= %lu us\r", ServoDownTime);
   if (ServoDownTime < 100) {
      ServoDownTime = 800;
      EepromWriteInt16(CONFIG_SERVO_DOWN_TIME, ServoDownTime);
      fprintf(STDOUT, "ServoDownTime initialized to %lu us\r\n", ServoDownTime);
   }   
}

uint16 ConfigGetServoUpTime() {
   uint16 x = EepromReadInt16(CONFIG_SERVO_UP_TIME);
   fprintf(STDOUT, "ServoUpTime= %lu us\r\n", x);
   return x;
}

uint16 ConfigGetServoParkTime() {
   uint16 x = EepromReadInt16(CONFIG_SERVO_PARK_TIME);
   fprintf(STDOUT, "ServoParkTime= %lu us\r\n", x);
   return x;
}

uint16 ConfigGetServoDownTime() {
   uint16 x = EepromReadInt16(CONFIG_SERVO_DOWN_TIME);
   fprintf(STDOUT, "ServoDownTime= %lu us\r\n", x);
   return x;
}

void ConfigSetServoUpTime(uint16 x) {
   EepromWriteInt16(CONFIG_SERVO_UP_TIME, x);
   ServoUpTime = x;
   fprintf(STDOUT, "ServoUpTime changed to %lu us\r\n", x);
}

void ConfigSetServoParkTime(uint16 x) {
   EepromWriteInt16(CONFIG_SERVO_PARK_TIME, x);
   ServoParkTime = x;
   fprintf(STDOUT, "ServoParkTime changed to %lu us\r\n", x);
}
void ConfigSetServoDownTime(uint16 x) {
   EepromWriteInt16(CONFIG_SERVO_DOWN_TIME, x);
   ServoDownTime = x;
   fprintf(STDOUT, "ServoDownTime changed to %lu us\r\n", x);
}
#endif
