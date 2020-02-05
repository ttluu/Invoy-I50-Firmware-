#ifndef __BLE__
#define __BLE__


typedef struct bd_addr_t
{
  uint8 addr[6];
} bd_addr;

typedef bd_addr hwaddr;

typedef struct
{
  uint8 len;
  uint8 data[];
} uint8array;

struct ble_header
{
    uint8  type_hilen;
    uint8  lolen;
    uint8  cls;
    uint8  command;
};

struct ble_msg_system_boot_evt_t
{
   uint16   major;
   uint16   minor;
   uint16   patch;
   uint16   build;
   uint16   ll_version;
   uint8    protocol_version;
   uint8    hw;
};

struct ble_msg_attributes_value_evt_t
{
   uint8       connection;
   uint8       reason;
   uint16      handle;
   uint16      offset;
   uint8array  value;
};

#define BLE_RESET_PIN PIN_B2

#define BLE_HANDLE_SERIAL_NUMBER 17
#define BLE_HANDLE_DEVICE_STAT 21
#define BLE_HANDLE_TEST_STAGE 25
#define BLE_HANDLE_TEST_STAT 29
#define BLE_HANDLE_TEST_REPORT 33
#define BLE_HANDLE_INDEX_START 37
#define BLE_HANDLE_INDEX_STOP 41
#define BLE_HANDLE_INDEX_CURRENT 45
#define BLE_HANDLE_READING 48
#define BLE_HANDLE_READING_IN_PROGRESS 52
#define BLE_HANDLE_OPCODE 56
#define BLE_HANDLE_STATE_CODE 59
#define BLE_HANDLE_ERROR_CODE 63

#define BLE_HANDLE_RX 68
#define BLE_HANDLE_TX 71

enum BleOpcodes 
{
   OPCODE_BLE_START_KETONE_TEST = 0,
   OPCODE_BLE_START_AMMONIA_TEST,
   OPCODE_RESET_USE_COUNTER,
   OPCODE_RESET_READINGS,
   OPCODE_RESET_STATE_MACHINE = 0XFA,
   OPCODE_BLE_START_FIRMWARE_UPDATE = 0xFF,

};

void ble_cmd_system_hello();
void ble_cmd_system_AddressGet();
void ble_cmd_gap_set_mode(uint8 discover, uint8 connect);
void ble_cmd_attributes_write(uint16 handle, uint8 offset, uint8 length, uint8 *value);
void ble_wait(int32 timeout);
void ble_wait_for_message(uint8 start);
void ble_process();
void ConstructBleModule();
void DisableBleModule();
void bleNotifyErrorSet(int8 errorBit);
void bleNotifyErrorReset(int8 errorBit);
uint8 ble_log_process();
void ble_configure();


#endif
