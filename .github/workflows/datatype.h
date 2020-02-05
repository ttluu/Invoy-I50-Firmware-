#ifndef DATATYPE_H_
#define DATATYPE_H_


#define PASS         1
#define FAIL         0
#define ON           1
#define OFF          0
#define YES          1
#define NO           0

#define ONE_HUNDRED  100


#define LED_BRIGHTNESS_DEFAULT_VALUE   50
#define LED_BRIGHTNESS_MINIMUM_SETTING 40
#define LED_BRIGHTNESS_MAXIMUM_SETTING 100

#define BAD_CARTRIDGE   1
#define GOOD_CARTRIDGE  0

#define NEW_CARTRIDGE_MIN_730   540           //Led 730
#define NEW_CARTRIDGE_MIN_588   558           //Led 588
#define NEW_CARTRIDGE_MIN_475   630           //Led 730

#define NEW_CARTRIDGE_MAX_730   640
#define NEW_CARTRIDGE_MAX_588   651
#define NEW_CARTRIDGE_MAX_475   750


#define NO_CARTRIDGE    75
#define JUST_WET_CARTRIDGE_MAX_588    410
#define JUST_WET_CARTRIDGE_MIN_588    170




#define UNIT_CALIBRATED 0xAA
#define SLOPE        .020
#define INTERCEPT    .090

//Test mode
#define CALIBRATION_TEST         0X1
#define KETONE_TEST              0X2
#define AMMONIA_TEST             0X3

#define MAXINUM_BREATH_VOLUME    2000
#define STANDARD_BREATH_VOLUME   600
#define MININUM_BREATH_VOLUME    300


#define INACTIVITY_TIME_LIMIT   30               //5 minutes time limit for inactivity
#define MAXIMUM_NUMBER_OF_RECORDS 24

#define FIVE_MINUTES    300000
#define FOUR_MINUTES    240000
#define THREE_MINUTES   180000
#define TWO_MINUTES     120000
#define ONE_MINUTE      60000
#define THIRTY_SECONDS  30000
#define FIFTEEN_SECONDS 15000
#define TEN_SECONDS     10000
#define FIVE_SECONDS     5000
#define FOUR_SECONDS     4000
#define THREE_SECONDS    3000
#define TWO_SECONDS      2000
#define ONE_SECOND       1000

#define BLE_MODULE_ID_SIZE 6

const char bad_resp[] = "Bad ";
const char cartridge_resp[] = "Cartridge ";
const char bright730_resp[] = "Bright730 ";
const char bright588_resp[] = "Bright588 ";
const char bright475_resp[] = "Bright475 ";
const char brightness_resp[] = "Brightness ";
const char calibrated_resp[] = "Calibrated ";
const char cartridge_wetting_check_resp[] = "Cartridge Wetting Check ";
const char colon_resp[] = ": ";
const char darkness_resp[] = "Darkness ";
const char dash_resp[] = "- ";
const char detected_resp[] = "Detected ";
const char fail_resp[] = "Fail ";
const char hardware_check_resp[] = "Hardware Checked ";
const char insert_new_cartridge_resp[] = "Insert new cartridge ";
const char led_resp[] = "LED ";
const char max_power_setting_exceeded_resp[] = "Exceeded maximum power limit allowed ";
const char new_line_resp[] = "\r\n";
const char new_resp[] = "New ";

const char not_resp[] = "Not ";
const char pass_resp[] = "Pass ";
const char please_wait_resp[] = "Please wait.....";
const char set_resp[] = "Set ";
const char test_started_resp[] = "test started ";
const char time_limit_exceeded[] = "Time limit exceeded";
const char two_new_lines_resp[] = "\r\n\r\n";
const char used_resp[] = "Used ";
const char wet_resp[] = "Wet ";




enum BreathTestState
{
    BREATH_TEST_NOT_RUN,
    BREATH_TEST_STARTED,
    BREATH_TEST_EXITING,
    BREATH_TEST_COMPLETED,
    BREATH_TEST_ENDED
};

enum CatridgeState
{
    CARTRIDGE_STATE_NEW,
    CARTRIDGE_STATE_USED,
    CARTRIDGE_STATE_JUST_WET,
    CARTRIDGE_STATE_UNKNOWN    
};

enum iosRequests
{
   IOS_REQUEST_START_KETONE_TEST= 1,
   IOS_REQUEST_START_AMMONIA_TEST,
   IOS_REQUEST_RESET_USE_COUNTER,
   IOS_REQUEST_RESET_READINGS,
   IOS_REQUEST_RESET_STATE_MACHINE = 0XFA,
   IOS_REQUEST_START_FIRMWARE_UPDATE = 0xFF,
};

enum I50StepState
{
   HARDWARE_CHECK = 0,
   TEST_STARTED = 1,
   INSERT_CARTRIDGE = 10,
   BLOW = 20,
   WET = 30,
   ANALYZE_BREATH_SAMPLE = 35,
   REMOVE_CARTRIDGE = 40,
   TEST_COMPLETED = 50,
   SEND_RESULTS = 60,
   ERROR_TERMINATION = 70,

   FULLTEST_UNKNOWN = 0xFFFF
};

enum I50ErrorCode
{
   NO_ERROR = 0,
   HARDWARE_FAIL = 1,
   BAD_CATRIDGE = 2,
   BAD_BLOW_VOLUME = 4,
   BAD_BLOW_PRESSURE = 8,
   BAD_SCORE = 10,
   NO_WETTING = 20,
   CARTRIDGE_NOT_REMOVED = 40,
   CARTRIDGE_REMOVED_DURING_ANALYSIS = 80,
   TIME_OUT = 100,
};

enum opCodeCmd
{
   START_TEST = 0, 
   INIT_FIRMWARE_UPDATE = 1
};

struct testResultPara
{
   int16    testMode;
   uint32   totalTestTime;
   int32    score;
   uint8    status;
   uint8    stepID;
};

struct timePara
{
   uint8   hour;
   uint8   minute;
   uint8   second;
   uint8   day;
   uint8   month;
   uint8   year;
};

struct commandPara
{
   uint8             opCode;
   uint16            volume;
   struct timePara   time;
};

struct statusPara
{
   uint32 cartridgeInsertionTime;
   uint32 blowTime;
   uint32 wettingTime;
   uint32 removeCartridgeTime;
};

struct deviceInfoPara
{
   uint16 fwMajor;
   uint16 fwMinor;
   uint16 fwRevision;
   uint16 batteryLevel;   
   uint16 useCount;
   uint8  bleModuleId[6];
};

struct testInfoPara
{
   int32  temperature;
   uint32 pressure;
   uint16 volume;
   uint16 humidity;
   uint16 numOfBlowAttempts;
};





#endif
