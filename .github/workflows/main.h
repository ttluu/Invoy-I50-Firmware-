//Rev 5   - implement state code machine, add Ble disconnect to shutdown function
//        - add dynamic volume setting via bluetooth and RS232
//    55 - modify volume accumulation to accomodated multiples blow
//        - update state machine code and save results to EEPROM
//    61  - Allow firmware to detect no change in pressure during breath test
//    62  - Reduce the number of handle in bluetooth and size to 20 bytes max in each handle
//    62.1  - Update the firmware and fixes all bugs related to the BLE update, changes Invoy 5.0 to I50
//    62.2  - Add 3 commands to test the storage capacity
//    62.3  - Sent test time immediately after breath test.  No need to wait for cartridge removal.
//    63.0  - Add a 30 seconds delay prior to set the error at wet cartridge and remove cartridge.
//    64.0  - Add a reset command to clear all BLE handles data 
//    65.0  - Modidy check_bad_or_used_cartridge to enhance detection and verifycation, update the breath volume before test init to
//          - avoid rebooting the app, update the breath attempt count.
//    65.1  - Optimize cartridge detection function to increase dectection better differentiate use & new cartridge
//    65.2  - Readjust the 588 led power level to 30% and only turn on 1/2 second every five seconds to notify user to blow
//
//
//



#define FIRMWARE_VERSION_MAJOR 00     // Changes due to requirements or baseline
#define FIRMWARE_VERSION_MINOR 65     // Changes to functionality
#define FIRMWARE_VERSION_REVISION 2   // Non-functional changes

#include <.\Devices\18LF46K22.h>
#device ADC=10
#device ANSI

#FUSES NOWDT                    //No Watch Dog Timer

#use delay(internal=8000000)

// NOTE: The last RS232 will be used by printf as STDOUT
#use rs232(baud=9600,parity=N,xmit=PIN_C6,rcv=PIN_C7,bits=8,ERRORS,stream=BLE)
#use rs232(baud=115200,parity=N,xmit=PIN_D6,rcv=PIN_D7,bits=8,ERRORS,RECEIVE_BUFFER=50,stream=STDOUT)

#use i2c(Master,Slow,sda=PIN_C4,scl=PIN_C3)          //Use this set-up - BLE works, BME does not 
//#use i2c(Master,Slow,sda=PIN_D5,scl=PIN_A7)        //Use this set-up - BME works, BLE does not

#use spi(MASTER, DO=PIN_D4, CLK=PIN_D0, ENABLE=PIN_D3, MODE=0, BITS=24, STREAM=SPI_1)
//#use spi(MASTER, DO=PIN_C4, DI=PIN_D4, CLK=PIN_C3, ENABLE=PIN_E2, MODE=0, BITS=16, STREAM=SPI_BME)

#define PWM_BUZZER PIN_D1
#use pwm(OUTPUT=PWM_BUZZER, FREQUENCY=260Hz, DUTY=50, PWM_OFF, STREAM=BUZZER)

#use timer(TIMER=1, TICK=1ms, BITS=32, ISR)
