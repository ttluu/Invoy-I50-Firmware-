#ifndef __PERIPHERALS__
#define __PERIPHERALS__

#define BUTTON_PRIMARY 0
#define BUTTON_RESET 1
#define DETECT_CARTRIDGE 2
#define DETECT_BOTTLE_DOOR_ALT 3
#define DETECT_TRAY 4
#define DETECT_BOTTLE_DOOR 7
#define DETECT_OVER_TEMPERATURE 6
#define DETECT_BOTTLE 5

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define CHECK_SWITCH(var, pos) !CHECK_BIT(var, pos)

#define SOLENOID_PIN PIN_D5
#define SOLENOID_POLARITY_SWAP 0

#define NOTE_C 261
#define NOTE_D 293
#define NOTE_E 329
#define NOTE_F 349
#define NOTE_G 391
#define NOTE_A 440
#define NOTE_B 493

struct colorRGB {
   uint16 Red, Green, Blue;
};



int read_switches();
int read_switch(int pin);
int16 photodiode();

#ifdef BASE_40
void tray_init();
void tray_open();
void tray_open_loop();
#endif

void tone(int32 frequency_hz);
void play();
void play_error();
void play_error_hw_failure() ;

#ifdef BASE_40
void SetButtonColor(uint16 Red, uint16 Green, uint16 Blue);
#endif


#endif
