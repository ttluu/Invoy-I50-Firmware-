#include <Peripherals.h>

#ifdef BASE_40
struct colorRGB CurrentColor;
#endif

////////////////////////////////////////
void tone(int32 frequency_hz)
{
   pwm_set_frequency(BUZZER, frequency_hz);
}

////////////////////////////////////////
void play() 
{
   pwm_on(BUZZER);
   tone(260);
   delay_ms(200);
   pwm_off(BUZZER);
}

////////////////////////////////////////
void play_error() 
{
   pwm_on(BUZZER);
   tone(500);
   delay_ms(250);
   tone(600);
   delay_ms(250);
   tone(500);
   delay_ms(250);
   pwm_off(BUZZER);
}

void play_error_hw_failure() 
{
   pwm_on(BUZZER);
   tone(500);
   delay_ms(100);
   tone(500);
   delay_ms(100);
   pwm_off(BUZZER);
}
////////////////////////////////////////
// void play()
// {
//    pwm_on(BUZZER);
//    int16 melody[] = { NOTE_C, NOTE_D, NOTE_E };
//
//    int16 i = 0;
//
//    for(i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
//       pwm_on(BUZZER);
//       tone(melody[i]);
//       delay_ms(200);
//       pwm_off(BUZZER);
//
//       pwm_on(BUZZER);
//       tone(2000);
//       delay_ms(200);
//       pwm_off(BUZZER);
//
//       delay_ms(200);
//    }
//    pwm_off(BUZZER);
// }
////////////////////////////////////////

#DEFINE ILLUMINATION_588 PIN_C0
#DEFINE ILLUMINATION_730 PIN_A5
#DEFINE ILLUMINATION_475 PIN_A4

////////////////////////////////////////
void illumination_brightness(BYTE brightness)
{
   int32 command = ((int32)0b0011 << 20) + ((int32)brightness << 8);
   spi_xfer(SPI_1, command, 24);
}

////////////////////////////////////////
void illumination_all_off()
{
   output_low(ILLUMINATION_475);
   output_low(ILLUMINATION_588);
   output_low(ILLUMINATION_730);
}

////////////////////////////////////////
void illumination_use(int16 pin)
{
   illumination_all_off();
   output_high(pin);
}

////////////////////////////////////////
int read_switches()
{
   int data = 0;
   i2c_start();
   i2c_write(0b10000110);
   i2c_write(0x0F);

   i2c_start();
   i2c_write(0b10000111);
   data = i2c_read(FALSE);
   i2c_stop();

   return data;
}
////////////////////////////////////////
int16 photodiode()
{
   set_adc_channel(0);
   return read_adc();
}

////////////////////////////////////////
#ifdef BASE_40
int LastTrayState = 0;
int read_switch(int pin)
{
   int state = CHECK_SWITCH(read_switches(), pin);
   
   if (pin == DETECT_TRAY)
   {
      if (LastTrayState != state) 
      {
         LastTrayState = state;
        
         if (state)
         {
            int v=0;
            ble_cmd_attributes_write(BLE_HANDLE_TRAY_IS_OPEN, 0, 1, &v);
            
            while(!ble_log_process());
               fprintf(STDOUT, "Tray closed\r\n");
         }
      }
   }
   
   return state;
}

////////////////////////////////////////
void tray_init()
{
   if(SOLENOID_POLARITY_SWAP) 
      output_high(SOLENOID_PIN);
}

////////////////////////////////////////
void tray_open()
{

   if(SOLENOID_POLARITY_SWAP) 
      output_low(SOLENOID_PIN);

   else 
      output_high(SOLENOID_PIN);

   delay_ms(1000);

   if(SOLENOID_POLARITY_SWAP) 
      output_high(SOLENOID_PIN);

   else
      output_low(SOLENOID_PIN);
}

////////////////////////////////////////

void tray_open_loop()
{
   while(TRUE) 
   {
      tray_open();
      if(read_switch(DETECT_TRAY) == 0)
      {
         int v=1;
         ble_cmd_attributes_write(BLE_HANDLE_TRAY_IS_OPEN, 0, 1, &v);
         
         while(!ble_log_process());
            break;
      }
   }
}

////////////////////////////////////////
void indicator_led_set(uint16 red, uint16 green, uint16 blue)
{
   // All the values are between 0 and 1000
   pwm_set_duty_percent(LED_RED, red);
   pwm_set_duty_percent(LED_GREEN, green);
   pwm_set_duty_percent(LED_BLUE, blue);
}


////////////////////////////////////////
#define SERVO PIN_E2
void set_servo(int16 time_us, int16 count)
{
   int16 i = 0;
   int16 rest_us = 20000 - time_us;

   for(i = 0; i < count; i++)
   {
      output_high(SERVO);
      delay_us(time_us);
      output_low(SERVO);
      delay_us(rest_us);
   }
}

////////////////////////////////////////
void servo_up() 
{
   set_servo(ServoUpTime, 50);
}

////////////////////////////////////////
void servo_down_with_adjustment(int16 adjustment) 
{
   uint16 i;
   if (g_bottle_uses > 0) 
      i = ServoDownTime + adjustment;

   else 
      i = ServoDownTime + 20 + adjustment;
   
   set_servo(i, 30);
   fprintf(STDOUT, "Servodown: %lu us\r\n", i);
  
   config_bottle_uses_increment();
}

////////////////////////////////////////
void servo_down() 
{
   servo_down_with_adjustment(0);
}

////////////////////////////////////////
void servo_park() 
{
   set_servo(ServoParkTime,50);
}
#endif

