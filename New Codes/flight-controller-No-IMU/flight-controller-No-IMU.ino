#include <Wire.h>
#include <EEPROM.h>

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;

void setup() {
  // put your setup code here, to run once:
  DDRD |= B11110000;                                                        //Set 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Set 12 and 13 as output.

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
}

void loop() {
  // put your main code here, to run repeatedly:

}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1
  if(PINB & B00000001){                                                     
    if(last_channel_1 == 0){                                                
      last_channel_1 = 1;                                                   
      timer_1 = current_time;                                               
    }
  }
  else if(last_channel_1 == 1){                                             
    last_channel_1 = 0;                                                     
    receiver_input[1] = current_time - timer_1;                             
  }
  //Channel 2
  if(PINB & B00000010 ){                                                    
    if(last_channel_2 == 0){                                                
      last_channel_2 = 1;                                                   
      timer_2 = current_time;                                               
    }
  }
  else if(last_channel_2 == 1){                                             
    last_channel_2 = 0;                                                     
    receiver_input[2] = current_time - timer_2;                             
  }
  //Channel 3
  if(PINB & B00000100 ){                                                    
    if(last_channel_3 == 0){                                                
      last_channel_3 = 1;                                                   
      timer_3 = current_time;                                               
    }
  }
  else if(last_channel_3 == 1){                                             
    last_channel_3 = 0;                                                     
    receiver_input[3] = current_time - timer_3;                             

  }
  //Channel 4
  if(PINB & B00001000 ){                                                    
    if(last_channel_4 == 0){                                                
      last_channel_4 = 1;                                                   
      timer_4 = current_time;                                               
    }
  }
  else if(last_channel_4 == 1){                                             
    last_channel_4 = 0;                                                     
    receiver_input[4] = current_time - timer_4;                             
  }
}
