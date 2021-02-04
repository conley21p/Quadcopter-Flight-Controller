/* By: Conley Price
 *    Description: This is for a arduino mega 2560 board, 
 *      this software is modelled after "http://www.brokking.net/ymfc-al_main.html".
 *      
 *  Video: video will be uploaded to youtube soon
 *  Hardware: MicroCOntroller/FlightController: arduino mega 2560 board
 *            MPU: MPU 6050
 *            Battery: Tattu 3s 11.1V
 *            Frame: F450 frame
 *            Motors: QWinOut A2212 1000KV Brushless Outrunner Motor 13T
 *            ESC: QWinOut 2-4S 30A RC Brushless ESC Simonk
 */

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;  //store transmitter inputs
byte highByte, lowByte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4; ////store transmitter inputs
int esc_1, esc_2, esc_3, esc_4;
int throttle,pitch, roll, yaw, battery_voltage;
int cal_int, start;
int receiver_input[5];


long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
;

int acc1[11], acc2[11], acc3[11];
int count = 0;




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(57600);
  
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRH |= B01111000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.

  PCICR |= (1 << PCIE0); //Set PCIEO to enable PCMSKO     allows interupt
  PCMSK0 |= (1 << PCINT4); //PCINT4 is pin 10   Set PCINT4 to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT5); //PCINT4 is pin 11   Set PCINT5 to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT6); //PCINT4 is pin 12   Set PCINT6 to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT7); //PCINT4 is pin 13   Set PCINT7 to trigger an interrupt on state changeSerial.println("almost");

Serial.println(receiver_input_channel_3);
  start = 0;                                                                //Set start back to 0.
  
  Serial.println("almost");
  
battery_voltage = 1150;
  loop_timer = micros();                                                    //Set the timer for the next loop.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  Serial.print("1: ");
  Serial.print(esc_1);
  Serial.print("    2: ");
  Serial.print(esc_2);
  Serial.print("    3: ");
  Serial.print(esc_3);
  Serial.print("    4: ");
  Serial.print(esc_4);
  Serial.print("    pitch: ");
  Serial.print(pitch);
  Serial.print("    roll: ");
  Serial.print(roll);
  Serial.print("  ch4: ");
  Serial.print(receiver_input_channel_4-1500);
  Serial.print("  ch4: ");
  Serial.print((receiver_input_channel_4-1500)*2);
  Serial.print("    yaw ");
  Serial.println(yaw);
  
  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
  }
  //Stopping the motors: throttle low and yaw right.
  if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;


battery_voltage = 1150;
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  pitch = ((receiver_input_channel_2-1500)*2)/5;
  roll = ((receiver_input_channel_1-1500)*2)/5;
  yaw = ((receiver_input_channel_4-1500)*1)/5;


  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pitch - roll - yaw;                                  //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle - pitch + roll + yaw;                                  //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pitch + roll - yaw;                                  //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle + pitch - roll + yaw;                                  //Calculate the pulse for esc 4 (front-left - CW)

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }


  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  if(micros() - loop_timer > 4050){}                   //Turn on the LED if the loop time exceeds 4050us.
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTH |= B11111000;                                                       //Set digital outputs 6,7,8 and 9 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  

  while(PORTH >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer){
      PORTH &= B11110111;
      PORTH &= B01111111;}                //Set digital output 6 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTH &= B11101111;                //Set digital output 7 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTH &= B11011111;                //Set digital output 8 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTH &= B10111111;                //Set digital output 9 to low if the time is expired.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00010000){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00100000 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B01000000 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

  }
  //Channel 4=========================================
  if(PINB & B10000000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }
}



  
