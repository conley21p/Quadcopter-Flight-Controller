//Declaring Varibles 
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;      //For checking last state of channel
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;


unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
int receiver_input[5];

/*
 * PCICR is pin change interrupt control register
 * PCIE0 is pin change interrupt enable bit 0
 * PCMSK0 pin change mask register zero
 */
void setup(){
PCICR |= (1 << PCIE0); //Set PCIEO to enable PCMSKO     allows interupt
PCMSK0 |= (1 << PCINT4); //PCINT4 is pin 10   Set PCINT4 to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT5); //PCINT4 is pin 11   Set PCINT5 to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT6); //PCINT4 is pin 12   Set PCINT6 to trigger an interrupt on state change
PCMSK0 |= (1 << PCINT7); //PCINT4 is pin 13   Set PCINT7 to trigger an interrupt on state change
Serial.begin(2000000);
}
//This routine is called everytime input 10 changes state
ISR(PCINT0_vect){
  current_time = micros();
  //Channel one
  if(PINB & B00010000){                                   //is input 10 HIGH ?
    if(last_channel_1==0){                                //Input 10 changed from 0 to 1
      last_channel_1 = 1;                                 //reme,ber current input state
      timer_1 = current_time;                             //set timer_1 to current_time
    }
  }else if(last_channel_1 ==1){                           //Input 10 is not high 
    last_channel_1 = 0;                                   //Remeberc current input state
    receiver_input[1] = current_time-timer_1;                 //channel 1 is current time - timer_1;
  }
    //Channel 2=========================================
  if(PINB & B00100000 ){                                                    
    if(last_channel_2 == 0){                                                
      last_channel_2 = 1;                                                   
      timer_2 = current_time;                                               
    }
  }
  else if(last_channel_2 == 1){                                             
    last_channel_2 = 0;                                                     
    receiver_input[2] = current_time - timer_2;                             
  }
  //Channel 3=========================================
  if(PINB & B01000000 ){                                                    
    if(last_channel_3 == 0){                                                
      last_channel_3 = 1;                                                   
      timer_3 = current_time;                                               
    }
  }
  else if(last_channel_3 == 1){                                             
    last_channel_3 = 0;                                                     
    receiver_input[3] = current_time - timer_3;                             

  }
  //Channel 4=========================================
  if(PINB & B10000000 ){                                                    
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


void loop() {
  delay(250);
  print_signals();
}

void print_signals(){
  //Roll Output
  Serial.print("ROLL:");
  if(receiver_input[1]-1480<0)Serial.print("<<<");
  else if(receiver_input[1]-1520>0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input[1]);
  //Pitch
  Serial.print(" Pitch:");
  if(receiver_input[2]-1480<0)Serial.print("<<<");
  else if(receiver_input[2]-1520>0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input[2]);
  //Thrust
  Serial.print(" Thrust:");
  if(receiver_input[3]-1480<0)Serial.print("<<<");
  else if(receiver_input[3]-1520>0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input[3]);
  //Yaw
  Serial.print(" Yaw:");
  if(receiver_input[4]-1480<0)Serial.print("<<<");
  else if(receiver_input[4]-1520>0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input[4]);
}
