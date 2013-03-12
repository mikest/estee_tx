// PPM encoding for Arduino.
//Copyright (c) 2013 Mike Estee.
//All rights reserved.
//
//Redistribution and use in source and binary forms are permitted
//provided that the above copyright notice and this paragraph are
//duplicated in all such forms and that any documentation,
//advertising materials, and other materials related to such
//distribution and use acknowledge that the software was developed
//by the <organization>.  The name of the
//<organization> may not be used to endorse or promote products derived
//from this software without specific prior written permission.
//THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
//IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
//WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

// PPM pinout for a JR compatible module is, as viewed from back:
//  5: PPM IN [0..5V]
//  4: +6V
//  3: VBAT, 6-12V
//  2: GND
//  1: ANTENNA OUT

// This code was tested with an OrangeRX JR compatible 2.4GHz DSMX transmitter module.
// http://orangerx.com/


#define CHAN_COUNT 8  // above 8 channels you run the risk of hitting the end of the 22ms frame.
float ppmChan[CHAN_COUNT];  // Channel values, range [0..1]; mid point: 0.5

// PPM sum phase timing, and hardcoded trims. These where determined by watching the receiver
// timings in an oscilliscope. PPM start pulse is 300uS, which makes the minimum trim point 700uS
// 300uS + 700uS = 1000uS minimum pulse width point. Actual max pulse appears to be 2050uS.
// This is probably a function of a 22000uS (22ms) pulse train timing for DSMX transmitter.
// Additional trim should be applied on top of this at the ppmChan[] registers.
int ppmStart = 300;
int ppmMin = 700 + 5;  // 5uS trim adjust for low end.
int ppmRange = 1000 + 45;  // 50uS trim adjust for range

// Initialize Timer1 to use for our PPM signal generation.
// We use an interrupt time to toggle the output HIGH/LOW so that the control loop is freed up
// to perform other tasks without impacting signal timing integrity.
void ppmTimerInit()
{
  // clear the channel registers
  for( int i=0; i<CHAN_COUNT; i++ )
    ppmChan[i] = 0;
  
  // clear config registers
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;  // compare register
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();
    
  // pin 13 is PPM sum out
  pinMode(13, OUTPUT);
}

// control variables used for PPM signal generation
int elapsedTime = 0;
unsigned char chan = 0;
unsigned char pulsePhase = 0;

// PPM timer interrupt routine
ISR(TIMER1_COMPA_vect)
{
  noInterrupts();
  
  int tdelta = 0;  // time delta for till the next phase
  switch( pulsePhase )
  {
   // rising edge of the initial 300uS pulse
   case 0:
     digitalWrite(13, HIGH);
     tdelta = ppmStart;
     elapsedTime += tdelta;
     pulsePhase = 1;
     break;
   
   // fall edge of the 300uS pulse, delay till next rising edge
   case 1:
     digitalWrite(13, LOW);
     tdelta = floor(ppmChan[chan] * ppmRange) + ppmMin;
     elapsedTime += tdelta;
     chan ++;
     if( chan < CHAN_COUNT )
       pulsePhase = 0;
     else
       pulsePhase = 2;  // last channel
     break;
   
   // synchronize by holding low for the remainder of the 22ms frame
   case 2:
     tdelta = 22000 - elapsedTime;  // 22ms per frame
     chan = 0;
     pulsePhase = 0;
     elapsedTime = 0;
     break;
   
   // shouldn't get here
   default:
     pulsePhase = 0;
  }
  
  // set next interrupt to fire in tdelta microseconds
  OCR1A = (F_CPU / 8) / floor(1.0/(float(tdelta)/1000000.0));
  interrupts();
}

// init our program
void setup()
{
  analogReference(DEFAULT);  // 0..5V
  ppmTimerInit();
}

// run forever
void loop()
{
  // read the analog input sticks
  float LY = (analogRead(A0) / 1023.0);
  float LX = (analogRead(A1) / 1023.0);
  float RY = (analogRead(A2) / 1023.0);
  float RX = (analogRead(A3) / 1023.0);
  
  // update the registers. this is a KK2 controller board mapping.
  ppmChan[0] = 1.0 - LY; // Throttle
  ppmChan[1] = RX;       // Roll
  ppmChan[2] = 1.0 - RY; // Pitch
  ppmChan[3] = LX;       // Yaw
  
  ppmChan[4] = 0.0;
  ppmChan[5] = 0.33;
  ppmChan[6] = 0.66;
  ppmChan[7] = 1.00;
}
