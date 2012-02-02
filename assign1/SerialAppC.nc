#include "serial_msg.h"

configuration SerialAppC
{
}
implementation {
  components MainC, SerialC as App, LedsC;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new TimerMilliC() as Timer2;
  components SerialActiveMessageC as Serial;
  
  App.Boot -> MainC.Boot;
  App.SerialSend -> Serial.AMSend[AM_SERIAL_MSG];
  App.SerialReceive -> Serial.Receive[AM_SERIAL_MSG];
  App.AMControl -> Serial;
  App.Packet -> Serial;
  App.MilliTimer1 -> Timer0;
  App.MilliTimer2 -> Timer1;
  App.MilliTimer3 -> Timer2;
  App.Leds -> LedsC;
}
