#include "serial_msg.h"

configuration SerialAppC
{
}
implementation {
  components MainC, SerialC as App, LedsC;
  components new TimerMilliC();
  components SerialActiveMessageC as Serial;
  
  App.Boot -> MainC.Boot;
  App.SerialSend -> Serial.AMSend[AM_SERIAL_MSG];
  App.SerialReceive -> Serial.Receive[AM_SERIAL_MSG];
  App.AMControl -> Serial;
  App.Packet -> Serial;
  App.MilliTimer -> TimerMilliC;
  App.Leds -> LedsC;
}
