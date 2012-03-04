/*
 * Wong Kangwei
 * U096904B
 * CS4222 Assignment 2
 */

#include "serial_msg.h"

configuration RadioToRadioAppC {
}



implementation {
  components MainC, RadioToRadioC as App, LedsC;
  components new TimerMilliC() as Timer;
  components SerialActiveMessageC as Serial;
  components ActiveMessageC as Radio;
  components new AMSenderC(AM_SERIAL_MSG);
  components new AMReceiverC(AM_SERIAL_MSG);
  
  components HplMsp430GeneralIOC;
  App.SBControl -> HplMsp430GeneralIOC.Port23;
  App.SBSwitch  -> HplMsp430GeneralIOC.Port26;
  
  // --------- ADC related ---------
  components new SBT80_ADCconfigC() as VL;
  components new SBT80_ADCconfigC() as TEMP;

  
  //App.WakeupTimer1 -> Timer;
  App.Boot -> MainC.Boot;
  App.SerialSend -> Serial.AMSend[AM_SERIAL_MSG];
  //  App.SerialReceive -> Serial.Receive[AM_SERIALTORADIOMSG];
  App.RadioSend -> AMSenderC;
  App.RadioReceive -> AMReceiverC;
  App.AMControl -> Radio;
  App.AMControl2 -> Serial;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.Leds -> LedsC;
  App.ReadLIGHT -> VL.ReadADC0;
  App.ReadTEMP -> TEMP.ReadADC3;

}
