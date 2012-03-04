/*
 * Wong Kangwei
 * U096904B
 * CS4222 Assignment 2
 */

#include "SerialToRadioMsg.h"
#include "serial_msg.h"

configuration SerialToRadioAppC {
}

implementation {
  components MainC, SerialToRadioC as App, LedsC;
  components new TimerMilliC() as Timer;
  components SerialActiveMessageC as Serial;
  components ActiveMessageC as Radio;
  components new AMSenderC(AM_SERIALTORADIOMSG);
  components new AMReceiverC(AM_SERIALTORADIOMSG);
  components new HamamatsuS10871TsrC() as TsrC; // throws warning about accessing TimerA for ADC12
  components new SensirionSht11C() as TempHumidC;
  
  App.WakeupTimer1 -> Timer;
  App.Boot -> MainC.Boot;
  App.SerialSend -> Serial.AMSend[AM_SERIALTORADIOMSG];
  App.SerialReceive -> Serial.Receive[AM_SERIALTORADIOMSG];
  App.RadioSend -> AMSenderC;
  App.RadioReceive -> AMReceiverC;
  App.AMControl -> Radio;
  App.AMControl2 -> Serial;
  App.Packet -> AMSenderC;
  App.AMPacket -> AMSenderC;
  App.Leds -> LedsC;
  App.ReadLIGHT -> TsrC;
  App.ReadTEMP -> TempHumidC.Temperature;
  App.ReadHUMID -> TempHumidC.Humidity;
}
