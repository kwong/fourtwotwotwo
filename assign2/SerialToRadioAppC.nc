/*
 * Wong Kangwei
 * U096904B
 * CS4222 Assignment 2
 */

#include "SerialToRadioMsg.h"

configuration SerialToRadioC {}

implementation {
  components MainC, SerialC as App, LedsC;
  components new TimerMilliC() as Timer;
  components SerialActiveMessageC as Serial;
  components new HamamatsuS10871TsrC() as TsrC;
  components new SensirionSht11C() as TempHumidC;
  

  App.Boot -> MainC.Boot;
  App.SerialSend -> Serial.AMSend[AM_SERIALTORADIOMSG];
  App.SerialReceive -> Serial.Receive[AM_SERIALTORADIOMSG];
  App.AMControl -> Serial;
  App.Packet -> Serial;
  App.Leds -> LedsC;
  App.ReadLight -> TsrC;
  App.ReadTemp -> TempHumid.Temperature;
  App.ReadHumidity -> TempHumid.Humidity;
}
