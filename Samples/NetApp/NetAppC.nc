configuration NetAppC 
{
}
implementation {
  components MainC, NetC as App, LedsC;
  components ActiveMessageC;
  components new TimerMilliC();
    
  App.Boot -> MainC.Boot;
  App.Receive -> ActiveMessageC.Receive[240];
  App.AMSend -> ActiveMessageC.AMSend[240];
  App.SplitControl -> ActiveMessageC;
  App.Leds -> LedsC;
  App.MilliTimer -> TimerMilliC;
}


