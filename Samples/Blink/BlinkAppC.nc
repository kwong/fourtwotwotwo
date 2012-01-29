configuration BlinkAppC
{
}
implementation
{
  components LedsC, new TimerMilliC(), MainC;
  components BlinkC;

  BlinkC.Leds -> LedsC;
  BlinkC.Timer -> TimerMilliC;
  BlinkC.Boot -> MainC.Boot;
}

