#include "Timer.h"

module NetC {
  uses {
    interface Leds;
    interface Boot;
    interface Receive;
    interface AMSend;
    interface Timer<TMilli> as MilliTimer;
    interface SplitControl;
  }
}
implementation 
{
  message_t packet;

  event void Boot.booted() 
	{
		call SplitControl.start();
  }
  
	event void SplitControl.startDone(error_t err)
  {
		if(TOS_NODE_ID == 1) {
    	call MilliTimer.startPeriodic(2000U);
		}
  }

	event void MilliTimer.fired()
  {
		call AMSend.send(AM_BROADCAST_ADDR, &packet, 10);
	}

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
    call Leds.led1Toggle();
    return bufPtr;
  }
  
  event void AMSend.sendDone(message_t* bufPtr, error_t error)
  {
		call Leds.led0Toggle();
  }

  event void SplitControl.stopDone(error_t err) { }
}
