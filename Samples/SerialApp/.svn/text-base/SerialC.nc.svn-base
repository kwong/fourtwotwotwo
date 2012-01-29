#include "Timer.h"
 
module SerialC {
  uses {
    interface Leds;
    interface Boot;
    interface AMSend as SerialSend;
		interface Receive as SerialReceive;
    interface Timer<TMilli> as MilliTimer;
    interface SplitControl as AMControl;
    interface Packet;
  }
}
implementation {

  message_t packet;

  bool locked;
  uint32_t counter = 0;
  
  event void Boot.booted() {
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }
 
	void sendMessage(); 
  
	event void MilliTimer.fired() {
		sendMessage();
	}
	
	void sendMessage()
	{
  	if (locked) {
			return;
    }
    else {
     	serial_msg_t* rcm = (serial_msg_t *)call Packet.getPayload(&packet, sizeof(serial_msg_t));
     	if (rcm == NULL) {
				return;
     	}
 			counter++;
     	rcm->param_one=1;
     	rcm->param_two=2;
     	rcm->param_three=3;
     	rcm->param_four=4;
     	rcm->param_five=5;
     	rcm->param_six=6;
     	rcm->param_seven=7;
     	rcm->param_eight=8;
     	rcm->param_nine=9;
     	rcm->param_ten=10;
     	if (call SerialSend.send(126, &packet, sizeof(serial_msg_t)) == SUCCESS) {
				locked = TRUE;
     	}
    }
  }

  event void SerialSend.sendDone(message_t* bufPtr, error_t error)
	{
		call Leds.led0Toggle();
		locked = FALSE;
  }

	event message_t* SerialReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
	{
		serial_msg_t* rcm = (serial_msg_t *)(payload);
		if(rcm->param_ten == 10) {
			call Leds.led1Toggle();
      call MilliTimer.startPeriodic(1024);
		}
		return bufPtr;
	}
			
}
