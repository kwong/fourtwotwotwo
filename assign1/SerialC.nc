#include "Timer.h"

#define LED0DONE 0 
#define LED1DONE 1
#define LED2DONE 2

module SerialC {
  uses {
    interface Leds;
    interface Boot;
    interface AMSend as SerialSend;
    interface Receive as SerialReceive;
    interface Timer<TMilli> as MilliTimer1;
    interface Timer<TMilli> as MilliTimer2;
    interface Timer<TMilli> as MilliTimer3;
    interface SplitControl as AMControl;
    interface Packet;
  }
}
implementation {

  message_t packet;
  bool locked;
  uint32_t counter = 0;
  int msg_code = -1;
  
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

  task void handleReply();

  
  /* Handle Timer fired() events */
  event void MilliTimer1.fired() {
    call Leds.led0Off();
    msg_code = LED0DONE;
    post handleReply();
    
  }

  event void MilliTimer2.fired() {
    call Leds.led1Off();
    msg_code = LED1DONE;
    post handleReply();
  }

  event void MilliTimer3.fired() {
    call Leds.led2Off();
    msg_code = LED2DONE;
    post handleReply();
  }

  // END fired() handling

  
  event void SerialSend.sendDone(message_t* bufPtr, error_t error)
  {
    locked = FALSE;
  }


  void handleRedOn(nx_uint16_t *duration);
  void handleGreenOn(nx_uint16_t *duration);
  void handleBlueOn(nx_uint16_t *duration);
  void handleCommand(nx_uint16_t *led_code, nx_uint16_t *mode, nx_uint16_t *duration);



  task void handleReply()
  {
    atomic{
    if (locked) {
      post handleReply();
      return;
    }
    else {
      serial_msg_t* rcm = (serial_msg_t *)call Packet.getPayload(&packet, sizeof(serial_msg_t));
      if (rcm == NULL) {
	return;
      }
      counter++;
      rcm->param_one=msg_code;
      if (call SerialSend.send(126, &packet, sizeof(serial_msg_t)) == SUCCESS) {
	locked = TRUE;
      }
    }
    }
  }
  
  void handleRedOn(nx_uint16_t *duration) {
    call Leds.led0On();
    call MilliTimer1.startOneShot((int)(*duration)*1000U);

  }

  void handleGreenOn(nx_uint16_t *duration) {
    call Leds.led1On();
    call MilliTimer2.startOneShot((int)(*duration)*1000U);
  }

  void handleBlueOn(nx_uint16_t *duration) {
    call Leds.led2On();
    call MilliTimer3.startOneShot((int)(*duration)*1000U);
  }

  
  void handleCommand(nx_uint16_t *led_code, nx_uint16_t *mode, nx_uint16_t *duration)
  {
    
    switch(*led_code) {

    case 0 : {

      if (*mode) {
	handleRedOn(duration);
      } else {
	call Leds.led0Off();
      }
      
    }break;
      
    case 1 : {

      if (*mode) {
	handleGreenOn(duration);
      } else {
	call Leds.led1Off();
      }
      
    }break;
      
    case 2 :{
      if (*mode) {
	handleBlueOn(duration);
      } else {
	call Leds.led2Off();
      }
      
    }break;
    }
  }
  
  /* Handles the event of receiving a packet */
  event message_t* SerialReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
    serial_msg_t* rcm = (serial_msg_t *)(payload);
    /*if(rcm->param_one  && rcm->param_four) {
      call Leds.led0On();
      call MilliTimer1.startOneShot(rcm->param_seven*1000);
    }
    if(rcm->param_two  && rcm->param_five) {
      call Leds.led1On();
      call MilliTimer2.startOneShot(rcm->param_eight*1000);
    }
    if(rcm->param_three && rcm->param_six) {
      call Leds.led2On();
      call MilliTimer3.startOneShot(rcm->param_nine*1000);
    }
    */
    handleCommand(&rcm->param_one, &rcm->param_four, &rcm->param_seven);
    handleCommand(&rcm->param_two, &rcm->param_five, &rcm->param_eight);
    handleCommand(&rcm->param_three, &rcm->param_six, &rcm->param_nine);
    
    return bufPtr;
  }
			
}
