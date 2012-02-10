#include "Timer.h"

#define LED0DONE 0 
#define LED1DONE 1
#define LED2DONE 2
#define LED0OFF 3
#define LED1OFF 4
#define LED2OFF 5

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
  int red_on = 0;
  int blue_on = 0;
  int green_on = 0;

  
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

  void handleReply();
  task void handleRedReply();
  task void handleBlueReply();
  task void handleGreenReply();
  task void handleRedOff();
  task void handleBlueOff();
  task void handleGreenOff();

  
  

  
  /* Handle Timer fired() events */
  event void MilliTimer1.fired() {
    call Leds.led0Off();
    post handleRedReply();
    
  }

  event void MilliTimer2.fired() {
    call Leds.led1Off();
    post handleGreenReply();
  }

  event void MilliTimer3.fired() {
    call Leds.led2Off();
    post handleBlueReply();
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

  /* Handles reply whenever the OFF command is given */
  task void handleRedOff() {
    call Leds.led0Off();
    msg_code = LED0OFF;
    red_on = 0;
    if (locked) {
      post handleRedOff();
      return;
    }
    handleReply();
  }

  task void handleGreenOff() {
    call Leds.led1Off();
    msg_code = LED1OFF;
    green_on = 0;
    if (locked) {
      post handleGreenOff();
      return;
    }
    handleReply();
  }

  task void handleBlueOff() {
    call Leds.led2Off();
    msg_code = LED2OFF;
    blue_on = 0;
    if (locked) {
      post handleBlueOff();
      return;
    }
    handleReply();
  }


  /* Handles reply whenever a job of ON command with time duration has been completed */
  task void handleRedReply() {
    msg_code = LED0DONE;
    if (locked) {
      post handleRedReply();
      return;
    }
    if (red_on) {
      // reset state
      red_on = 0;
      handleReply();
    }
  }

  task void handleGreenReply() {
    msg_code = LED1DONE;
    if (locked) {
      post handleGreenReply();
      return;
    }
    if (green_on) {
      green_on = 0;
      handleReply();
    }
  }

  task void handleBlueReply() {
    msg_code = LED2DONE;
    if (locked) {
      post handleBlueReply();
      return;
    }
    if (blue_on) {
      blue_on = 0;
      handleReply();
    }
  }

  void handleReply()
  {
    atomic{
      if (locked) {
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
    red_on = 1;
    call MilliTimer1.startOneShot((int)(*duration)*1000U);

  }

  void handleGreenOn(nx_uint16_t *duration) {
    call Leds.led1On();
    green_on = 1;
    call MilliTimer2.startOneShot((int)(*duration)*1000U);
  }

  void handleBlueOn(nx_uint16_t *duration) {
    call Leds.led2On();
    blue_on = 1;
    call MilliTimer3.startOneShot((int)(*duration)*1000U);
  }

  
  void handleCommand(nx_uint16_t *led_code, nx_uint16_t *mode, nx_uint16_t *duration)
  {
    
    switch(*led_code) {

    case 0 : {

      if (*mode) {
	handleRedOn(duration);
      } else {
	post handleRedOff();
      }
      
    }break;
      
    case 1 : {

      if (*mode) {
	handleGreenOn(duration);
      } else {
	post handleGreenOff();
      }
      
    }break;
      
    case 2 :{
      if (*mode) {
	handleBlueOn(duration);
      } else {
	post handleBlueOff();
      }
      
    }break;
    }
  }
  
  /* Handles the event of receiving a packet */
  event message_t* SerialReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
    serial_msg_t* rcm = (serial_msg_t *)(payload);


    /* Handle control commands */
    handleCommand(&rcm->param_one, &rcm->param_four, &rcm->param_seven);
    handleCommand(&rcm->param_two, &rcm->param_five, &rcm->param_eight);
    handleCommand(&rcm->param_three, &rcm->param_six, &rcm->param_nine);
    
    return bufPtr;
  }
			
}
