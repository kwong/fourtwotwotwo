#include "Timer.h"

#define FROMPC 0
#define NODE_A 1
#define NODE_B 2

module SerialToRadioC {
  uses {
    interface Leds;
    interface Boot;
    interface AMSend as SerialSend;
    interface Receive as SerialReceive;
    interface Timer<TMilli> as WakeupTimer1;
    interface SplitControl as AMControl;
    interface Packet;
    interface Read<uint16_t> as ReadLIGHT;
    interface Read<uint16_t> as ReadTEMP;
    interface Read<uint16_t> as ReadHUMID;
  }
}

implementation {

  message_t packet;
  bool locked;
  uint16_t temp = 0;
  uint16_t humidity = 0;
  uint16_t light = 0;
  uint16_t my_type = 0;


  void SendToUART(uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		  , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b);
  void readSensors();
  void handleNodeA();
  
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

  event void SerialSend.sendDone(message_t* bufPtr, error_t error)
  {
    locked = FALSE;
  }

  event void ReadTEMP.readDone(error_t err, uint16_t data)
  {
    temp = data;
    call ReadLIGHT.read();
  }

  event void ReadLIGHT.readDone(error_t err, uint16_t data)
  {
    humidity = data;
    call ReadHUMID.read();
  }

  event void ReadHUMID.readDone(error_t err, uint16_t data) {
    light = data;
    switch (my_type) {// only interested
    case NODE_A: {
      SendToUART(&light, &temp, &humidity, 0, 0, 0);
      // SEND TO RADIO
    }  break;
    case NODE_B: {
      // DO CONVERT
      // Send to USB
    }  break;
      
    }    
  }  

  void SendToUART(uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		  , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b)
  {
    if (locked){
      return;
    } else {
      SerialToRadioMsg* str = (SerialToRadioMsg*)(call Packet.getPayload(&packet, sizeof(SerialToRadioMsg)));
      if (str == NULL) {
	return; // nothing to send
      }
      str->light_a    = *light_a;
      str->temp_a     = *temp_a;
      str->humidity_a = *humidity_a;
      str->light_b    = *light_b;
      str->temp_b     = *temp_b;
      str->humidity_b = *humidity_b;

      if (call SerialSend.send(126, &packet, sizeof(SerialToRadioMsg))==SUCCESS) {
	locked = TRUE;
      }
    }  // end if (locked)       
  } // end SendToUART


  void readSensors() {
    call ReadTEMP.read(); // start call chain
  }

  
  event void WakeupTimer1.fired()
  {
    readSensors();
  }
  
  
  /* Messages received from Serial
   * > 1. Perform sensor jobs
   * > 2. Send to neighboring motes
   */
  event message_t* SerialReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
   
    SerialToRadioMsg* rcm = (SerialToRadioMsg *)(payload);
    call Leds.led0On();

     if (rcm->type != FROMPC) { 		        // Only accept messages from PC
      return NULL;
    } else {
      
      my_type = NODE_A; // acting as NodeA
      
      call WakeupTimer1.startPeriodic(5000U);
      return bufPtr;  
      }
    } 
} // end implementation
