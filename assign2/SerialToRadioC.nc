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
    interface AMSend as RadioSend;
    interface Receive as RadioReceive;
    interface Timer<TMilli> as WakeupTimer1;
    interface SplitControl as AMControl;
    interface SplitControl as AMControl2;
    interface Packet;
    interface AMPacket;
    interface Read<uint16_t> as ReadLIGHT;
    interface Read<uint16_t> as ReadTEMP;
    interface Read<uint16_t> as ReadHUMID;
  }
}

implementation {

  message_t packet, r_packet;
  bool locked;
  uint16_t temp = 0;
  uint16_t humidity = 0;
  uint16_t light = 0;
  uint16_t my_type = 0;
  uint16_t tempConv = 0;
  uint16_t humidConv = 0;

  /* For Node B to store information sent from Node A */
  uint16_t tempA = 0;
  uint16_t lightA = 0;



  void SendToUART(uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		  , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b);
  void readSensors();
  void handleNodeA();
  task void SendToRadio();
  
  event void Boot.booted() {
    call AMControl.start();
    call AMControl2.start();
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
  
  event void AMControl2.startDone(error_t err) {
        if (err == SUCCESS) {
    }
    else {
      call AMControl2.start();
    }
  }

  event void AMControl2.stopDone(error_t err) {
    // do nothing
    }

  event void SerialSend.sendDone(message_t* bufPtr, error_t error)
  {
    locked = FALSE;
  }

  event void RadioSend.sendDone(message_t* bufPtr, error_t error)
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
    light = data;
    switch (my_type) {
    case NODE_A: {
      call ReadHUMID.read();
    } break;
    case NODE_B: {
      
      if (my_type == NODE_A) tempConv = -38.4 + 0.0098 * temp; // convert temp before sending
      else tempConv = (temp- 0.40)/0.01953;
      SendToUART(&lightA, &tempA, NULL, &light, &tempConv, NULL);
    } break;
      
    }
  }

  event void ReadHUMID.readDone(error_t err, uint16_t data) {
    humidity = data;
    if (my_type == NODE_A) {// only interested
      tempConv = -38.4 + 0.0098 * temp;
      humidConv = -0.0000028*humidity*humidity + 0.0405*humidity-4;

      SendToUART(&light, &tempConv, &humidity, NULL, NULL, NULL);
      post SendToRadio();
    }
  }

  task void SendToRadio()
  {
    if (locked) {
      post SendToRadio();
      return;
    } else {
      SerialToRadioMsg* str = (SerialToRadioMsg*) (call Packet.getPayload(&r_packet, sizeof(SerialToRadioMsg)));

      if (str == NULL) {
	return; // nothing to send
      }
      str->type = NODE_A;
      str->light_a = light;
      str->temp_a = tempConv;

      if (call RadioSend.send(AM_BROADCAST_ADDR, &r_packet, sizeof(SerialToRadioMsg))==SUCCESS) {
	call Leds.led2Toggle();
	locked = TRUE;
	}
    } // if (locked)
  } // end SendToRadio
		   

  void SendToUART( uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		  , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b)
  {
    if (locked){
      return;
    } else {
      SerialToRadioMsg* str = (SerialToRadioMsg*)(call Packet.getPayload(&packet, sizeof(SerialToRadioMsg)));
      if (str == NULL) {
	return; // nothing to send
      }
      str->type = my_type;
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
   * > 2. Send to Node B via radio
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

  /* Messages received from Mote
   * > 1. Perform sensor jobs
   * > 2. Send received sensor data and own sensor data to USB
   */
  event message_t* RadioReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
    SerialToRadioMsg* rcm = (SerialToRadioMsg *)(payload);
    
    if (rcm->type != NODE_A) {
      return NULL;
    } else {
      call Leds.led1On();
      my_type = NODE_B; // acting as NodeB
      // read sensor readings
      tempA = rcm->temp_a;
      lightA = rcm->light_a;
      // engage own sensors
      readSensors();
      return bufPtr;
    }
  }
} // end implementation
