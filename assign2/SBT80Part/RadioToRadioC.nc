#include "Timer.h"
#include "SBT80ADCmap.h"

#define NODE_A 1
#define NODE_B 2

module RadioToRadioC {
  uses {
    interface Leds;
    interface Boot;
    interface AMSend as SerialSend;
    //    interface Receive as SerialReceive;
    interface AMSend as RadioSend;
    interface Receive as RadioReceive;
    // interface Timer<TMilli> as WakeupTimer1;
    interface SplitControl as AMControl;
    interface SplitControl as AMControl2;
    interface Packet;
    interface AMPacket;
    interface Read<uint16_t> as ReadLIGHT;
    interface Read<uint16_t> as ReadTEMP;
    interface HplMsp430GeneralIO as SBControl;
    interface HplMsp430GeneralIO as SBSwitch;
    
  }
}

implementation {

  message_t packet, r_packet;
  bool locked;
  //uint16_t temp = 0;
  //uint16_t humidity = 0;
  // uint16_t light = 0;
  uint16_t light = 0;
  uint16_t temp = 0;
  //  uint16_t humidity = 0;
  uint16_t my_type = 0;
  uint16_t tempConv = 0;
  uint16_t lightConv = 0;
  //uint16_t humidConv = 0;
  uint16_t dummy = 0;

  /* For Node B to store information sent from Node A */
  uint16_t tempA = 0;
  uint16_t lightA = 0;



  void SendToUART(uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		  , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b);
  void readSensors();
  void handleNodeA();
  
  event void Boot.booted() {
    call AMControl.start();
    call AMControl2.start();
    call SBControl.clr();
    call SBControl.makeOutput();
    call SBControl.selectIOFunc();
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
    tempConv = ((temp * 3.0/4096.0) - 0.4)/0.01953;
    lightConv = 3.0322 * light;
  
    SendToUART(&lightA, &tempA, &dummy, &lightConv, &tempConv, &dummy);
   
  }

  void SendToUART( uint16_t *light_a, uint16_t *temp_a, uint16_t *humidity_a
		   , uint16_t *light_b, uint16_t *temp_b, uint16_t *humidity_b)
  {
    if (locked){
      return;
    } else {
      radio_msg_t* str = (radio_msg_t*)(call Packet.getPayload(&packet, sizeof(radio_msg_t)));
      if (str == NULL) {
	return; // nothing to send
      }
      str->param_one = NODE_B;
      str->param_two = *light_a;
      str->param_three = *temp_a;
      str->param_four = *humidity_a;
      str->param_five = *light_b;
      str->param_six = *temp_b;
      str->param_seven = *humidity_b;

      if (call SerialSend.send(126, &packet, sizeof(radio_msg_t))==SUCCESS) {
	locked = TRUE;
      }
    }  // end if (locked)       
  } // end SendToUART


  void readSensors() {
    call ReadTEMP.read(); // start call chain
  }

  /*
    event void WakeupTimer1.fired()
    {
    readSensors();
    }*/
  
  
  /* Messages received from Serial
   * > 1. Perform sensor jobs
   * > 2. Send to Node B via radio
   */
  /*  event message_t* SerialReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
      {
   
      radio_msg_t* rcm = (radio_msg_t *)(payload);
      call Leds.led0On();

      if (rcm->type != FROMPC) { 		        // Only accept messages from PC
      return NULL;
      } else {
      
      my_type = NODE_A; // acting as NodeA
      
      call WakeupTimer1.startPeriodic(5000U);
      return bufPtr;  
      }
      }*/

  /* Messages received from Mote
   * > 1. Perform sensor jobs
   * > 2. Send received sensor data and own sensor data to USB
   */
  event message_t* RadioReceive.receive(message_t* bufPtr, void* payload, uint8_t len)
  {
    radio_msg_t* rcm = (radio_msg_t *)(payload);
    
    //if (rcm->type != NODE_A) {
    //  return NULL;
    //} else {
      call Leds.led1On();
      my_type = NODE_B; // acting as NodeB
      tempA = rcm->param_four;
      lightA = rcm->param_two;
      // engage own sensors
      readSensors();
      return bufPtr;
      //}
  }
} // end implementation
