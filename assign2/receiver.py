#!/usr/bin/env python

import os,sys,traceback,time
import string

sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF
import SerialToRadioMsg


def main(*args):
  t = serial_recv(args[1], args[2])

class serial_recv():
  def __init__(self, host, port):
    self.mif = MoteIF.MoteIF()
    host_string = "%s%s" % ("sf@", host)
    mote_string = "%s:%d" % (host_string, string.atoi(port))
    print(mote_string)
    self.source = self.mif.addSource(mote_string)
    time.sleep(1)
    self.mif.addListener(self, SerialToRadioMsg.SerialToRadioMsg)


  def receive(self, src, m):
    
    msg_type = m.get_type()
    print time.strftime('[%X]'), "Light=%d Humidity=%d Temperature=%dC" % (m.get_light_a(), m.get_humidity_a(), (m.get_temp_a()-3955)/100);


if __name__=='__main__':
 main(*sys.argv)
