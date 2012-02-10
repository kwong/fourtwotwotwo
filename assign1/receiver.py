#!/usr/bin/env python

import os,sys,traceback,time
import string

sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF
import serial_msg


def main(*args):
  t = serial_recv(args[1], args[2])

class serial_recv():
  def __init__(self, host, port):
    self.mif = MoteIF.MoteIF()
    hostString = "%s%s" % ("sf@", host)
    moteString = "%s:%d" % (hostString, string.atoi(port))
    print(moteString)
    self.source = self.mif.addSource(moteString)
    time.sleep(1)
    self.mif.addListener(self, serial_msg.serial_msg)


  def receive(self, src, m):
    
    param_1 = m.get_param_one()
    leds = {0:'Red LED', 1:'Green LED', 2:'Blue LED'}
    if param_1 in [0, 1, 2]:
      print time.strftime('%X %x %Z'), ":: Job on %s completed" % leds[param_1]
    elif param_1 in [3, 4, 5]:
      print time.strftime('%X %x %Z'), ":: %s turned off" % leds[param_1 - 3]
    


if __name__=='__main__':
 main(*sys.argv)
