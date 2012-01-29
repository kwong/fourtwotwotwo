#!/usr/bin/env python

import os,sys,traceback,time
import string

sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF
import serial_msg

def main(*args):
  t = serial_send(args[1], args[2])
  time.sleep(1)
  t.send(string.atoi(args[3]), string.atoi(args[4]), string.atoi(args[5]), string.atoi(args[6]), string.atoi(args[7]), string.atoi(args[8]), string.atoi(args[9]), string.atoi(args[10]), string.atoi(args[11]), string.atoi(args[12]))

class serial_send():
  def __init__(self, host, port):
    self.mif = MoteIF.MoteIF()
    hostString = "%s%s" % ("sf@", host)
    moteString = "%s:%d" % (hostString, string.atoi(port))
    print(moteString)
    self.source = self.mif.addSource(moteString)
  
  def send(self, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10):
	  m = serial_msg.serial_msg()
	  m.set_param_one(arg1)
	  m.set_param_two(arg2)
	  m.set_param_three(arg3)
	  m.set_param_four(arg4)
	  m.set_param_five(arg5)
	  m.set_param_six(arg6)
	  m.set_param_seven(arg7)
	  m.set_param_eight(arg8)
	  m.set_param_nine(arg9)
	  m.set_param_ten(arg10)
	  self.mif.sendMsg(self.source, 0x0, m.get_amType(), 0xFF, m)

if __name__=='__main__':
 main(*sys.argv)
