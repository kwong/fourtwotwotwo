#!/usr/bin/env python

# Modified by Wong Kangwei
# for CS4222 Assignment 1

import os,sys,traceback,time
import string

sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF
import serial_msg

def main(*args):
  t = serial_send(args[1], args[2])
  
  leds = ['red', 'green', 'blue']
  mode = ['off', 'on']
  a = []
  b = []
  c = []

  # parse input
  for x in args[3:12]:
    e = x.strip().lower()
    if e in leds:
      a.append(leds.index(e))
    elif e in mode:
      b.append(mode.index(e))
    elif e.isdigit():
      c.append(int(x))
    else:
      print 'error parsing input'

  print a, b, c

  
  # sanity check
  assert len(a) == len(b) == len(c) 

  # fill blanks
  a = a + [-1]*(3-len(a));
  b = b + [-1]*(3-len(b));
  c = c + [-1]*(3-len(c));
  
  time.sleep(1)

  # send packet
  t.send(a.pop(),
         a.pop(),
         a.pop(),
         b.pop(),
         b.pop(),
         b.pop(),
         c.pop(),
         c.pop(),
         c.pop())

class serial_send():
  def __init__(self, host, port):
    self.mif = MoteIF.MoteIF()
    hostString = "%s%s" % ("sf@", host)
    moteString = "%s:%d" % (hostString, string.atoi(port))
    print(moteString)
    self.source = self.mif.addSource(moteString)
  
  def send(self, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9):
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

	  self.mif.sendMsg(self.source, 0x0, m.get_amType(), 0xFF, m)
          sys.exit(0)

if __name__=='__main__':
  main(*sys.argv)

