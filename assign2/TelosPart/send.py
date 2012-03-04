#!/usr/bin/env python

import os,sys,traceback,time
import string

sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF

import SerialToRadioMsg
#import serial_msg

def main(*args):
    t = serial_send(args[1], args[2])
    time.sleep(1)

    t.send()
    
class serial_send():
    def __init__(self, host, port):
        self.mif = MoteIF.MoteIF()
        host_string = "%s%s" % ("sf@", host)
        mote_string = "%s:%d" % (host_string, string.atoi(port))
        print mote_string
        self.source = self.mif.addSource(mote_string)

    def send(self):
        m = SerialToRadioMsg.SerialToRadioMsg()
        m.set_type(0)
        self.mif.sendMsg(self.source, 0x0, m.get_amType(), 0xFF, m)



if __name__=='__main__':
    main(*sys.argv)

    
    
    
