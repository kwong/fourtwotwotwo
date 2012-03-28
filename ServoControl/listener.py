#!/usr/bin/env python

import os,sys,traceback,time
import string
from time import sleep

print os.getenv("USER")
print os.environ["TOSROOT"]
sys.path.append(os.path.join(os.environ["TOSROOT"], "support/sdk/python"))

import tinyos.message.MoteIF as MoteIF
import output_msg


def show_time():
    print time.strftime('%X %x %Z')
    

class serial_recv():
    def __init__(self, host, port):
        self.mif = MoteIF.MoteIF()
        self.controller = Controller()
        hostString = "%s%s" % ("sf@", "localhost")
        moteString = "%s:%d" % (hostString, 9000)
        self.source = self.mif.addSource(moteString)
        time.sleep(1)
        self.mif.addListener(self, output_msg.output_msg)



    def receive(self, src, msg):
        fn = {0:'WindDown', 1:'WindUp'}
        rcv_status = msg.get_status()
        if rcv_status == 1:
            self.controller.windMax()
            print show_time(), "Received Open Request" 
        elif rcv_status == 0:
            self.controller.windMin()
            print show_time(), "Received Close Request"
