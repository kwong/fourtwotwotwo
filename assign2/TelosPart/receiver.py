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
    print time.strftime('[%X]') , \
        ('Light@A=%d' % m.get_light_a()) or '', \
        (msg_type == 1) and ('Humidity@A=%d' % m.get_humidity_a()) or '', \
        ('Temperature@A=%dC' % m.get_temp_a()) or '', \
        (msg_type == 2) and ('Light@B=%d' % m.get_light_b()) or '', \
        (msg_type == 2) and ('Temperature@B=%dC' % m.get_temp_b()) or ''

    
                                           
                                           


if __name__=='__main__':
 main(*sys.argv)
