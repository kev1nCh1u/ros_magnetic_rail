#!/usr/bin/env python
# license removed for brevity

#############################################################################
# python ros magnetic_rail
# by Kevin Chiu 2020
#############################################################################
import numpy as np
import serial
import time
import sys

import rospy
from std_msgs.msg import String
from magnetic_rail.msg import MrMsg

import threading

mr_msg = MrMsg()

############################################################################
# magnet function
############################################################################3
def magnetFuc():
    num = 0
    offset = 0
    width = 0

    data_raw = g_ser_magnet.read()

    # int_raw = int.from_bytes(data_raw, byteorder='big')
    int_raw = int(data_raw.encode('hex'), 16)

    num = int_raw//64
    id = int_raw % 64

    if(id == 57 and num == 1):
        data_raw1 = g_ser_magnet.read()
        data_raw2 = g_ser_magnet.read()
        # int_raw1 = int.from_bytes(data_raw1, byteorder='big')
        # int_raw2 = int.from_bytes(data_raw2, byteorder='big')
        int_raw1 = int(data_raw1.encode('hex'), 16)
        int_raw2 = int(data_raw2.encode('hex'), 16)
        P_N = (int_raw1 & 64) >> 6
        if(P_N == 1):
            offset = int_raw1 & 63
        else:
            offset = -(int_raw1 & 63)
        width = (((int_raw2 & 192) >> 2)+((int_raw2 & 63)-41))*2

    elif(id == 57 and num != 0):
        data_raw1 = g_ser_magnet.read()
        # int_raw1 = int.from_bytes(data_raw1, byteorder='big')
        int_raw1 = int(data_raw1.encode('hex'), 16)
        P_N = (int_raw1 & 64) >> 6
        if(P_N == 1):
            offset = int_raw1 & 63
        else:
            offset = -(int_raw1 & 63)

    # print("num:{} offset:{} width:{}".format(num, offset, width))
    return num, offset, width


##############################################################################
# mr_talker
###########################################################################
def mr_talker():
    global mr_msg

    # serial_work = threading.Thread(target=publisher_thread)
    # serial_work.start()

    pub = rospy.Publisher('mr_msg', MrMsg, queue_size=10)
    rospy.init_node('mr_talker', anonymous=True)
    # rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        
        num, offset, width = magnetFuc()
        # print("num:{} offset:{} width:{}".format(num, offset, width))
        
        mr_msg.num = num
        mr_msg.offset = offset
        mr_msg.width = width

        print(mr_msg.offset, "\t",mr_msg.width)
        pub.publish(mr_msg)

        # rate.sleep()

############################################################################
# main
#############################################################################
if __name__ == '__main__':
    try:
        input_argv = sys.argv
        input_port = input_argv[1]
        input_baudrate = input_argv[2]
        print('====== input setting ======')
    except:
        input_port = "/dev/ttyUSB0"
        input_baudrate = "115200"
        print('====== defalt setting ======')
    print("port: " + input_port)
    print("baudrate: " + input_baudrate)
    print('=========================')
    g_ser_magnet = serial.Serial(input_port, input_baudrate, bytesize=8,
                             parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)
    time.sleep(1)

    try:
        mr_talker()
    except rospy.ROSInterruptException:
        pass
