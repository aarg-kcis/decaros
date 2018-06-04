#!/usr/bin/env python

import rospy
import DW1000
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import AnchorTimeStamps
from decaros.msg import TagTimeStamps
import time

Tag_seq 		= 0
Anchor_seq 		= 0
RATE            = 100
anchor_timemsg     = {}
tag_timemsg        = {}
round1=0
round1=0
reply1=0
reply2=0
current_sequence = 0
current_Tag_sequence = 0
current_Anchor_sequence = 0
calc_done_flag = [0]*256

def init():
    global sequence,current_sequence
    global timePollReceived, timePollAckSent, timeRangeReceived,timePollSent,timePollAckReceived,timeRangeSent
    current_sequence    = 0
    rospy.Subscriber('tag_timestamps', TagTimeStamps, TagTimeStampsCB)
    rospy.Subscriber('anchor_timestamps', AnchorTimeStamps, AnchorTimeStampsCB)

def TagTimeStampsCB(Time_msg):
    global tag_timemsg, current_Tag_sequence
    tag_timemsg[Time_msg.sequence] = Time_msg
    current_Tag_sequence = Time_msg.sequence
    # print tag_timemsg

def AnchorTimeStampsCB(Time_msg):
    global anchor_timemsg, current_Anchor_sequence
    anchor_timemsg[Time_msg.sequence]= Time_msg
    current_Anchor_sequence = Time_msg.sequence
    # print anchor_timemsg



def getrange():
    global current_sequence,tag_timemsg,anchor_timemsg,round1,round2,reply1,reply2,calc_done_flag
    # print "A :" , anchor_timemsg
    # print "T :" , tag_timemsg
    # else:
    #     current_sequence = min(current_Anchor_sequence,current_Tag_sequence)
    #     if current_sequence == 0 and 

    if current_Tag_sequence == current_Anchor_sequence :
        current_sequence = current_Anchor_sequence

    else : 
        current_sequence = min(current_Anchor_sequence,current_Tag_sequence)

    if len(tag_timemsg)!=0 and len(anchor_timemsg)!=0 and calc_done_flag[current_sequence]==0 and current_sequence in anchor_timemsg.keys() and current_sequence in tag_timemsg.keys():
        round1 = DW1000.wrapTimestamp(tag_timemsg[current_sequence].timePollAckReceived - tag_timemsg[current_sequence].timePollSent)
        reply1 = DW1000.wrapTimestamp(anchor_timemsg[current_sequence].timePollAckSent - anchor_timemsg[current_sequence].timePollReceived)
        round2 = DW1000.wrapTimestamp(anchor_timemsg[current_sequence].timeRangeReceived - anchor_timemsg[current_sequence].timePollAckSent)
        reply2 = DW1000.wrapTimestamp(tag_timemsg[current_sequence].timeRangeSent - tag_timemsg[current_sequence].timePollAckReceived)
        # time.sleep(5)
        print "round1: {}\t\t reply1:{}".format(round1, reply1)
        print "round2: {}\t\t reply2:{}".format(round2, reply2)
        # deletePreviousSequenceData()
        # if round1==0 and round2==0 and reply1==0 and reply2==0 :
        #     return 0
        range1 = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2)
        print ((range1 % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO)
        calc_done_flag[current_sequence] = 1

        if current_sequence == 255 : 
            calc_done_flag = [0]*256




def spin():
    rospy.loginfo("Finding Global Positions")
    rate = rospy.Rate(RATE)
    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        getrange()
        rate.sleep()
    rospy.spin()

def shutdown():
    rospy.loginfo("Shutting down.")
    rospy.sleep(1)

if __name__ == '__main__':
    init()
    rospy.init_node("range", anonymous=True)
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass
