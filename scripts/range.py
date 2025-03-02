#!/usr/bin/env python

import rospy
import DW1000Constants as C
from Deca_device import DecaDevice
from decaros.msg import AnchorTimeStamps
from decaros.msg import TagTimeStamps
import time
import DW1000

Tag_seq         = 0
Anchor_seq      = 0
RATE            = 1000
round1=0
round1=0
reply1=0
reply2=0
current_sequence = 0
tag_timemsg = {}
anchor_timemsg = {}
sequence_over = {}


def init():
    global sequence,current_sequence
    global timePollReceived, timePollAckSent, timeRangeReceived,timePollSent,timePollAckReceived,timeRangeSent
    current_sequence    = 0
    rospy.Subscriber('tag_timestamps', TagTimeStamps, TagTimeStampsCB,  queue_size=1)
    rospy.Subscriber('anchor_timestamps', AnchorTimeStamps, AnchorTimeStampsCB, queue_size=1)



def TagTimeStampsCB(Time_msg):
    global tag_timemsg, anchor_timemsg
    tag_timemsg[Time_msg.id][Time_msg.anchor] = Time_msg
    # if anchor_timemsg[Time_msg.id][Time_msg.anchor]!=0 :
    #     if anchor_timemsg[Time_msg.id][Time_msg.anchor].sequence == tag_timemsg[Time_msg.id][Time_msg.anchor].sequence :
    #         getrange(Time_msg.id,Time_msg.anchor)
    

def AnchorTimeStampsCB(Time_msg):
    global anchor_timemsg, tag_timemsg
    anchor_timemsg[Time_msg.tag][Time_msg.id] = Time_msg
    # if tag_timemsg[Time_msg.tag][Time_msg.id]!=0 :
    #     if anchor_timemsg[Time_msg.tag][Time_msg.id].sequence == tag_timemsg[Time_msg.tag][Time_msg.id].sequence : 
    #         getrange(Time_msg.tag,Time_msg.id)

def wrapTimestamp(timestamp):
    """
    This function converts the negative values of the timestamp due to the overflow into a correct one.

    Args :
            timestamp : the timestamp's value you want to correct.
    
    Returns:
            The corrected timestamp's value.
    """
    if timestamp < 0:
        timestamp += C.TIME_OVERFLOW
    return timestamp

def getrange(tag_id,anchor_id):
    global current_sequence,tag_timemsg,anchor_timemsg,round1,round2,reply1,reply2,calc_done_flag,sequence_over
    current_Anchor = anchor_timemsg[tag_id][anchor_id]
    current_Tag = tag_timemsg[tag_id][anchor_id]
    current_sequence = current_Tag.sequence
    round1 = wrapTimestamp(current_Tag.timePollAckReceived - current_Tag.timePollSent)
    reply1 = wrapTimestamp(current_Anchor.timePollAckSent - current_Anchor.timePollReceived)
    round2 = wrapTimestamp(current_Anchor.timeRangeReceived - current_Anchor.timePollAckSent)
    reply2 = wrapTimestamp(current_Tag.timeRangeSent - current_Tag.timePollAckReceived)
    
    print "round1: {}\t\t reply1:{}".format(round1, reply1)
    print "round2: {}\t\t reply2:{}".format(round2, reply2)

    print "POLLS : {} \t\t POLLR : {}".format(current_Tag.timePollSent,current_Anchor.timePollReceived)
    print "PACKR : {} \t\t PACKS : {}".format(current_Tag.timePollAckReceived,current_Anchor.timePollAckSent)
    print "RSENT : {} \t\t RRCVD : {}".format(current_Tag.timeRangeSent,current_Anchor.timeRangeReceived)
    
    range1 = ((round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2))
    print current_sequence
    print range1
    print "Range between {} and {}".format(tag_id,anchor_id)
    print (range1 % C.TIME_OVERFLOW) * C.DISTANCE_OF_RADIO
    print "------"

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
    anchorList  = map(int, rospy.get_param("/range/anchors").split(","))
    tagList     = map(int, rospy.get_param("/range/tags").split(','))
    print("Anchors: {}".format(anchorList))
    print("Tags: {}".format(tagList))
    for i in tagList : 
        tag_timemsg[i] = {}
        anchor_timemsg[i] = {}
        sequence_over[i] = {}
    for i in tagList :
        for j in anchorList :
            tag_timemsg[i][j] = 0 
            anchor_timemsg[i][j] = 0
            sequence_over[i][j] = 0
    rospy.init_node("range", anonymous=True)
    try:
        spin()
    except rospy.ROSInterruptException:
        print("Closing...")
        pass
